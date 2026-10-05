#include "trajectory/apf_seeder.hpp"

#include <cmath>

namespace drone_core::trajectory {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Discrete 4-Sector Z-Stratification: replaces the earlier continuous H*sin(theta) modulation, which
// cancels to ~0 exactly at due-East/West headings (theta ~= 0, pi) and left
// those head-on flows with no altitude separation at all. Splits the full
// horizontal heading circle into 4 orthogonal 90-degree sectors and assigns
// each an independent, fixed altitude band (in units of `step_m`, so the
// default 0.75m step gives +-0.75m/+-2.25m bands) — any two
// drones whose headings fall in different sectors (in particular any
// head-on or 90-degree-crossing pair) are separated by at least one full
// step before the SCP solver ever starts.
double get_sector_z_offset(double theta_rad, double step_m) {
    const double angle = std::atan2(std::sin(theta_rad), std::cos(theta_rad));  // normalize to [-pi, pi]
    constexpr double kQuarterPi = kPi / 4.0;
    if (angle >= -kQuarterPi && angle < kQuarterPi) {
        return 3.0 * step_m;  // +2.25 m (East)
    } else if (angle >= kQuarterPi && angle < 3.0 * kQuarterPi) {
        return 1.0 * step_m;  // +0.75 m (North)
    } else if (angle >= -3.0 * kQuarterPi && angle < -kQuarterPi) {
        return -3.0 * step_m;  // -2.25 m (South)
    } else {
        return -1.0 * step_m;  // -0.75 m (West)
    }
}

}  // namespace

std::vector<Eigen::MatrixXd> seed_control_points_with_apf(const std::vector<BoundaryConditions>& starts,
                                                           const std::vector<BoundaryConditions>& ends,
                                                           double duration, int num_control_points,
                                                           const ApfSeedingConfig& config) {
    const int num_drones = static_cast<int>(starts.size());
    std::vector<Eigen::MatrixXd> result(num_drones);
    for (int i = 0; i < num_drones; ++i) {
        result[i] = seed_control_points(starts[i], ends[i], duration, num_control_points);
    }

    const int free_begin = 3;
    const int free_end = num_control_points - 3;  // exclusive
    const int num_free = free_end - free_begin;
    if (num_free <= 0 || !config.enabled || num_drones == 0) {
        return result;  // no free interior points to seed, or APF disabled
    }

    const int m = num_control_points - 1;

    // r[i][k] is drone i's warm-start position for the free control point at
    // index (free_begin + k), k in [0, num_free) -- step 1's
    // r_i^(0)(t_k), sampled at that control point's own nominal parametric
    // fraction rather than a fixed K_sample=8 grid (see header comment).
    std::vector<double> tau(num_free);
    for (int k = 0; k < num_free; ++k) {
        tau[k] = static_cast<double>(free_begin + k) / static_cast<double>(m);
    }

    const bool z_stratification_enabled = config.z_stratification_mode == "4_sector_discrete";

    // A stationary (parked) drone keeps its constant seed: no altitude band
    // (atan2(0, 0) = 0 would otherwise lift it into the East band) and no
    // Euler steps, while still repelling the drones around it.
    std::vector<char> fixed(num_drones);
    for (int i = 0; i < num_drones; ++i) fixed[i] = is_stationary_hold(starts[i], ends[i]);

    std::vector<std::vector<Eigen::Vector3d>> r(num_drones, std::vector<Eigen::Vector3d>(num_free));
    for (int i = 0; i < num_drones; ++i) {
        if (fixed[i]) {
            for (int k = 0; k < num_free; ++k) r[i][k] = result[i].row(free_begin + k).transpose();
            continue;
        }
        const Eigen::Vector3d d = ends[i].position - starts[i].position;
        // Discrete 4-Sector Z-Stratification (step 2): head-on horizontal
        // flows (theta_i measured from the +x axis) get lifted into a
        // direction-dependent, fixed-band altitude offset (see
        // get_sector_z_offset() above), peaking mid-transition
        // (sin(pi*tau)) and vanishing at both pinned endpoints.
        const double theta_i = std::atan2(d.y(), d.x());
        const double z_offset = z_stratification_enabled ? get_sector_z_offset(theta_i, config.z_layer_step_m) : 0.0;
        for (int k = 0; k < num_free; ++k) {
            Eigen::Vector3d p = starts[i].position + tau[k] * d;
            p.z() += z_offset * std::sin(kPi * tau[k]);
            r[i][k] = p;
        }
    }

    // Euler-integrated pairwise Coulomb repulsion (step 3): each free-point
    // "slice" k is its own independent 3D particle system -- drone i's k-th
    // warm-start point only repels/is repelled by other drones' k-th
    // warm-start point (matching parametric progress, not absolute time,
    // since mega-cluster sub-stage decomposition can already shift a
    // drone's real per-window timing independently of this seed).
    //
    // FIXED numerical-blowup bug (found and fixed 2026-09-18): the force
    // magnitude k_rep*(1/dist - 1/R_det)/dist^2 behaves like k_rep/dist^3 as
    // dist -> 0, which is essentially unbounded for any dist just above the
    // dist_sq < 1e-12 exact-coincidence guard below -- that guard only
    // prevents literal division by zero, not the near-singular blowup for
    // small-but-nonzero separations. Root-caused on the real 300-drone
    // Phase 1 export (`C:\Users\brake\Local\Temp\intermediate_export-1.json`)
    // via direct instrumentation: a single drone's seeded control point
    // reached ~8982 m from origin (a plausible show position is within tens
    // of meters), which then produced an astronomically large spline
    // bounding box and, downstream, an equally astronomical voxel range in
    // collision::SpatioTemporalHash::insert()'s voxel_keys_for_bbox() --
    // this, not that function itself, was the real driver of the
    // multi-gigabyte memory growth this class of real-world dense scenario
    // produced (see spatio_temporal_hash.hpp's class comment for the
    // separate, genuine rescan-redundancy fix that this one complements).
    // Fixed with the standard N-body-integrator safety valve: clamp each
    // point's per-step displacement magnitude, so no single Euler step can
    // move a point further than `kMaxStepDisplacementM` regardless of how
    // large the (possibly near-singular) force is. Direction is preserved;
    // only magnitude is capped. Chosen as `r_det` itself (already the
    // config's own characteristic length scale for this simulation) so a
    // legitimately congested pair can still be pushed a full detection-
    // radius apart in one step -- generous for the seed's actual purpose
    // (nudging a few meters to escape a bad local geometry), while bounding
    // worst-case cumulative displacement to num_euler_steps * r_det instead
    // of unbounded.
    const double r_det = config.detection_radius_m;
    const double r_det_sq = r_det * r_det;
    const double kMaxStepDisplacementM = r_det;
    for (int step = 0; step < config.num_euler_steps; ++step) {
        std::vector<std::vector<Eigen::Vector3d>> force(num_drones,
                                                          std::vector<Eigen::Vector3d>(num_free, Eigen::Vector3d::Zero()));
#pragma omp parallel for schedule(dynamic)
        for (int k = 0; k < num_free; ++k) {
            // Every thread owns a disjoint slice k exclusively (both reading
            // r[*][k] and writing force[*][k]), so no cross-thread race.
            for (int i = 0; i < num_drones; ++i) {
                for (int j = i + 1; j < num_drones; ++j) {
                    const Eigen::Vector3d d_ij = r[i][k] - r[j][k];
                    const double dist_sq = d_ij.squaredNorm();
                    if (dist_sq >= r_det_sq || dist_sq < 1e-12) continue;
                    const double dist = std::sqrt(dist_sq);
                    // F_rep,ij = k_rep*(1/dist - 1/R_det) * d_ij/dist^3
                    const double magnitude = config.k_repulsion * (1.0 / dist - 1.0 / r_det) / dist_sq;
                    const Eigen::Vector3d f = magnitude * (d_ij / dist);
                    force[i][k] += f;
                    force[j][k] -= f;  // Newton's third law
                }
            }
        }
        for (int i = 0; i < num_drones; ++i) {
            if (fixed[i]) continue;
            for (int k = 0; k < num_free; ++k) {
                Eigen::Vector3d step_vec = config.euler_dt * force[i][k];
                const double step_norm = step_vec.norm();
                if (step_norm > kMaxStepDisplacementM) {
                    step_vec *= (kMaxStepDisplacementM / step_norm);
                }
                r[i][k] += step_vec;
            }
        }
    }

    for (int i = 0; i < num_drones; ++i) {
        if (fixed[i]) continue;
        for (int k = 0; k < num_free; ++k) {
            result[i].row(free_begin + k) = r[i][k].transpose();
        }
    }
    return result;
}

}  // namespace drone_core::trajectory
