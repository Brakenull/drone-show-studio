#pragma once

#include <vector>

#include <Eigen/Dense>

#include "config.hpp"
#include "trajectory/quintic_bspline.hpp"

// APF Warm-Start Seeding & Directional Z-Stratification (docs/2-phase_2.md
// Rev 2.9 sections 1.2/3.2): replaces Rev 2.4-2.7's drone_id-parity bow
// heuristic (trajectory/heuristic_seeder.hpp, retired in Rev 2.8).
//
// INCIDENT THIS REPLACES: the parity bow only ever split drones into two
// groups (even/odd drone_id), which is enough to break a single isolated
// pairwise 2-drone crossing but not a real N-way simultaneous squeeze --
// confirmed on a real 300-drone Phase 1 export (~692 violating pairs, some
// drones passing within 0.02 m) and a synthetic N-drone "ring swap" stress
// test (N=20 settled at 1.02 m, N=40 at 0.20-1.2 m, both below the 1.5 m
// minimum), reproducing regardless of SCP iteration budget, slack weight,
// collision time-window density, or graph-coloring vs. sequential solve
// order -- i.e. a seeding/local-optimum problem, not a solver-tuning one.
//
// APF instead runs a joint, all-drones-at-once Coulomb-repulsion particle
// simulation over each drone's nominal straight-line path before the SCP
// ever starts, so a crowded region "balloons" outward into a genuinely
// separated 3D volume from iteration 0.
//
// Rev 2.8's Directional Z-Stratification lifted head-on horizontal flows by
// H*sin(theta) -- which cancels to ~0 exactly at due-East/West headings
// (theta ~= 0, pi), leaving those flows with no altitude separation at all.
// Rev 2.9 section 1.2 replaces it with a Discrete 4-Sector assignment (see
// get_sector_z_offset() in apf_seeder.cpp): every heading falls into one of
// 4 orthogonal 90-degree sectors, each with its own fixed, nonzero altitude
// band, so no heading (including due-East/West) is ever left unstratified.
//
// Rev 2.8's own verified impact substantially raised the fleet size that
// solves safely but did not fully clear the doc's own N=40 acceptance bar,
// and root-caused the residual gap to solve_single_stage()'s best-iterate
// worst-case tracking under-reporting the true continuous-time minimum
// between coarse per-window collocation samples (see
// stage2_nway_conflict_limitation memory). Rev 2.9 sections 1.9/1.10 close
// that gap from two directions: Adaptive Cutting-Plane Collocation
// (scp_solver.cpp) inserts extra hard collocation rows exactly at detected
// near-miss extrema so the *optimizer* reacts to them, and the Decoupled
// Continuous Gatekeeper (scp_solver.cpp's evaluate_continuous_clearance())
// independently re-verifies the final splines at 100Hz without reusing any
// of the solver's own per-iteration bookkeeping.
//
// Unlike seed_control_points_with_bow() (called once per drone,
// independently), this runs once per solve_single_stage() call across every
// drone in that stage together, since the repulsion term is inherently an
// N-body interaction.

namespace drone_core::trajectory {

// Runs the APF warm-start simulation for every drone in `starts`/`ends`
// (same size, index-aligned) sharing one `duration` and `num_control_points`
// (both uniform within a single solve_single_stage() call), and returns each
// drone's full seeded control-point matrix: the same 6 boundary-pinned
// points seed_control_points() would produce, with the (num_control_points -
// 6) free interior points replaced by the APF-settled positions.
//
// Sampling deviates from the doc's literal fixed K_sample=8: each drone is
// sampled at exactly `num_control_points - 6` fractions (one per free
// interior control point, at that point's own nominal parametric fraction
// k/(num_control_points-1), matching seed_control_points_with_bow()'s old
// convention) rather than a fixed 8, so the settled APF positions map
// directly onto control points with no separate curve-fitting step --
// reducing to the doc's own K_sample=8 exactly when num_control_points=14
// (8 free interior points, the doc's own Holding Area -> Square Test scale).
//
// If `config.enabled` is false, returns the plain (unseeded, linear-interior)
// seed_control_points() result for every drone.
std::vector<Eigen::MatrixXd> seed_control_points_with_apf(const std::vector<BoundaryConditions>& starts,
                                                           const std::vector<BoundaryConditions>& ends,
                                                           double duration, int num_control_points,
                                                           const ApfSeedingConfig& config);

}  // namespace drone_core::trajectory
