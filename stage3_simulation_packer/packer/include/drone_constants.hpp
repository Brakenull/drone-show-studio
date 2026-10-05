#pragma once

// Tier 3 of the drone_profile.json fallback chain:
//   --profile <path>  ->  config/drone_profile.json  ->  these constants.
//
// Every profile-backed constant carries a trailing `// profile: <json.key.path>`
// tag. warp_sim/profile.py parses this header (not a hand-kept Python copy)
// to build its own tier-3 defaults, so the tag format is load-bearing: one
// constant per line, value terminated by `;`, tag on the same line.

#include <array>
#include <cstdint>

namespace drone_constants {

inline constexpr const char* kProfileName = "Standard_Show_Quad_v1";  // profile: profile_name

// physical
inline constexpr double kMassKg = 0.55;                                              // profile: physical.mass_kg
inline constexpr double kArmLengthM = 0.16;                                          // profile: physical.arm_length_m
inline constexpr double kPropellerDiameterM = 0.127;                                 // profile: physical.propeller_diameter_m
inline constexpr std::array<double, 3> kInertiaDiagKgm2{0.0025, 0.0025, 0.0045};     // profile: physical.inertia_diag_kgm2
inline constexpr double kDragCoefficientCd = 0.45;                                   // profile: physical.drag_coefficient_cd
inline constexpr double kReferenceAreaM2 = 0.035;                                    // profile: physical.reference_area_m2

// motor_prop
inline constexpr double kMaxThrustPerMotorN = 3.2;                                   // profile: motor_prop.max_thrust_per_motor_n
inline constexpr double kThrustToWeightRatio = 2.37;                                 // profile: motor_prop.thrust_to_weight_ratio
inline constexpr double kMotorTimeConstantMs = 20.0;                                 // profile: motor_prop.motor_time_constant_ms
inline constexpr double kKvRating = 2300.0;                                          // profile: motor_prop.kv_rating
inline constexpr double kRotorTorqueToThrustM = 0.006;                               // profile: motor_prop.rotor_torque_to_thrust_m

// battery
inline constexpr int kCellsS = 4;                                                    // profile: battery.cells_s
inline constexpr double kCapacityMah = 2200.0;                                       // profile: battery.capacity_mah
inline constexpr double kNominalVoltageV = 14.8;                                     // profile: battery.nominal_voltage_v
inline constexpr double kCutoffVoltageV = 13.6;                                      // profile: battery.cutoff_voltage_v
inline constexpr double kInternalResistanceOhm = 0.035;                              // profile: battery.internal_resistance_ohm
inline constexpr double kLedMaxPowerW = 18.0;                                        // profile: battery.led_max_power_w
inline constexpr double kAvionicsCurrentA = 0.35;                                    // profile: battery.avionics_current_a
inline constexpr double kPowertrainEfficiency = 0.8;                                 // profile: battery.powertrain_efficiency
inline constexpr double kPeukertExponent = 1.05;                                     // profile: battery.peukert_exponent
inline constexpr double kInitialSoc = 1.0;                                           // profile: battery.initial_soc

// controller_gains
inline constexpr std::array<double, 3> kPosP{2.5, 2.5, 3.0};                         // profile: controller_gains.pos_p
inline constexpr std::array<double, 3> kVelP{1.8, 1.8, 2.2};                         // profile: controller_gains.vel_p
inline constexpr std::array<double, 3> kVelI{0.4, 0.4, 0.8};                         // profile: controller_gains.vel_i
inline constexpr std::array<double, 3> kAttP{6.5, 6.5, 4.0};                         // profile: controller_gains.att_p
inline constexpr std::array<double, 3> kRateP{18.0, 18.0, 10.0};                     // profile: controller_gains.rate_p
inline constexpr double kMaxTiltDeg = 35.0;                                          // profile: controller_gains.max_tilt_deg

// tolerances (Monte Carlo spread)
inline constexpr double kTolMassPct = 3.0;                                           // profile: tolerances.mass_pct
inline constexpr double kTolMaxThrustPct = 5.0;                                      // profile: tolerances.max_thrust_pct
inline constexpr double kTolMotorTimeConstantPct = 20.0;                             // profile: tolerances.motor_time_constant_pct
inline constexpr double kTolDragCoefficientPct = 10.0;                               // profile: tolerances.drag_coefficient_pct
inline constexpr double kTolBatteryCapacityPct = 5.0;                                // profile: tolerances.battery_capacity_pct
inline constexpr double kTolInternalResistancePct = 15.0;                            // profile: tolerances.internal_resistance_pct
inline constexpr double kTolInitialSocMin = 0.95;                                    // profile: tolerances.initial_soc_min
inline constexpr double kTolGnssRtkNoiseM = 0.02;                                    // profile: tolerances.gnss_rtk_noise_m
inline constexpr double kTolGnssDriftRateMPerSqrtS = 0.01;                           // profile: tolerances.gnss_drift_rate_m_per_sqrt_s
inline constexpr double kTolInitialPositionErrorM = 0.05;                            // profile: tolerances.initial_position_error_m

// environment: the rain rule. Placeholders until the drone
// model's water protection rating is known.
inline constexpr double kRainAlertMmH = 0.5;                                         // profile: environment.rain_alert_mm_h
inline constexpr double kRainLimitMmH = 2.5;                                         // profile: environment.rain_limit_mm_h
inline constexpr double kReturnReactionS = 5.0;                                      // profile: environment.return_reaction_s

// Not profile-backed: fixed by the flight file format / safety standard.
inline constexpr std::uint16_t kSamplingDtMs = 50;   // 20 Hz waypoint rate
inline constexpr double kNominalSeparationM = 1.5;   // d_min, planned by Phase 2
inline constexpr double kCrashFloorM = 0.5;          // d_crash, propeller contact
inline constexpr double kBufferWarningM = 1.0;       // [d_crash, 1.0) -> Warning
inline constexpr double kMinLandingSoc = 0.15;       // battery pass criterion

}  // namespace drone_constants
