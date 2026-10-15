#ifndef ADAPTIVE_HARDWARECONFIG_H
#define ADAPTIVE_HARDWARECONFIG_H
#pragma once
// Select the physical rig: 0=A4988 shield, 1=HY-DIV268N external module.
// Pass the SAME macros to all translation units via compiler.cpp.extra_flags.
#ifndef ADAPTIVE_DRIVER_PROFILE
#define ADAPTIVE_DRIVER_PROFILE 1
#endif
#ifndef ADAPTIVE_MICROSTEPS
#define ADAPTIVE_MICROSTEPS 8
#endif
#ifndef ADAPTIVE_COMMON_ANODE
#define ADAPTIVE_COMMON_ANODE 0
#endif
#ifndef ADAPTIVE_HOLD_AT_ZERO
#define ADAPTIVE_HOLD_AT_ZERO 0
#endif
#ifndef ADAPTIVE_CONTROL_DIAGNOSTICS
#define ADAPTIVE_CONTROL_DIAGNOSTICS 1
#endif
#if (ADAPTIVE_HOLD_AT_ZERO != 0 && ADAPTIVE_HOLD_AT_ZERO != 1) ||                                  \
    (ADAPTIVE_CONTROL_DIAGNOSTICS != 0 && ADAPTIVE_CONTROL_DIAGNOSTICS != 1)
#error "Holding and diagnostics switches must be 0 or 1."
#endif
#if ADAPTIVE_DRIVER_PROFILE != 0 && ADAPTIVE_DRIVER_PROFILE != 1
#error "ADAPTIVE_DRIVER_PROFILE must be 0 (A4988) or 1 (HY-DIV268N)."
#endif
#if ADAPTIVE_COMMON_ANODE != 0 && ADAPTIVE_COMMON_ANODE != 1
#error "ADAPTIVE_COMMON_ANODE must be 0 or 1."
#endif
#if ADAPTIVE_DRIVER_PROFILE == 0 && ADAPTIVE_COMMON_ANODE
#error "A4988 uses native logic, not the external module's optocoupler interface."
#endif
namespace adaptive_hardware {
#if ADAPTIVE_DRIVER_PROFILE == 0
constexpr int stepPin = 11, directionPin = 12, enablePin = 7, ms1Pin = 8, ms2Pin = 9, ms3Pin = 10;
constexpr int controlAccelerationScale = 40; // rad/s^2, established A4988 rig policy
constexpr int wheelSpeedGuard = 40; // rad/s, established A4988 rig policy
#else
constexpr int stepPin = 9, directionPin = 10, enablePin = 8, ms1Pin = 7, ms2Pin = 6, ms3Pin = 5;
constexpr int controlAccelerationScale = 100; // rad/s^2, operator-selected HY-DIV268N rig policy
constexpr int wheelSpeedGuard = 100; // rad/s, retained after the 200-scale nominal-cycle test
#endif
// Deployment tuning, not an identified or independently measured actuator capability.
// Identification excitation/braking and wheel-speed guards remain separate policies.
static_assert(controlAccelerationScale > 0, "Control acceleration scale must be positive.");
static_assert(wheelSpeedGuard > 0, "Wheel-speed guard must be positive.");
constexpr int microsteps = ADAPTIVE_MICROSTEPS;
constexpr bool commonAnode = ADAPTIVE_COMMON_ANODE != 0;
constexpr bool holdAtZero = ADAPTIVE_HOLD_AT_ZERO != 0;
constexpr int disabledLevel = commonAnode ? 0 : 1;
template <int Profile, int Microsteps, int CommonAnode, int Hold, int Diagnostics>
struct ConfigurationTag {};
using BuildConfiguration =
    ConfigurationTag<ADAPTIVE_DRIVER_PROFILE, ADAPTIVE_MICROSTEPS, ADAPTIVE_COMMON_ANODE,
                     ADAPTIVE_HOLD_AT_ZERO, ADAPTIVE_CONTROL_DIAGNOSTICS>;
void verifyBuildConfiguration(BuildConfiguration *);
static_assert(stepPin != directionPin && stepPin != enablePin && directionPin != enablePin,
              "STEP, DIR and EN must be distinct.");
static_assert(ms1Pin != stepPin && ms1Pin != directionPin && ms1Pin != enablePin &&
                  ms2Pin != stepPin && ms2Pin != directionPin && ms2Pin != enablePin &&
                  ms3Pin != stepPin && ms3Pin != directionPin && ms3Pin != enablePin,
              "Microstep/DIAG pins must not release an actuator output.");
} // namespace adaptive_hardware
#endif // ADAPTIVE_HARDWARECONFIG_H
