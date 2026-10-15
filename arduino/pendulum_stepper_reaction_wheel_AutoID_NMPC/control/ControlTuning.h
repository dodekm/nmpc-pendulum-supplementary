#ifndef ADAPTIVE_CONTROLTUNING_H
#define ADAPTIVE_CONTROLTUNING_H
#pragma once
#include "Precision.h"
#include <cstdint>

namespace pendulum {
// Scalar solver/supervisor policy; dimensional penalties are model-normalized.
// The existing firmware's fixed coefficient/terminal configuration is not used.
// Changes here are tuning changes and require closed-loop regression evidence.
template <class Scalar> struct ControlTuning {
  static constexpr Scalar energyBlendLow = Scalar(.002);
  static constexpr Scalar angleBlendLowDegrees = Scalar(3);
  static constexpr Scalar disturbanceLeanLimitDegrees = Scalar(2);
  static constexpr Scalar captureMembership = Scalar(.95);
  static constexpr Scalar captureAngularSpeed = Scalar(.6);
  static constexpr uint32_t captureSamples = 3; // ceil(0.3 s / 0.1 s), MATLAB parity
  static constexpr Scalar lostCaptureAngleDegrees = Scalar(30);
  static constexpr Scalar lostCaptureRelativeEnergy = Scalar(.1);
  static constexpr Scalar unloadingMembership = Scalar(.99);
  static constexpr Scalar unloadingAngleDegrees = Scalar(1.5);
  static constexpr Scalar unloadingAngularSpeed = Scalar(.2);
  // Per-sample model-error variances, selected by delay/noise/gain-mismatch
  // regressions. Velocity uncertainty accommodates unmodelled input response;
  // the reliable angle observation must also correct velocity through P_w_phi.
  // These are observer tuning values, not measured sensor-noise estimates.
  static constexpr Scalar velocityProcessVariance = Scalar(3e-3);
  static constexpr Scalar angleProcessVariance = Scalar(3e-6);
  // Disturbance-acceleration random walk; measurement R remains 5e-6 rad^2.
  static constexpr Scalar disturbanceProcessVariance = Scalar(1e-5);
  // Retain a small quartic speed cost after capture too: the local quadratic
  // holding/momentum policy dominates near zero, but high RPM is never free.
  static constexpr Scalar holdingSpeedPenaltyFraction = Scalar(.25);
  // Generic solver API default; the cycle uses ControllerConfig::maxIterations.
  static constexpr int solverMaxIterations = 8, solverMaxBacktracks = 6;
  // Selected on a separate numerical screen, then checked on 22 held-out
  // disturbance/mismatch cases. Avoid spending float iterations below useful
  // control resolution; convergence still requires the VERIFIED gradient.
  static constexpr Scalar gradientTolerance = Scalar(1e-4);
  // Cooperative MCU work budget, including EKF/preview time before solve.
  // One indivisible evaluation may finish after this budget; the application's
  // 100-ms late-command rejection remains the final publication guard.
  static constexpr uint32_t controlWorkBudgetUs = 70000;
  static constexpr Scalar relativeStepTolerance = Scalar(1e-9);
  static constexpr Scalar regularization = Scalar(1e-5), armijoFraction = Scalar(1e-4);
};
} // namespace pendulum
#endif // ADAPTIVE_CONTROLTUNING_H
