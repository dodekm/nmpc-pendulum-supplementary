#ifndef ADAPTIVE_CONTROLLERVALIDATION_H
#define ADAPTIVE_CONTROLLERVALIDATION_H
#pragma once
#include "ControllerConfig.h"
#include "MatrixValidation.h"
#include "ControlTuning.h"

namespace pendulum {
// Duration conversions must be checked before converting floating point to an
// integer; NaN, infinity and a rounded 2^32 would otherwise have undefined behavior.
template <class Scalar> bool durationIsRepresentable(Scalar duration, Scalar period) {
  if (!std::isfinite(duration) || duration < Scalar(0) || !std::isfinite(period) ||
      period <= Scalar(0)) {
    return false;
  }
  const Scalar intervals = std::ceil(duration / period);
  return std::isfinite(intervals) && intervals < Scalar(4294967296ULL);
}
template <class Scalar> uint32_t durationIntervalsOrZero(Scalar duration, Scalar period) {
  return durationIsRepresentable(duration, period) ? uint32_t(std::ceil(duration / period)) : 0;
}

// The configuration is immutable for a controller's lifetime. This validation
// runs once; the frozen MATLAB export in ControllerConfig.h remains untouched.
template <class Scalar>
bool isControllerConfigValid(const ControllerConfig<Scalar> &config, Scalar period) {
  if (config.activeHorizon < 1 || config.activeHorizon > kHorizon) {
    return false;
  }
  const Scalar nonnegative[] = {config.energy_cost_weight,
                                config.terminal_scale,
                                config.wheel_nonlinear_weight,
                                config.momentum_unload_rate,
                                config.momentum_wheel_weight,
                                config.cycle_return_terminal_scale,
                                config.cycle_upright_angular_speed_tolerance,
                                config.cycle_down_angular_speed_tolerance,
                                config.cycle_down_wheel_speed_tolerance,
                                config.cycle_down_acceleration_tolerance};
  for (Scalar value : nonnegative) {
    if (!std::isfinite(value) || value < Scalar(0)) {
      return false;
    }
  }
  const Scalar positive[] = {config.u_limit_swing,
                             config.u_limit_stabilize,
                             config.wheel_cost_scale,
                             config.momentum_weight_ramp_time,
                             config.energy_reference_increment,
                             config.R_swing,
                             config.R_stabilize,
                             config.R_return,
                             config.cycle_return_timeout};
  for (Scalar value : positive) {
    if (!std::isfinite(value) || value <= Scalar(0)) {
      return false;
    }
  }
  const Scalar angles[] = {config.momentum_capture_angle_tolerance,
                           config.cycle_upright_angle_tolerance, config.cycle_down_angle_tolerance};
  for (Scalar value : angles) {
    if (!std::isfinite(value) || value < Scalar(0) || value >= kPi<Scalar>) {
      return false;
    }
  }
  using Tuning = ControlTuning<Scalar>;
  const Scalar innerAngle = Tuning::angleBlendLowDegrees * Scalar(3.14159265358979323846 / 180);
  if (!std::isfinite(Tuning::energyBlendLow) || Tuning::energyBlendLow < Scalar(0) ||
      !std::isfinite(Tuning::angleBlendLowDegrees) || Tuning::angleBlendLowDegrees < Scalar(0) ||
      Tuning::angleBlendLowDegrees >= Scalar(180) || !std::isfinite(config.energy_blend_high) ||
      config.energy_blend_high <= Tuning::energyBlendLow ||
      !std::isfinite(config.angle_blend_high) || config.angle_blend_high <= innerAngle ||
      config.angle_blend_high > kPi<Scalar> || !std::isfinite(config.encoder_reference_offset) ||
      config.maxIterations < 1 || config.maxSamples < 1) {
    return false;
  }
  // Distinct angles can have IDENTICAL cosines after rounding, especially in
  // binary32. Check the actual scheduling divisor, not just the angle ordering.
  if (!(std::cos(innerAngle) > std::cos(config.angle_blend_high))) {
    return false;
  }
  const Scalar degrees[] = {Tuning::disturbanceLeanLimitDegrees, Tuning::lostCaptureAngleDegrees,
                            Tuning::unloadingAngleDegrees};
  for (Scalar value : degrees) {
    if (!std::isfinite(value) || value < Scalar(0) || value >= Scalar(90)) {
      return false;
    }
  }
  if (!std::isfinite(Tuning::captureMembership) || Tuning::captureMembership < Scalar(0) ||
      Tuning::captureMembership > Scalar(1) || !std::isfinite(Tuning::unloadingMembership) ||
      Tuning::unloadingMembership < Scalar(0) || Tuning::unloadingMembership > Scalar(1) ||
      !std::isfinite(Tuning::captureAngularSpeed) || Tuning::captureAngularSpeed < Scalar(0) ||
      !std::isfinite(Tuning::unloadingAngularSpeed) || Tuning::unloadingAngularSpeed < Scalar(0) ||
      !std::isfinite(Tuning::lostCaptureRelativeEnergy) ||
      Tuning::lostCaptureRelativeEnergy < Scalar(0) || Tuning::captureSamples == 0) {
    return false;
  }
  for (Scalar duration : {config.momentum_unload_dwell, config.cycle_upright_dwell,
                          config.cycle_down_dwell, config.cycle_return_timeout}) {
    if (!durationIsRepresentable(duration, period)) {
      return false;
    }
  }
  for (const auto *weight : {&config.swing, &config.hold, &config.down}) {
    if (!(weight->array() == weight->transpose().array()).all() ||
        !isSymmetricPositiveSemidefinite(*weight)) {
      return false;
    }
  }
  Matrix<Scalar, 3, 3> unloadingWeight = config.hold;
  unloadingWeight(2, 2) = config.momentum_wheel_weight;
  if (!isSymmetricPositiveSemidefinite(unloadingWeight)) {
    return false;
  }
  return true;
}
} // namespace pendulum
#endif // ADAPTIVE_CONTROLLERVALIDATION_H
