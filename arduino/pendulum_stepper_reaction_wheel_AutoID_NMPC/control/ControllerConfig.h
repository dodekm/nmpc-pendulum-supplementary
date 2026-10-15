#ifndef ADAPTIVE_CONTROLLERCONFIG_H
#define ADAPTIVE_CONTROLLERCONFIG_H
#pragma once
#include "NumericTypes.h"
#include "ControlTuning.h"
// Operating policy derives from the established cycle. ModelTuning.h replaces
// dimensional penalties/horizon/capture range after startup identification.

namespace pendulum {
// Both terminal matrices are designed from the validated startup model.
template <class Scalar> struct ControllerConfig {
  int activeHorizon = kHorizon;
  Scalar energy_cost_weight = Scalar(1);
  Scalar terminal_scale = Scalar(0.20000000000000001);
  Scalar u_limit_swing = Scalar(40);
  Scalar u_limit_stabilize = Scalar(40);
  Scalar wheel_nonlinear_weight = Scalar(0.10000000000000001);
  Scalar wheel_cost_scale = Scalar(20);
  Scalar momentum_unload_rate = Scalar(2);
  Scalar momentum_wheel_weight = Scalar(0.040000000000000001);
  Scalar momentum_weight_ramp_time = Scalar(0.29999999999999999);
  Scalar momentum_unload_dwell = Scalar(2);
  Scalar momentum_capture_angle_tolerance = Scalar(0.05235987755982989);
  Scalar energy_reference_increment = Scalar(0.025000000000000001);
  Scalar angle_blend_high = Scalar(0.52359877559829882);
  Scalar energy_blend_high = Scalar(0.025000000000000001);
  Scalar R_swing = Scalar(0.0001);
  Scalar R_stabilize = Scalar(0.0015);
  Scalar R_return = Scalar(0.0030000000000000001);
  Scalar cycle_return_terminal_scale = Scalar(1);
  Scalar cycle_upright_dwell = Scalar(30);
  Scalar cycle_upright_angle_tolerance = Scalar(0.05235987755982989);
  Scalar cycle_upright_angular_speed_tolerance = Scalar(0.5);
  Scalar cycle_down_dwell = Scalar(5);
  Scalar cycle_down_angle_tolerance = Scalar(0.034906585039886591);
  Scalar cycle_down_angular_speed_tolerance = Scalar(0.14999999999999999);
  Scalar cycle_down_wheel_speed_tolerance = Scalar(0.20000000000000001);
  Scalar cycle_down_acceleration_tolerance = Scalar(1);
  Scalar cycle_return_timeout = Scalar(180);
  Scalar encoder_reference_offset = Scalar(0.0061359233222901821);
  int maxIterations = ControlTuning<Scalar>::solverMaxIterations, maxSamples = 4000;
  Matrix<Scalar, 3, 3> swing = Matrix<Scalar, 3, 3>::Zero(), hold = Matrix<Scalar, 3, 3>::Zero(),
                       down = Matrix<Scalar, 3, 3>::Zero();
  ControllerConfig() {
    swing << Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0.10000000000000001), Scalar(0),
        Scalar(0), Scalar(0), Scalar(0.001);
    hold << Scalar(12), Scalar(0), Scalar(0), Scalar(0), Scalar(4), Scalar(0), Scalar(0), Scalar(0),
        Scalar(0.01);
    down << Scalar(2), Scalar(0), Scalar(0), Scalar(0), Scalar(8), Scalar(0), Scalar(0), Scalar(0),
        Scalar(0.20000000000000001);
  }
};
} // namespace pendulum
#endif // ADAPTIVE_CONTROLLERCONFIG_H
