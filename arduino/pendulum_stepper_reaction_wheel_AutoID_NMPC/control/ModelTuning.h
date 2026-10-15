#ifndef ADAPTIVE_MODEL_TUNING_H
#define ADAPTIVE_MODEL_TUNING_H
#include "Model.h"
#include "ControllerConfig.h"
namespace pendulum {
// Dimensionless coordinates: omega/sqrt(alpha), angle, gamma*wheel/sqrt(alpha),
// gamma*acceleration/alpha, and relative pendulum energy E/(2*alpha).
// These are penalty weights, NOT pre-supplied plant coefficients.
template <class T>
bool configureIdentifiedModel(const ModelParameters<T> &model, ControllerConfig<T> &config) {
  if (!model.isValid()) {
    return false;
  }
  const T a = model.alpha, g2 = model.gamma * model.gamma, momentumScale = g2 / a,
          inputScale = g2 / (a * a);
  // Approximately three natural-time units, with bounded static storage.
  // A long physical horizon around a stiff upright makes sensitivities grow
  // exponentially and can destroy single-precision conditioning.
  // Clamp in floating point BEFORE conversion; extreme valid periods can overflow int.
  const T desiredHorizon = std::ceil(T(3) / (std::sqrt(a) * model.samplePeriod));
  config.activeHorizon = int(std::max(T(6), std::min(T(kHorizon), desiredHorizon)));
  config.energy_cost_weight = T(1024) / (T(4) * a * a);
  config.swing.setZero();
  // Capture-mismatch candidate: stronger position guidance on the last approach.
  // Energy, momentum penalties and capture confirmation gates remain unchanged.
  config.swing(1, 1) = T(.4);
  config.swing(2, 2) = T(12.8) * momentumScale;
  config.hold.setZero();
  config.hold.diagonal() << T(96) / a, T(4), T(128) * momentumScale;
  config.down.setZero();
  config.down.diagonal() << T(16) / a, T(8), T(2560) * momentumScale;
  config.momentum_wheel_weight = T(512) * momentumScale;
  config.R_swing = T(1) * inputScale;
  config.R_stabilize = T(150) * inputScale;
  config.R_return = T(300) * inputScale;
  config.energy_reference_increment = T(2) * a / T(700); // 70 s relative-energy ramp
  // Do not activate a large local terminal cost while the pendulum is still
  // outside the angle at which the selected actuator can counter gravity.
  const T authority = std::min(T(.95), std::abs(model.gamma) * config.u_limit_stabilize / a);
  config.angle_blend_high =
      std::min(T(.5235987755982988), std::max(T(.0534), T(.8) * std::asin(authority)));
  config.energy_blend_high = std::max(T(.025), T(1) - std::cos(config.angle_blend_high));
  config.encoder_reference_offset = T(0); // origin already frozen by application
  return std::isfinite(config.energy_cost_weight) && config.R_swing > T(0) &&
         config.hold.allFinite() && config.down.allFinite() && config.swing.allFinite();
}
} // namespace pendulum
#endif
