#ifndef ADAPTIVE_MODEL_H
#define ADAPTIVE_MODEL_H
#pragma once
#include "NumericTypes.h"

namespace pendulum {
template <class Scalar> struct ModelParameters {
  // x=[pendulum angular velocity, angle from downward, prescribed wheel speed].
  // xdot=[-alpha*sin(angle)-beta*omega-c*tanh(omega/ws)-gamma*u+d, omega, u].
  // EKF adds constant disturbance acceleration d as its fourth state.
  // Invalid until a validated startup estimate is explicitly supplied.
  Scalar alpha = std::numeric_limits<Scalar>::quiet_NaN(),
         beta = std::numeric_limits<Scalar>::quiet_NaN(),
         gamma = std::numeric_limits<Scalar>::quiet_NaN(), samplePeriod = Scalar(.1);
  Scalar coulomb = Scalar(0);
  static constexpr Scalar frictionVelocityScale = Scalar(.04);
  struct DampingEvaluation {
    Scalar frictionAcceleration, velocityDerivative;
  };
  DampingEvaluation evaluateDamping(Scalar angularVelocity) const {
    if (coulomb == Scalar(0)) {
      return {Scalar(0), beta};
    }
    const Scalar t = std::tanh(angularVelocity / frictionVelocityScale);
    return {coulomb * t, beta + coulomb / frictionVelocityScale * (Scalar(1) - t * t)};
  }
  Scalar dampingSlope(Scalar angularVelocity) const {
    return evaluateDamping(angularVelocity).velocityDerivative;
  }
  Scalar angularAccelerationFromSine(Scalar angularVelocity, Scalar sineAngle,
                                     Scalar wheelAndDisturbanceAcceleration,
                                     DampingEvaluation *cached = nullptr) const {
    const auto damping = evaluateDamping(angularVelocity);
    if (cached) {
      *cached = damping;
    }
    return -alpha * sineAngle - beta * angularVelocity + wheelAndDisturbanceAcceleration -
           damping.frictionAcceleration;
  }
  bool isValid() const {
    return std::isfinite(alpha) && alpha > Scalar(0) && std::isfinite(beta) && beta >= Scalar(0) &&
           std::isfinite(gamma) && gamma != Scalar(0) && std::isfinite(coulomb) &&
           coulomb >= Scalar(0) && std::isfinite(samplePeriod) && samplePeriod > Scalar(0);
  }
};
} // namespace pendulum
#endif // ADAPTIVE_MODEL_H
