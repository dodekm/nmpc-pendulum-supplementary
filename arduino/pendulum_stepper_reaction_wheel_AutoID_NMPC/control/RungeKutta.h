#ifndef ADAPTIVE_RUNGEKUTTA_H
#define ADAPTIVE_RUNGEKUTTA_H
#pragma once
#include "Model.h"

namespace pendulum {
// Only the FIRST angle is decision-independent after a speed jump. The
// second RK angle depends on the jumped velocity and must be recomputed.
template <class Scalar> struct InitialStageTrigonometry {
  const Scalar initialAngle, firstSine, firstCosine;
  InitialStageTrigonometry(const Matrix<Scalar, 3> &state, const ModelParameters<Scalar> &)
      : initialAngle(state(1)), firstSine(std::sin(initialAngle)),
        firstCosine(std::cos(initialAngle)) {}
  bool matches(const Matrix<Scalar, 3> &state, const ModelParameters<Scalar> &) const {
    return initialAngle == state(1);
  }
};
// Differentiate the actual four RK stages, not an endpoint-frozen ODE.
template <class Scalar>
bool integrateContinuousRK4(const Matrix<Scalar, 3> &state, Scalar decisionInput,
                            const ModelParameters<Scalar> &model, Scalar disturbanceAcceleration,
                            Scalar accelerationScale, Matrix<Scalar, 3> &nextState,
                            Matrix<Scalar, 3, 3> *stateJacobian = nullptr,
                            Matrix<Scalar, 3> *inputJacobian = nullptr,
                            const InitialStageTrigonometry<Scalar> *trigonometry = nullptr) {
  if (!model.isValid() || !state.allFinite() || !std::isfinite(decisionInput) ||
      !std::isfinite(disturbanceAcceleration) || !std::isfinite(accelerationScale) ||
      accelerationScale < Scalar(0)) {
    return false;
  }
  Scalar inputDerivative = Scalar(1), physicalInput = decisionInput;
  if (accelerationScale > Scalar(0)) {
    Scalar t = std::tanh(decisionInput / accelerationScale);
    physicalInput = accelerationScale * t;
    inputDerivative = Scalar(1) - t * t;
  }
  const Scalar halfStep = model.samplePeriod / Scalar(2),
               sixthStep = model.samplePeriod / Scalar(6),
               constantAcceleration = -model.gamma * physicalInput + disturbanceAcceleration;
  const bool cached = trigonometry && trigonometry->matches(state, model);
  // Reuse each stage's tanh evaluation for force AND variational dynamics.
  typename ModelParameters<Scalar>::DampingEvaluation damping1, damping2, damping3, damping4;
  const Scalar omega1 = state(0), angle1 = state(1),
               angularAcceleration1 = model.angularAccelerationFromSine(
                   omega1, cached ? trigonometry->firstSine : std::sin(angle1),
                   constantAcceleration, &damping1);
  const Scalar omega2 = omega1 + halfStep * angularAcceleration1,
               angle2 = angle1 + halfStep * omega1,
               angularAcceleration2 = model.angularAccelerationFromSine(
                   omega2, std::sin(angle2), constantAcceleration, &damping2);
  const Scalar omega3 = omega1 + halfStep * angularAcceleration2,
               angle3 = angle1 + halfStep * omega2,
               angularAcceleration3 = model.angularAccelerationFromSine(
                   omega3, std::sin(angle3), constantAcceleration, &damping3);
  const Scalar omega4 = omega1 + model.samplePeriod * angularAcceleration3,
               angle4 = angle1 + model.samplePeriod * omega3,
               angularAcceleration4 = model.angularAccelerationFromSine(
                   omega4, std::sin(angle4), constantAcceleration, &damping4);
  nextState << omega1 + sixthStep * (angularAcceleration1 + Scalar(2) * angularAcceleration2 +
                                     Scalar(2) * angularAcceleration3 + angularAcceleration4),
      angle1 + sixthStep * (omega1 + Scalar(2) * omega2 + Scalar(2) * omega3 + omega4),
      state(2) + sixthStep * (physicalInput + Scalar(2) * physicalInput +
                              Scalar(2) * physicalInput + physicalInput);
  if (!nextState.allFinite()) {
    return false;
  }
  if (stateJacobian || inputJacobian) {
    const Matrix<Scalar, 2, 2> identity = Matrix<Scalar, 2, 2>::Identity();
    Matrix<Scalar, 2, 2> dynamicsJacobian1, dynamicsJacobian2, dynamicsJacobian3, dynamicsJacobian4;
    dynamicsJacobian1 << -damping1.velocityDerivative,
        -model.alpha * (cached ? trigonometry->firstCosine : std::cos(angle1)), 1, 0;
    dynamicsJacobian2 << -damping2.velocityDerivative, -model.alpha * std::cos(angle2), 1, 0;
    dynamicsJacobian3 << -damping3.velocityDerivative, -model.alpha * std::cos(angle3), 1, 0;
    dynamicsJacobian4 << -damping4.velocityDerivative, -model.alpha * std::cos(angle4), 1, 0;
    if (stateJacobian) {
      Matrix<Scalar, 2, 2> sensitivity1 = dynamicsJacobian1,
                           sensitivity2 = dynamicsJacobian2 * (identity + halfStep * sensitivity1),
                           sensitivity3 = dynamicsJacobian3 * (identity + halfStep * sensitivity2),
                           sensitivity4 =
                               dynamicsJacobian4 * (identity + model.samplePeriod * sensitivity3);
      stateJacobian->setIdentity();
      stateJacobian->template topLeftCorner<2, 2>() =
          identity + sixthStep * (sensitivity1 + Scalar(2) * sensitivity2 +
                                  Scalar(2) * sensitivity3 + sensitivity4);
      if (!stateJacobian->allFinite()) {
        return false;
      }
    }
    if (inputJacobian) {
      Matrix<Scalar, 2> inputEffect;
      inputEffect << -model.gamma * inputDerivative, 0;
      Matrix<Scalar, 2> inputSensitivity1 = inputEffect,
                        inputSensitivity2 =
                            dynamicsJacobian2 * (halfStep * inputSensitivity1) + inputEffect,
                        inputSensitivity3 =
                            dynamicsJacobian3 * (halfStep * inputSensitivity2) + inputEffect,
                        inputSensitivity4 =
                            dynamicsJacobian4 * (model.samplePeriod * inputSensitivity3) +
                            inputEffect;
      inputJacobian->template head<2>() =
          sixthStep * (inputSensitivity1 + Scalar(2) * inputSensitivity2 +
                       Scalar(2) * inputSensitivity3 + inputSensitivity4);
      (*inputJacobian)(2) = model.samplePeriod * inputDerivative;
      if (!inputJacobian->allFinite()) {
        return false;
      }
    }
  }
  return true;
}
// Free flow over an arbitrary nonnegative duration, without changing the
// nominal controller sampling period. Used on both sides of an acknowledged
// wheel-speed jump by the chronological EKF.
template <class Scalar>
bool integrateFreeFlowRK4(const Matrix<Scalar, 3> &state,
                          const ModelParameters<Scalar> &model, Scalar disturbanceAcceleration,
                          Scalar duration, Matrix<Scalar, 3> &nextState,
                          Matrix<Scalar, 3, 3> *stateJacobian = nullptr,
                          const InitialStageTrigonometry<Scalar> *trigonometry = nullptr) {
  if (!state.allFinite() || !model.isValid() || !std::isfinite(disturbanceAcceleration) ||
      !std::isfinite(duration) || duration < Scalar(0)) return false;
  if (duration == Scalar(0)) {
    nextState = state;
    if (stateJacobian) stateJacobian->setIdentity();
    return true;
  }
  const Scalar frequency = std::max(std::sqrt(model.alpha), model.dampingSlope(Scalar(0)));
  const Scalar requestedPieces =
      std::max(frequency * duration / Scalar(.2),
               model.coulomb > Scalar(0) ? duration / Scalar(.02) : Scalar(1));
  if (!std::isfinite(requestedPieces) || !(requestedPieces > Scalar(0)) ||
      requestedPieces > Scalar(64)) return false;
  const int pieces = int(std::ceil(requestedPieces));
  if (pieces < 1 || pieces > 64) return false;
  ModelParameters<Scalar> submodel = model;
  submodel.samplePeriod = duration / Scalar(pieces);
  Matrix<Scalar, 3, 3> f = Matrix<Scalar, 3, 3>::Identity(), stepF;
  Matrix<Scalar, 3> current = state;
  for (int piece = 0; piece < pieces; ++piece) {
    if (!integrateContinuousRK4<Scalar>(current, Scalar(0), submodel, disturbanceAcceleration,
                                        Scalar(0), nextState, stateJacobian ? &stepF : nullptr,
                                        nullptr, piece == 0 ? trigonometry : nullptr)) return false;
    if (stateJacobian) f = (stepF * f).eval();
    current = nextState;
  }
  if (stateJacobian) *stateJacobian = f;
  return true;
}
// Exact prescribed held-speed realization: the command changes wheel speed
// by h*psi(a) at the interval START, then the pendulum follows free flow.
// This matches startup identification instead of mixing impulse identification
// with an averaged-acceleration controller model.
template <class Scalar>
bool integrateRK4(const Matrix<Scalar, 3> &state, Scalar decisionInput,
                  const ModelParameters<Scalar> &model, Scalar disturbanceAcceleration,
                  Scalar accelerationScale, Matrix<Scalar, 3> &nextState,
                  Matrix<Scalar, 3, 3> *stateJacobian = nullptr,
                  Matrix<Scalar, 3> *inputJacobian = nullptr,
                  const InitialStageTrigonometry<Scalar> *trigonometry = nullptr) {
  if (!state.allFinite() || !model.isValid() || !std::isfinite(decisionInput) ||
      !std::isfinite(accelerationScale) || accelerationScale < Scalar(0)) {
    return false;
  }
  Scalar u = decisionInput, derivative = Scalar(1);
  if (accelerationScale > Scalar(0)) {
    const Scalar t = std::tanh(decisionInput / accelerationScale);
    u = accelerationScale * t;
    derivative = Scalar(1) - t * t;
  }
  Matrix<Scalar, 3> jumped = state;
  jumped(0) -= model.gamma * u * model.samplePeriod;
  jumped(2) += u * model.samplePeriod;
  Matrix<Scalar, 3, 3> f;
  if (!integrateFreeFlowRK4(jumped, model, disturbanceAcceleration, model.samplePeriod,
                            nextState, (stateJacobian || inputJacobian) ? &f : nullptr,
                            trigonometry)) return false;
  if (stateJacobian) {
    *stateJacobian = f;
  }
  if (inputJacobian) {
    Matrix<Scalar, 3> b;
    b << -model.gamma, Scalar(0), Scalar(1);
    *inputJacobian = (f * b) * (model.samplePeriod * derivative);
    if (!inputJacobian->allFinite()) {
      return false;
    }
  }
  return true;
}
} // namespace pendulum
#endif // ADAPTIVE_RUNGEKUTTA_H
