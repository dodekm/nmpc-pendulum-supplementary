#ifndef ADAPTIVE_AUGMENTEDEKF_H
#define ADAPTIVE_AUGMENTEDEKF_H
#pragma once
#include "RungeKutta.h"
#include "MatrixExponential.h"
#include "MatrixValidation.h"
#include "ControlTuning.h"
#include "MeasurementTiming.h"

namespace pendulum {
template <class Scalar> bool isCovarianceValid(const Matrix<Scalar, 4, 4> &predictedCovariance) {
  return isSymmetricPositiveSemidefinite(predictedCovariance);
}
template <class Scalar> struct InnovationDiagnostics {
  Matrix<Scalar, 4> prior = Matrix<Scalar, 4>::Zero(), gain = Matrix<Scalar, 4>::Zero();
  Scalar innovation = Scalar(0), variance = Scalar(0), nis = Scalar(0);
};
// State order: [pendulum angular velocity, downward-referenced angle,
// prescribed wheel angular velocity, disturbance angular acceleration].
// The owner establishes initial state/covariance; step() commits both, and
// its diagnostics, only after the complete chronological update succeeds.
template <class Scalar> class AugmentedEKF {
public:
  Matrix<Scalar, 4> state = Matrix<Scalar, 4>::Zero();
  Matrix<Scalar, 4, 4> covariance = Matrix<Scalar, 4, 4>::Zero();
  AugmentedEKF() { covariance(3, 3) = Scalar(.01); }
  bool step(Scalar acceleration, Scalar measurement, const ModelParameters<Scalar> &model,
            int elapsedIntervals, InnovationDiagnostics<Scalar> &diagnostics,
            const MeasurementTiming<Scalar> *timing = nullptr) {
    if (!state.allFinite() || !std::isfinite(acceleration) || !std::isfinite(measurement) ||
        !model.isValid() || elapsedIntervals < 1 || elapsedIntervals > 100 ||
        !isCovarianceValid(covariance) ||
        (timing && (elapsedIntervals != 1 || !timing->isValid(model.samplePeriod)))) {
      return false;
    }
    // Commit state/covariance only if the whole prediction + correction succeeds.
    Matrix<Scalar, 4> predictedState = state;
    Matrix<Scalar, 4, 4> predictedCovariance = covariance,
                         processCovariance = Matrix<Scalar, 4, 4>::Zero();
    processCovariance.diagonal() << ControlTuning<Scalar>::velocityProcessVariance,
        ControlTuning<Scalar>::angleProcessVariance, Scalar(0),
        ControlTuning<Scalar>::disturbanceProcessVariance;
    for (int intervalIndex = 0; intervalIndex < elapsedIntervals; ++intervalIndex) {
      const Scalar interval = timing ? timing->interval : model.samplePeriod;
      const bool commandOccurred = timing ? timing->commandOccurred : intervalIndex == 0;
      const Scalar delay =
          timing && commandOccurred && acceleration != Scalar(0) ? timing->commandDelay : Scalar(0);
      // Known-input jump has identity state Jacobian. Its magnitude is the
      // acknowledged speed increment (acceleration * NOMINAL samplePeriod).
      // Free flow first accounts for the previous command's publication delay.
      if (!propagateFreeFlow(predictedState, predictedCovariance, model, delay))
        return false;
      if (commandOccurred) {
        predictedState(0) -= model.gamma * acceleration * model.samplePeriod;
        predictedState(2) += acceleration * model.samplePeriod;
      }
      if (!propagateFreeFlow(predictedState, predictedCovariance, model, interval - delay))
        return false;
      // Q is per observed nominal interval, NOT per split flow segment. Splitting
      // chronology must not double the configured process variance.
      predictedCovariance += processCovariance;
      if (!isCovarianceValid(predictedCovariance)) {
        return false;
      }
    }
    InnovationDiagnostics<Scalar> pending;
    pending.prior = predictedState;
    pending.innovation = measurement - predictedState(1);
    pending.variance = predictedCovariance(1, 1) + Scalar(5e-6);
    if (!std::isfinite(pending.variance) || !(pending.variance > Scalar(0))) {
      return false;
    }
    pending.gain = predictedCovariance.col(1) / pending.variance;
    predictedState += pending.gain * pending.innovation;
    Matrix<Scalar, 4, 4> correction = Matrix<Scalar, 4, 4>::Identity();
    correction.col(1) -= pending.gain;
    predictedCovariance = (correction * predictedCovariance * correction.transpose() +
                           Scalar(5e-6) * pending.gain * pending.gain.transpose())
                              .eval();
    predictedCovariance =
        (Scalar(.5) * predictedCovariance + Scalar(.5) * predictedCovariance.transpose()).eval();
    pending.nis = pending.innovation * pending.innovation / pending.variance;
    if (!predictedState.allFinite() || !std::isfinite(pending.nis) ||
        !isCovarianceValid(predictedCovariance)) {
      return false;
    }
    state = predictedState;
    covariance = predictedCovariance;
    diagnostics = pending;
    return true;
  }

private:
  static bool propagateFreeFlow(Matrix<Scalar, 4> &predictedState,
                                Matrix<Scalar, 4, 4> &predictedCovariance,
                                const ModelParameters<Scalar> &model, Scalar duration) {
    if (duration == Scalar(0))
      return predictedState.allFinite() && predictedCovariance.allFinite();
    Matrix<Scalar, 4, 4> continuousJacobian = Matrix<Scalar, 4, 4>::Zero(), transition;
    continuousJacobian(0, 0) = -model.dampingSlope(predictedState(0));
    continuousJacobian(0, 1) = -model.alpha * std::cos(predictedState(1));
    continuousJacobian(0, 3) = continuousJacobian(1, 0) = Scalar(1);
    if (!matrixExponential(Matrix<Scalar, 4, 4>(continuousJacobian * duration), transition))
      return false;
    Matrix<Scalar, 3> next, physical = predictedState.template head<3>();
    if (!integrateFreeFlowRK4(physical, model, predictedState(3), duration, next))
      return false;
    predictedState.template head<3>() = next;
    predictedCovariance = (transition * predictedCovariance * transition.transpose()).eval();
    return predictedState.allFinite() && predictedCovariance.allFinite();
  }
};
} // namespace pendulum
#endif // ADAPTIVE_AUGMENTEDEKF_H
