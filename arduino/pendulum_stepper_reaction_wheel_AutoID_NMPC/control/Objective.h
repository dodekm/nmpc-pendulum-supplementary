#ifndef ADAPTIVE_OBJECTIVE_H
#define ADAPTIVE_OBJECTIVE_H
#pragma once
#include "MPCProblem.h"
#include "RungeKutta.h"
#include "numerics/CompensatedSum.h"

namespace pendulum {
template <class Scalar> struct ObjectiveDerivatives {
  Scalar cost = std::numeric_limits<Scalar>::infinity();
  InputPlan<Scalar> gradient = InputPlan<Scalar>::Zero();
  Matrix<Scalar, kHorizon, kHorizon> hessian = Matrix<Scalar, kHorizon, kHorizon>::Zero();
  Matrix<Scalar, 3> firstPrediction = Matrix<Scalar, 3>::Zero(),
                    lastPrediction = Matrix<Scalar, 3>::Zero();
};
template <class Scalar> struct PredictionTrace {
  Matrix<Scalar, 3 * kHorizon> states = Matrix<Scalar, 3 * kHorizon>::Zero();
  Matrix<Scalar, 3 * kHorizon, kHorizon> jacobian = Matrix<Scalar, 3 * kHorizon, kHorizon>::Zero();
};
template <class Scalar>
struct ObjectiveEvaluation : ObjectiveDerivatives<Scalar>, PredictionTrace<Scalar> {};

namespace detail {
// Same arithmetic for all entry points. Compile-time modes eliminate unused
// derivative/trace storage and initialization from Armijo cost trials.
template <bool Derivatives, bool RecordTrace, class Scalar>
bool evaluateTrajectory(const MPCProblem<Scalar> &problem, const InputPlan<Scalar> &inputPlan,
                        Scalar &cost,
                        [[maybe_unused]] ObjectiveDerivatives<Scalar> *derivativeOutput,
                        [[maybe_unused]] PredictionTrace<Scalar> *trace,
                        const InitialStageTrigonometry<Scalar> *trigonometry = nullptr) {
  cost = std::numeric_limits<Scalar>::infinity();
  if (!inputPlan.allFinite()) {
    return false;
  }
  cost = Scalar(0);
  numerics::CompensatedSum<Scalar> costSum;
  if constexpr (Derivatives) {
    derivativeOutput->gradient.setZero();
    derivativeOutput->hessian.setZero();
  }
  if constexpr (RecordTrace) {
    trace->states.setZero();
    trace->jacobian.setZero();
  }
  Matrix<Scalar, 3> state = problem.initialState;
  Matrix<Scalar, 3, kHorizon> sensitivity;
  if constexpr (Derivatives) {
    sensitivity.setZero();
  }
  const Scalar scaleSquared = problem.wheelSpeedScale * problem.wheelSpeedScale;
  if (!std::isfinite(scaleSquared) || !(scaleSquared > Scalar(0))) {
    return false;
  }
  for (int stageIndex = 0; stageIndex < problem.activeHorizon; ++stageIndex) {
    Matrix<Scalar, 3> nextState, inputJacobian;
    Matrix<Scalar, 3, 3> stateJacobian;
    if (!integrateRK4(state, inputPlan(stageIndex), problem.model, problem.disturbanceAcceleration,
                      problem.accelerationScale, nextState, Derivatives ? &stateJacobian : nullptr,
                      Derivatives ? &inputJacobian : nullptr,
                      stageIndex == 0 ? trigonometry : nullptr)) {
      return false;
    }
    state = nextState;
    if constexpr (Derivatives) {
      if (stageIndex == 0) {
        derivativeOutput->firstPrediction = state;
      }
      if (stageIndex == problem.activeHorizon - 1) {
        derivativeOutput->lastPrediction = state;
      }
    }
    if constexpr (RecordTrace) {
      trace->states.template segment<3>(3 * stageIndex) = state;
    }
    if constexpr (Derivatives) {
      for (int column = 0; column < stageIndex; ++column) {
        // Exact structure: wheel row [0,0,1]; pendulum/wheel cross block zero.
        const Scalar omega = sensitivity(0, column), angle = sensitivity(1, column);
        sensitivity(0, column) = stateJacobian(0, 0) * omega + stateJacobian(0, 1) * angle;
        sensitivity(1, column) = stateJacobian(1, 0) * omega + stateJacobian(1, 1) * angle;
      }
      sensitivity.col(stageIndex) = inputJacobian;
      if (!sensitivity.leftCols(stageIndex + 1).allFinite()) {
        return false;
      }
      if constexpr (RecordTrace) {
        trace->jacobian.template block<3, kHorizon>(3 * stageIndex, 0) = sensitivity;
      }
    }
    const Matrix<Scalar, 3> error = state - problem.referenceState[stageIndex];
    const Matrix<Scalar, 3> weightedError = problem.stateWeight[stageIndex] * error;
    const Scalar stageCost =
        Scalar(.5) *
        (error.dot(weightedError) +
         inputPlan(stageIndex) * (problem.inputWeight(stageIndex) * inputPlan(stageIndex)));
    if (!std::isfinite(stageCost)) {
      return false;
    }
    costSum.add(stageCost);
    if constexpr (Derivatives) {
      auto &gradient = derivativeOutput->gradient;
      auto &hessian = derivativeOutput->hessian;
      gradient(stageIndex) += problem.inputWeight(stageIndex) * inputPlan(stageIndex);
      hessian(stageIndex, stageIndex) += problem.inputWeight(stageIndex);
      Matrix<Scalar, 3, kHorizon> weightedSensitivity;
      for (int column = 0; column <= stageIndex; ++column) {
        weightedSensitivity.col(column) = problem.stateWeight[stageIndex] * sensitivity.col(column);
      }
      for (int row = 0; row <= stageIndex; ++row) {
        gradient(row) += sensitivity.col(row).dot(weightedError);
        // Q is validated symmetric. S' Q S is symmetric too: accumulate
        // one triangle, then mirror once after all horizon stages. This
        // removes redundant products without discarding any residual term.
        for (int column = 0; column <= row; ++column) {
          hessian(row, column) += sensitivity.col(row).dot(weightedSensitivity.col(column));
        }
      }
    }
    if (problem.energyWeight(stageIndex) > Scalar(0) ||
        problem.wheelSpeedWeight(stageIndex) > Scalar(0)) {
      Scalar energyResidual = Scalar(0), wheelResidual = Scalar(0);
      if (problem.energyWeight(stageIndex) > Scalar(0)) {
        const Scalar energy = Scalar(.5) * state(0) * state(0) +
                              problem.model.alpha * (Scalar(1) - std::cos(state(1)));
        energyResidual = energy - problem.energyReference(stageIndex);
      }
      if (problem.wheelSpeedWeight(stageIndex) > Scalar(0)) {
        wheelResidual = state(2) * state(2) / scaleSquared;
      }
      if (!std::isfinite(energyResidual) || !std::isfinite(wheelResidual)) {
        return false;
      }
      costSum.add(Scalar(.5) * (problem.energyWeight(stageIndex) * energyResidual * energyResidual +
                               problem.wheelSpeedWeight(stageIndex) * wheelResidual * wheelResidual));
      if constexpr (Derivatives) {
        const Scalar angleGradient = problem.energyWeight(stageIndex) > Scalar(0)
                                         ? problem.model.alpha * std::sin(state(1))
                                         : Scalar(0);
        InputPlan<Scalar> energyJacobian, wheelJacobian;
        for (int column = 0; column <= stageIndex; ++column) {
          energyJacobian(column) =
              problem.energyWeight(stageIndex) > Scalar(0)
                  ? state(0) * sensitivity(0, column) + angleGradient * sensitivity(1, column)
                  : Scalar(0);
          wheelJacobian(column) = problem.wheelSpeedWeight(stageIndex) > Scalar(0)
                                      ? Scalar(2) * state(2) / scaleSquared * sensitivity(2, column)
                                      : Scalar(0);
        }
        for (int row = 0; row <= stageIndex; ++row) {
          derivativeOutput->gradient(row) +=
              problem.energyWeight(stageIndex) * energyResidual * energyJacobian(row) +
              problem.wheelSpeedWeight(stageIndex) * wheelResidual * wheelJacobian(row);
          for (int column = 0; column <= row; ++column) {
            derivativeOutput->hessian(row, column) +=
                problem.energyWeight(stageIndex) * energyJacobian(row) * energyJacobian(column) +
                problem.wheelSpeedWeight(stageIndex) * wheelJacobian(row) * wheelJacobian(column);
          }
        }
      }
    }
    cost = costSum.value();
    if (!std::isfinite(cost)) {
      return false;
    }
  }
  // Inactive storage slots are independent positive input penalties. They
  // neither predict future states nor affect the active optimization problem.
  for (int j = problem.activeHorizon; j < kHorizon; ++j) {
    costSum.add(Scalar(.5) * problem.inputWeight(j) * inputPlan(j) * inputPlan(j));
    if constexpr (Derivatives) {
      derivativeOutput->gradient(j) = problem.inputWeight(j) * inputPlan(j);
      derivativeOutput->hessian(j, j) = problem.inputWeight(j);
    }
  }
  // Inactive-slot penalties participate in the same objective. Their final
  // accumulation can overflow even when every derivative remains finite;
  // success must certify the COST too, for both Armijo and verified plans.
  cost = costSum.value();
  if (!std::isfinite(cost)) {
    return false;
  }
  if constexpr (Derivatives) {
    for (int row = 1; row < problem.activeHorizon; ++row) {
      for (int column = 0; column < row; ++column) {
        derivativeOutput->hessian(column, row) = derivativeOutput->hessian(row, column);
      }
    }
    return derivativeOutput->gradient.allFinite() && derivativeOutput->hessian.allFinite();
  }
  return true;
}
// Internal fast path: solveGaussNewton validates the immutable problem ONCE.
// Do not repeat eigensolves for every derivative evaluation or Armijo trial.
template <class Scalar>
bool evaluateValidatedCost(const MPCProblem<Scalar> &problem, const InputPlan<Scalar> &inputPlan,
                           Scalar &cost, const InitialStageTrigonometry<Scalar> *trigonometry) {
  return evaluateTrajectory<false, false, Scalar>(problem, inputPlan, cost, nullptr, nullptr,
                                                  trigonometry);
}
template <class Scalar>
bool evaluateValidatedDerivatives(const MPCProblem<Scalar> &problem,
                                  const InputPlan<Scalar> &inputPlan,
                                  ObjectiveDerivatives<Scalar> &output,
                                  const InitialStageTrigonometry<Scalar> *trigonometry) {
  return evaluateTrajectory<true, false, Scalar>(problem, inputPlan, output.cost, &output, nullptr,
                                                 trigonometry);
}
} // namespace detail

template <class Scalar>
bool evaluateCost(const MPCProblem<Scalar> &problem, const InputPlan<Scalar> &inputPlan,
                  Scalar &cost, const InitialStageTrigonometry<Scalar> *trigonometry = nullptr) {
  cost = std::numeric_limits<Scalar>::infinity();
  return problem.isValid() && detail::evaluateValidatedCost(problem, inputPlan, cost, trigonometry);
}
template <class Scalar>
bool evaluateDerivatives(const MPCProblem<Scalar> &problem, const InputPlan<Scalar> &inputPlan,
                         ObjectiveDerivatives<Scalar> &output,
                         const InitialStageTrigonometry<Scalar> *trigonometry = nullptr) {
  output.cost = std::numeric_limits<Scalar>::infinity();
  return problem.isValid() &&
         detail::evaluateValidatedDerivatives(problem, inputPlan, output, trigonometry);
}
// Opt-in traces for validation/debugging, never allocated by the optimizer.
template <class Scalar>
bool evaluateObjective(const MPCProblem<Scalar> &problem, const InputPlan<Scalar> &inputPlan,
                       ObjectiveEvaluation<Scalar> &output, bool derivatives = true) {
  output.cost = std::numeric_limits<Scalar>::infinity();
  if (!problem.isValid()) {
    return false;
  }
  if (derivatives) {
    return detail::evaluateTrajectory<true, true, Scalar>(problem, inputPlan, output.cost, &output,
                                                          &output);
  }
  output.gradient.setZero();
  output.hessian.setZero();
  const bool valid = detail::evaluateTrajectory<false, true, Scalar>(problem, inputPlan,
                                                                     output.cost, nullptr, &output);
  if (valid) {
    output.firstPrediction = output.states.template head<3>();
    output.lastPrediction = output.states.template segment<3>(3 * (problem.activeHorizon - 1));
  }
  return valid;
}
} // namespace pendulum
#endif // ADAPTIVE_OBJECTIVE_H
