#ifndef ADAPTIVE_GAUSSNEWTON_H
#define ADAPTIVE_GAUSSNEWTON_H
#pragma once
#include "Objective.h"
#include "ControlTuning.h"
#include "SolverWorkBudget.h"
#include "DescentDirection.h"

namespace pendulum {
enum class SolverExit : uint8_t {
  IterationLimit,
  GradientTolerance,
  LineSearchFailed,
  StepStagnation,
  DirectionFailed,
  LastVerifiedPlan,
  NumericalFailure,
  InvalidProblem,
  WorkBudgetExceeded // appended wire value 8; existing exit codes stay unchanged
};
// A usable plan has been numerically verified in the current problem. This is
// distinct from convergence and from successful physical capture of the rig.
template <class Scalar> struct SolverResult {
  bool usable = false, converged = false, directionFallback = false, warmReset = false,
       verifiedFallback = false;
  int iterations = 0, nonfiniteTrials = 0;
  int derivativeEvaluations = 0, costEvaluations = 0;
  SolverExit exit = SolverExit::IterationLimit;
  Scalar cost = std::numeric_limits<Scalar>::infinity(),
         gradientNorm = std::numeric_limits<Scalar>::infinity();
  Matrix<Scalar, 3> firstPrediction =
                        Matrix<Scalar, 3>::Constant(std::numeric_limits<Scalar>::quiet_NaN()),
                    lastPrediction = firstPrediction;
};

template <class Scalar> struct SolverSettings {
  int maxIterations = ControlTuning<Scalar>::solverMaxIterations,
      maxBacktracks = ControlTuning<Scalar>::solverMaxBacktracks;
  Scalar gradientTolerance = ControlTuning<Scalar>::gradientTolerance;
  // Preserve accepted trajectories. Precision-aware alternatives are measured
  // in the regression suite, not silently selected on hardware.
  Scalar relativeStepTolerance = ControlTuning<Scalar>::relativeStepTolerance;
  Scalar regularization = ControlTuning<Scalar>::regularization,
         armijoFraction = ControlTuning<Scalar>::armijoFraction;
  bool isValid() const {
    return maxIterations >= 1 && maxBacktracks >= 1 && std::isfinite(gradientTolerance) &&
           gradientTolerance >= Scalar(0) && std::isfinite(relativeStepTolerance) &&
           relativeStepTolerance >= Scalar(0) && std::isfinite(regularization) &&
           regularization >= Scalar(0) && std::isfinite(armijoFraction) &&
           armijoFraction > Scalar(0) && armijoFraction < Scalar(1);
  }
};

// Mutates inputPlan to an accepted plan or a verified fallback. All residuals,
// derivatives and line-search trials share the same immutable MPCProblem.
template <class Scalar>
SolverResult<Scalar>
solveGaussNewton(const MPCProblem<Scalar> &problem, InputPlan<Scalar> &inputPlan,
                 const SolverSettings<Scalar> &settings = SolverSettings<Scalar>(),
                 const SolverWorkBudget &budget = SolverWorkBudget()) {
  SolverResult<Scalar> result;
  if (!problem.isValid() || !settings.isValid()) {
    result.exit = SolverExit::InvalidProblem;
    return result;
  }
  if (!inputPlan.allFinite()) {
    inputPlan.setZero();
    result.warmReset = true;
  }
  const InitialStageTrigonometry<Scalar> trigonometry(problem.initialState, problem.model);
  ObjectiveDerivatives<Scalar> evaluation;
  Scalar trialCost = std::numeric_limits<Scalar>::infinity();
  InputPlan<Scalar> verifiedPlan = inputPlan;
  Scalar verifiedCost = Scalar(0), verifiedGradientNorm = Scalar(0);
  Matrix<Scalar, 3> verifiedFirst = Matrix<Scalar, 3>::Zero(), verifiedLast = verifiedFirst;
  bool hasVerifiedPlan = false, currentPlanVerified = false;
  for (int iteration = 0; iteration < settings.maxIterations; ++iteration) {
    if (budget.exhausted()) {
      result.exit = SolverExit::WorkBudgetExceeded;
      break;
    }
    ++result.derivativeEvaluations;
    if (!detail::evaluateValidatedDerivatives(problem, inputPlan, evaluation, &trigonometry)) {
      if (iteration == 0 && (inputPlan.array() != Scalar(0)).any()) {
        inputPlan.setZero();
        result.warmReset = true;
        if (budget.exhausted()) {
          result.exit = SolverExit::WorkBudgetExceeded;
          break;
        }
        ++result.derivativeEvaluations;
        if (!detail::evaluateValidatedDerivatives(problem, inputPlan, evaluation, &trigonometry)) {
          result.exit = SolverExit::NumericalFailure;
          return result;
        }
      } else if (hasVerifiedPlan) {
        inputPlan = verifiedPlan;
        result.verifiedFallback = true;
        result.exit = SolverExit::LastVerifiedPlan;
        currentPlanVerified = true;
        break;
      } else {
        result.exit = SolverExit::NumericalFailure;
        return result;
      }
    }
    currentPlanVerified = true;
    hasVerifiedPlan = true;
    verifiedPlan = inputPlan;
    verifiedCost = evaluation.cost;
    verifiedGradientNorm = planInfinityNorm(evaluation.gradient);
    verifiedFirst = evaluation.firstPrediction;
    verifiedLast = evaluation.lastPrediction;
    if (verifiedGradientNorm <= settings.gradientTolerance) {
      break;
    }
    if (budget.exhausted()) {
      result.exit = SolverExit::WorkBudgetExceeded;
      break;
    }
    InputPlan<Scalar> direction;
    if (!computeDescentDirection(evaluation.hessian, evaluation.gradient, direction,
                                 result.directionFallback, settings.regularization,
                                 problem.activeHorizon)) {
      result.exit = SolverExit::DirectionFailed;
      break;
    }
    Scalar stepLength = Scalar(1), slope = evaluation.gradient.dot(direction);
    bool accepted = false, stagnant = false;
    InputPlan<Scalar> candidate;
    for (int backtrackIndex = 0; backtrackIndex < settings.maxBacktracks; ++backtrackIndex) {
      if (budget.exhausted()) {
        result.exit = SolverExit::WorkBudgetExceeded;
        break;
      }
      candidate = inputPlan + stepLength * direction;
      if ((candidate.array() == inputPlan.array()).all()) {
        // No representable coordinate moved. Retrying this identical point
        // cannot satisfy meaningful descent, even if a rounded Armijo bound
        // equals the old cost. Keep its already verified gradient and plan.
        stagnant = true;
        break;
      }
      ++result.costEvaluations;
      bool finite = detail::evaluateValidatedCost(problem, candidate, trialCost, &trigonometry);
      if (!finite) {
        ++result.nonfiniteTrials;
      }
      if (finite && trialCost <= evaluation.cost + settings.armijoFraction * stepLength * slope) {
        accepted = true;
        break;
      }
      stepLength *= Scalar(.5);
    }
    if (!accepted) {
      if (result.exit != SolverExit::WorkBudgetExceeded)
        result.exit = stagnant ? SolverExit::StepStagnation : SolverExit::LineSearchFailed;
      break;
    }
    inputPlan = candidate;
    currentPlanVerified = false;
    result.iterations = iteration + 1;
    if (planInfinityNorm(InputPlan<Scalar>(stepLength * direction)) <=
        settings.relativeStepTolerance * (Scalar(1) + planInfinityNorm(inputPlan))) {
      result.exit = SolverExit::StepStagnation;
      break;
    }
  }
  if (!hasVerifiedPlan) {
    // Never publish a plan whose derivatives/cost/trajectory were not verified
    // in THIS immutable problem, even if the previous sample's plan was usable.
    result.usable = false;
    return result;
  }
  if (!currentPlanVerified) {
    if (budget.exhausted()) {
      inputPlan = verifiedPlan;
      result.verifiedFallback = true;
      result.exit = SolverExit::WorkBudgetExceeded;
    } else {
      ++result.derivativeEvaluations;
      if (detail::evaluateValidatedDerivatives(problem, inputPlan, evaluation, &trigonometry)) {
        verifiedCost = evaluation.cost;
        verifiedGradientNorm = planInfinityNorm(evaluation.gradient);
        verifiedFirst = evaluation.firstPrediction;
        verifiedLast = evaluation.lastPrediction;
      } else {
        inputPlan = verifiedPlan;
        result.verifiedFallback = true;
        result.exit = SolverExit::LastVerifiedPlan;
      }
    }
  }
  result.usable = true;
  result.cost = verifiedCost;
  result.gradientNorm = verifiedGradientNorm;
  result.firstPrediction = verifiedFirst;
  result.lastPrediction = verifiedLast;
  result.converged =
      std::isfinite(verifiedGradientNorm) && verifiedGradientNorm <= settings.gradientTolerance;
  if (result.converged) {
    result.exit = SolverExit::GradientTolerance;
  }
  return result;
}
} // namespace pendulum
#endif // ADAPTIVE_GAUSSNEWTON_H
