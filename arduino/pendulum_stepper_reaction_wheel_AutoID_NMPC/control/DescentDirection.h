#ifndef PENDULUM_DESCENT_DIRECTION_H
#define PENDULUM_DESCENT_DIRECTION_H
#include "NumericTypes.h"
#include "ControlTuning.h"
#include "Eigen/Cholesky"

namespace pendulum {
namespace detail {
template <class Scalar>
bool activeCholeskyDirection(const Matrix<Scalar, kHorizon, kHorizon> &matrix,
                             const InputPlan<Scalar> &gradient,
                             InputPlan<Scalar> &direction, int active) {
  // Inactive decisions have independent positive input penalties. Never
  // silently discard couplings when this public kernel is used independently.
  for (int row = active; row < kHorizon; ++row) {
    if (!(matrix(row, row) > Scalar(0))) return false;
    for (int column = 0; column < kHorizon; ++column)
      if (column != row && matrix(row, column) != Scalar(0)) return false;
    direction(row) = -gradient(row) / matrix(row, row);
  }
  // Runtime extent, compile-time bounded storage: no dynamic Eigen allocation.
  using ActiveSquare = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, kHorizon, kHorizon>;
  using ActiveVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, kHorizon, 1>;
  ActiveSquare block = matrix.topLeftCorner(active, active);
  Eigen::LLT<ActiveSquare, Eigen::Lower> factorization(block);
  if (factorization.info() != Eigen::Success) return false;
  const Scalar epsilon = std::numeric_limits<Scalar>::epsilon();
  const Scalar reciprocalCondition = factorization.rcond();
  if (!std::isfinite(reciprocalCondition) || reciprocalCondition <= epsilon) return false;
  const ActiveVector rhs = -gradient.head(active);
  const ActiveVector solution = factorization.solve(rhs);
  direction.head(active) = solution;
  const Scalar slope = gradient.dot(direction);
  if (!direction.allFinite() || !std::isfinite(slope) || !(slope < Scalar(0))) return false;
  // Backward-error check is independent of the estimated condition number.
  const ActiveVector residual = block * solution - rhs;
  const ActiveVector magnitude = block.cwiseAbs() * solution.cwiseAbs() + rhs.cwiseAbs();
  return residual.allFinite() && magnitude.allFinite() &&
         residual.cwiseAbs().maxCoeff() <= Scalar(64) * epsilon * magnitude.maxCoeff();
}
} // namespace detail

template <class Scalar>
bool computeDescentDirection(const Matrix<Scalar, kHorizon, kHorizon> &hessian,
                             const InputPlan<Scalar> &gradient, InputPlan<Scalar> &direction,
                             bool &usedFallback,
                             Scalar regularization = ControlTuning<Scalar>::regularization,
                             int activeHorizon = kHorizon) {
  if (!gradient.allFinite() || !std::isfinite(regularization) || regularization < Scalar(0) ||
      activeHorizon < 1 || activeHorizon > kHorizon) {
    direction.setZero();
    return false;
  }
  using Square = Matrix<Scalar, kHorizon, kHorizon>;
  Square regularizedHessian = Scalar(.5) * hessian + Scalar(.5) * hessian.transpose();
  regularizedHessian.diagonal().array() += regularization;
  const Scalar machineEpsilon = std::numeric_limits<Scalar>::epsilon();
  if (regularizedHessian.allFinite()) {
    if (detail::activeCholeskyDirection(regularizedHessian, gradient, direction, activeHorizon))
      return true;
    // General coupled/indefinite or roundoff-damaged matrices keep the original
    // pivoted solve, conditioning checks and scaled/damped descent fallbacks.
    Eigen::PartialPivLU<Square> factorization(regularizedHessian);
    const Scalar reciprocalCondition = factorization.rcond();
    if (std::isfinite(reciprocalCondition) && reciprocalCondition > machineEpsilon) {
      direction = factorization.solve(-gradient);
      const Scalar slope = gradient.dot(direction);
      if (direction.allFinite() && std::isfinite(slope) && slope < Scalar(0)) return true;
    }
  }
  usedFallback = true;
  const Scalar matrixScale = regularizedHessian.cwiseAbs().maxCoeff(),
               gradientScale = planInfinityNorm(gradient);
  if (std::isfinite(matrixScale) && matrixScale > Scalar(0) && std::isfinite(gradientScale) &&
      gradientScale > Scalar(0)) {
    InputPlan<Scalar> diagonalScaling;
    for (int rowIndex = 0; rowIndex < kHorizon; ++rowIndex) {
      diagonalScaling(rowIndex) =
          Scalar(1) /
          std::sqrt(std::max(std::abs(regularizedHessian(rowIndex, rowIndex) / matrixScale),
                             std::sqrt(machineEpsilon)));
    }
    Square equilibratedHessian = diagonalScaling.asDiagonal() * (regularizedHessian / matrixScale) *
                                 diagonalScaling.asDiagonal();
    InputPlan<Scalar> scaledGradient = diagonalScaling.cwiseProduct(gradient / gradientScale);
    const Scalar damping[] = {Scalar(0), std::sqrt(machineEpsilon), Scalar(1e-4), Scalar(1e-2)};
    for (Scalar dampingValue : damping) {
      Square damped = equilibratedHessian;
      damped.diagonal().array() += dampingValue;
      Eigen::PartialPivLU<Square> factorization(damped);
      const Scalar reciprocalCondition = factorization.rcond();
      if (std::isfinite(reciprocalCondition) && reciprocalCondition > std::sqrt(machineEpsilon)) {
        InputPlan<Scalar> equilibratedDirection = factorization.solve(-scaledGradient);
        direction =
            diagonalScaling.cwiseProduct(equilibratedDirection) * (gradientScale / matrixScale);
        const Scalar slope = gradient.dot(direction);
        if (direction.allFinite() && std::isfinite(slope) && slope < Scalar(0)) return true;
      }
    }
  }
  direction = -gradient / std::max(Scalar(1), planInfinityNorm(gradient));
  Scalar slope = gradient.dot(direction);
  if (!std::isfinite(slope)) {
    direction /= Scalar(kHorizon);
    slope = gradient.dot(direction);
  }
  return direction.allFinite() && std::isfinite(slope) && slope < Scalar(0);
}
} // namespace pendulum
#endif
