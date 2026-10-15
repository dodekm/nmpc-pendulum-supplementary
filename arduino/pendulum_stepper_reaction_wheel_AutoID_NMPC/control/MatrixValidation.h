#ifndef ADAPTIVE_MATRIXVALIDATION_H
#define ADAPTIVE_MATRIXVALIDATION_H
#pragma once
#include "NumericTypes.h"

namespace pendulum {
// Relative roundoff test, independent of physical units and matrix magnitude.
// Normalize BEFORE summing: finite entries may otherwise overflow a row norm.
// Do not project or modify the supplied matrix; failure is reported to callers.
template <class Scalar, int Size>
bool isSymmetricPositiveSemidefinite(const Matrix<Scalar, Size, Size> &matrix) {
  if (!matrix.allFinite()) {
    return false;
  }
  const Scalar scale = matrix.cwiseAbs().maxCoeff();
  if (scale == Scalar(0)) {
    return true;
  }
  Matrix<Scalar, Size, Size> normalized;
  for (int column = 0; column < Size; ++column) {
    for (int row = 0; row < Size; ++row) {
      // Explicit division also handles subnormal scales whose reciprocal would
      // overflow. No fast-math reciprocal substitution is permitted.
      normalized(row, column) = matrix(row, column) / scale;
    }
  }
  const Scalar tolerance = Scalar(100) * std::numeric_limits<Scalar>::epsilon() *
                           normalized.cwiseAbs().rowwise().sum().maxCoeff();
  if ((normalized - normalized.transpose()).cwiseAbs().rowwise().sum().maxCoeff() > tolerance) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Matrix<Scalar, Size, Size>> eigenSolver(
      (Scalar(.5) * normalized + Scalar(.5) * normalized.transpose()).eval(),
      Eigen::EigenvaluesOnly);
  return eigenSolver.info() == Eigen::Success && eigenSolver.eigenvalues().allFinite() &&
         eigenSolver.eigenvalues().minCoeff() >= -tolerance;
}
} // namespace pendulum
#endif // ADAPTIVE_MATRIXVALIDATION_H
