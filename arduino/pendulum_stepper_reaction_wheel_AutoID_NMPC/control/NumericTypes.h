#ifndef ADAPTIVE_NUMERICTYPES_H
#define ADAPTIVE_NUMERICTYPES_H
#pragma once
#include "Precision.h"
#ifndef EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_VECTORIZE
#endif
#ifndef EIGEN_UNROLLING_LIMIT
#define EIGEN_UNROLLING_LIMIT 0
#endif
#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/Eigenvalues"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace pendulum {
constexpr int kHorizon = 16; // 1.6 s; retains useful look-ahead for softer rigs
template <class Scalar, int Rows, int Columns = 1>
using Matrix = Eigen::Matrix<Scalar, Rows, Columns>;
template <class Scalar> using InputPlan = Matrix<Scalar, kHorizon>;
template <class Scalar> inline Scalar clamp(Scalar v, Scalar lo, Scalar hi) {
  return std::max(lo, std::min(v, hi));
}
template <class Scalar> inline Scalar planInfinityNorm(const InputPlan<Scalar> &v) {
  return v.cwiseAbs().maxCoeff();
}
} // namespace pendulum
#endif // ADAPTIVE_NUMERICTYPES_H
