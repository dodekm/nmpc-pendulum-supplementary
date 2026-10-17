#ifndef ADAPTIVE_TERMINALDESIGN_H
#define ADAPTIVE_TERMINALDESIGN_H
#pragma once
#include "RungeKutta.h"
#include "MatrixValidation.h"
namespace pendulum {
template <class T> bool isSchurStable(const Matrix<T, 3, 3> &a) {
  if (!a.allFinite()) {
    return false;
  }
  const T trace = a.trace(), second = (trace * trace - (a * a).trace()) / T(2);
  T c[4] = {T(1), -trace, second, -a.determinant()};
  for (int degree = 3; degree > 0; --degree) {
    if (!std::isfinite(c[0]) || !(c[0] > T(0)) || !(std::abs(c[degree]) < c[0])) {
      return false;
    }
    T next[4]{};
    for (int j = 0; j < degree; ++j) {
      next[j] = c[0] * c[j] - c[degree] * c[degree - j];
    }
    const T scale = next[0];
    if (!(scale > T(0))) {
      return false;
    }
    for (int j = 0; j < degree; ++j) {
      c[j] = next[j] / scale;
    }
  }
  return true;
}
// Cooperative DARE iteration using the SAME one-step RK4 linearization as MPC.
// begin() copies its inputs; poll() performs bounded work. complete reports
// termination, while valid separately reports an admissible terminal design.
template <class T> class TerminalDesign {
public:
  Matrix<T, 3, 3> tail = Matrix<T, 3, 3>::Zero(), Ad = Matrix<T, 3, 3>::Zero();
  Matrix<T, 3> Bd = Matrix<T, 3>::Zero();
  T residual = std::numeric_limits<T>::infinity();
  int iterations = 0;
  bool complete = false, valid = false;
  bool begin(const ModelParameters<T> &model, const Matrix<T, 3, 3> &weight, T inputWeight,
             bool upright) {
    initialized_ = false;
    complete = valid = false;
    iterations = 0;
    residual = std::numeric_limits<T>::infinity();
    tail.setZero();
    Ad.setZero();
    Bd.setZero();
    q_ = weight;
    r_ = inputWeight;
    p_ = weight;
    if (!model.isValid() || !std::isfinite(r_) || !(r_ > T(0)) ||
        !isSymmetricPositiveSemidefinite(q_)) {
      complete = true;
      return false;
    }
    Matrix<T, 3> x = Matrix<T, 3>::Zero(), next;
    if (upright) {
      x(1) = kPi<T>;
    }
    if (!integrateRK4(x, T(0), model, T(0), T(0), next, &Ad, &Bd)) {
      complete = true;
      return false;
    }
    initialized_ = true;
    return true;
  }
  void poll(int budget = 32) {
    // Never iterate stale data after a failed begin(), or consume an invalid budget.
    if (complete || !initialized_ || budget <= 0) {
      return;
    }
    const T tolerance = std::is_same<T, float>::value ? T(3e-6) : T(1e-11);
    while (budget--) {
      const Matrix<T, 3> pb = p_ * Bd;
      const T denominator = r_ + Bd.dot(pb);
      if (!p_.allFinite() || !(denominator > T(0)) || !std::isfinite(denominator)) {
        complete = true;
        return;
      }
      const Matrix<T, 3, 3> next = (q_ + Ad.transpose() * p_ * Ad -
                                    (Ad.transpose() * pb) * (pb.transpose() * Ad) / denominator)
                                       .eval();
      if (!next.allFinite()) {
        complete = true;
        return;
      }
      residual = (next - p_).cwiseAbs().maxCoeff() / std::max(T(1), p_.cwiseAbs().maxCoeff());
      p_ = (T(.5) * next + T(.5) * next.transpose()).eval();
      if (++iterations > 12000) {
        complete = true;
        return;
      }
      if (residual < tolerance) {
        const Matrix<T, 3> finalPb = p_ * Bd;
        const T finalDenominator = r_ + Bd.dot(finalPb);
        if (!(finalDenominator > T(0)) || !std::isfinite(finalDenominator)) {
          complete = true;
          return;
        }
        const Matrix<T, 1, 3> gain = (finalPb.transpose() * Ad / finalDenominator).eval();
        if (!gain.allFinite()) {
          complete = true;
          return;
        }
        tail = (p_ - q_).eval();
        tail = (T(.5) * tail + T(.5) * tail.transpose()).eval();
        valid =
            isSymmetricPositiveSemidefinite(tail) && isSchurStable(Matrix<T, 3, 3>(Ad - Bd * gain));
        complete = true;
        return;
      }
    }
  }

private:
  bool initialized_ = false;
  Matrix<T, 3, 3> p_ = Matrix<T, 3, 3>::Zero(), q_ = Matrix<T, 3, 3>::Zero();
  T r_ = T(0);
};
} // namespace pendulum
#endif // ADAPTIVE_TERMINALDESIGN_H
