#ifndef ADAPTIVE_STREAMINGQR_H
#define ADAPTIVE_STREAMINGQR_H
#pragma once
#include "Config.h"
namespace identification {
// Givens QR of weighted residual rows; never form J'J for the fit direction.
// r and z are the compressed upper-triangular system R*step=z. They remain
// explicit fixed arrays because the fitter also uses this factor for damping,
// information checks and uncertainty. add() copies its input row; solve()
// writes a solution only as its backward substitution progresses.
template <class T, int N> struct StreamingQR {
  T r[N][N]{}, z[N]{};
  void clear() {
    for (int i = 0; i < N; ++i) {
      z[i] = T(0);
      for (int j = 0; j < N; ++j) {
        r[i][j] = T(0);
      }
    }
  }
  bool add(const T *values, T target) {
    T row[N];
    for (int i = 0; i < N; ++i) {
      row[i] = values[i];
      if (!std::isfinite(row[i])) {
        return false;
      }
    }
    if (!std::isfinite(target)) {
      return false;
    }
    for (int i = 0; i < N; ++i) {
      const T h = std::hypot(r[i][i], row[i]);
      if (!std::isfinite(h)) {
        return false;
      }
      if (h == T(0)) {
        continue;
      }
      const T c = r[i][i] / h, s = row[i] / h;
      r[i][i] = h;
      for (int j = i + 1; j < N; ++j) {
        const T a = r[i][j], b = row[j];
        r[i][j] = c * a + s * b;
        row[j] = -s * a + c * b;
      }
      const T old = z[i];
      z[i] = c * old + s * target;
      target = -s * old + c * target;
    }
    for (int i = 0; i < N; ++i) {
      if (!std::isfinite(z[i])) {
        return false;
      }
      for (int j = i; j < N; ++j) {
        if (!std::isfinite(r[i][j])) {
          return false;
        }
      }
    }
    return true;
  }
  bool solve(T *solution, T relativeTolerance = T(1e-7)) const {
    T largest = T(0);
    for (int i = 0; i < N; ++i) {
      largest = std::max(largest, std::abs(r[i][i]));
    }
    if (!(largest > T(0)) || !std::isfinite(largest)) {
      return false;
    }
    for (int i = N - 1; i >= 0; --i) {
      if (!(std::abs(r[i][i]) > relativeTolerance * largest)) {
        return false;
      }
      T b = z[i];
      for (int j = i + 1; j < N; ++j) {
        b -= r[i][j] * solution[j];
      }
      solution[i] = b / r[i][i];
      if (!std::isfinite(solution[i])) {
        return false;
      }
    }
    return true;
  }
};
} // namespace identification
#endif // ADAPTIVE_STREAMINGQR_H
