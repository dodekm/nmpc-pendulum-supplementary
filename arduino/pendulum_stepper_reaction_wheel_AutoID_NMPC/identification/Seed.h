#ifndef ADAPTIVE_SEED_H
#define ADAPTIVE_SEED_H
#pragma once
#include "StreamingQR.h"
namespace identification {
template <class T> bool weakSeed(const Data<T, kTrainIntervals> &data, T *p) {
  StreamingQR<T, 4> qr;
  // Compact multiscale weak tests. Integrate HELD speed exactly, instead
  // of interpolating command samples across discontinuities. Angle differences
  // below form integral functionals, not fitted velocity observations.
  const int spans[] = {6, 12, 30, 60};
  for (int span : spans) {
    const T h = Policy<T>::period, length = T(span) * h, frequency = pendulum::kTwoPi<T> / length;
    const T scale = length * std::sqrt(length) / T(5.196152422706632);
    for (int begin = 0; begin + span <= data.intervals; begin += std::max(2, span / 6)) {
      T row[4]{}, rhs = T(0), previousW = T(0), previousPrimitive = T(0);
      row[3] = -length / T(2);
      for (int j = 0; j < span; ++j) {
        const int point = begin + j;
        const T t = T(j + 1) * h,
                w = j + 1 == span ? T(0) : (T(1) - std::cos(frequency * t)) / T(2),
                primitive = j + 1 == span ? length / T(2)
                                          : t / T(2) - std::sin(frequency * t) / (T(2) * frequency),
                deltaW = w - previousW, deltaPhi = data.angle[point + 1] - data.angle[point],
                slope = deltaPhi / h;
        row[1] += slope * (primitive - previousPrimitive);
        row[2] -= data.command[point] * deltaW;
        rhs += slope * deltaW;
        const T nodes[] = {T(.21132486540518712), T(.78867513459481288)};
        for (T node : nodes) {
          const T localTime = (T(j) + node) * h,
                  localW = (T(1) - std::cos(frequency * localTime)) / T(2);
          row[0] += h / T(2) * localW * std::sin(data.angle[point] + node * deltaPhi);
        }
        previousW = w;
        previousPrimitive = primitive;
      }
      for (T &value : row) {
        value *= scale;
      }
      if (!qr.add(row, rhs * scale)) {
        return false;
      }
    }
  }
  T values[4]{};
  if (!qr.solve(values, T(1e-6))) {
    return false;
  }
  p[0] = pendulum::clamp(values[0], Policy<T>::minAlpha * T(2), Policy<T>::maxAlpha * T(.95));
  p[1] = pendulum::clamp(values[1], T(.005), T(.8));
  p[2] = pendulum::clamp(values[2], -Policy<T>::maxGamma / T(2), Policy<T>::maxGamma / T(2));
  if (std::abs(p[2]) < Policy<T>::minGamma) {
    return false;
  }
  p[3] = pendulum::clamp(values[3], -T(.5), T(.5));
  p[4] = T(0); // nested viscous model is the seed, not a prescribed friction value
  for (int window = 0; window < kWindows; ++window) {
    const int i = window * kWindowIntervals;
    p[kShared + window] = (data.angle[i + 2] - data.angle[i]) / (T(2) * Policy<T>::period);
  }
  return true;
}
} // namespace identification
#endif // ADAPTIVE_SEED_H
