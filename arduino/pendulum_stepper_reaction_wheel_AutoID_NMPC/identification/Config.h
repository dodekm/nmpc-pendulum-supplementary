#ifndef ADAPTIVE_CONFIG_H
#define ADAPTIVE_CONFIG_H
#pragma once
#include "control/Model.h"
namespace identification {
constexpr int kTrainIntervals = 600, kValidationIntervals = 600;
constexpr int kWindows = 6, kWindowIntervals = kTrainIntervals / kWindows;
constexpr int kShared = 5, kVariables = kShared + kWindows, kSensitivity = kShared + 1;
constexpr int kValidationBands = 4;
inline int validationBand(int interval) {
  return interval < 200 ? 0 : interval < 300 ? 1 : interval < 500 ? 2 : 3;
}
template <class T> struct Policy {
  static constexpr T period = T(.1), maxStep = T(.02), huber = T(.012);
  static constexpr T minAlpha = T(.25), maxAlpha = T(80), maxBeta = T(2), maxGamma = T(.4);
  static constexpr T minGamma = T(.003), maxBias = T(2), maxInitialSpeed = T(10);
  static constexpr T maxFriction = T(.5),
                     frictionVelocityScale = pendulum::ModelParameters<T>::frictionVelocityScale;
  static constexpr T excitationAcceleration = T(25), excitationSpeed = T(8);
  static constexpr T maxAngle = T(.8726646259971648), minSignal = T(.007);
  static constexpr T maxTrainingRms = T(.05), maxValidationRms = T(.07);
  static constexpr T maxRelativeValidation = T(.30), maxCondition = T(20000);
  static constexpr int maxIterations = 60, intervalsPerPoll = 12;
};
template <class T, int Capacity> struct Data {
  T angle[Capacity + 1]{}, command[Capacity]{};
  int intervals = 0;
  bool valid() const {
    if (intervals < 20 || intervals > Capacity) {
      return false;
    }
    for (int i = 0; i <= intervals; ++i) {
      if (!std::isfinite(angle[i]) || std::abs(angle[i]) > Policy<T>::maxAngle) {
        return false;
      }
    }
    for (int i = 0; i < intervals; ++i) {
      if (!std::isfinite(command[i]) || std::abs(command[i]) > Policy<T>::excitationSpeed) {
        return false;
      }
    }
    return true;
  }
};
template <class T> T coarseChirp(int interval, T amplitudeScale = T(1)) {
  const T duration = T(kTrainIntervals) * Policy<T>::period;
  const T t = T(interval) * Policy<T>::period;
  const T f0 = T(.75), f1 = T(7);
  const T phase = f0 * t + (f1 - f0) * t * t / (T(2) * duration);
  const T envelope = std::min(T(1), std::min(t / T(2), (duration - t) / T(2)));
  const T amplitude = T(1.5) + T(4) * t / duration;
  return amplitudeScale * amplitude * std::max(T(0), envelope) * std::sin(phase);
}
template <class T> T limitedVelocity(T requested, T previous) {
  if (!std::isfinite(requested) || !std::isfinite(previous) ||
      std::abs(previous) > Policy<T>::excitationSpeed) {
    return std::numeric_limits<T>::quiet_NaN();
  }
  const T change = Policy<T>::excitationAcceleration * Policy<T>::period;
  return pendulum::clamp(requested, std::max(-Policy<T>::excitationSpeed, previous - change),
                         std::min(Policy<T>::excitationSpeed, previous + change));
}
} // namespace identification
#endif // ADAPTIVE_CONFIG_H
