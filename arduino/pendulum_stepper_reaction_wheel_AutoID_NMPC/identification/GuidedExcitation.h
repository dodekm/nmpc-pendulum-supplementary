#ifndef ADAPTIVE_GUIDEDEXCITATION_H
#define ADAPTIVE_GUIDEDEXCITATION_H
#include "Config.h"
namespace identification {
// Held-speed excitation: phi/v=-gamma*s/(s^2+beta*s+alpha) for the
// viscous linear model. sqrt(alpha) locates its resonance. Amplitude and
// frequency bands account for nonlinear softening and estimated dry losses.
template <class T> class GuidedExcitation {
public:
  bool begin(T alpha, T beta, T gamma, T attemptScale = T(1), bool validation = false,
             T coulomb = T(0)) {
    ready_ = false;
    if (!std::isfinite(alpha) || alpha < Policy<T>::minAlpha || alpha > Policy<T>::maxAlpha ||
        !std::isfinite(beta) || beta < T(0) || beta > Policy<T>::maxBeta || !std::isfinite(gamma) ||
        std::abs(gamma) < Policy<T>::minGamma || std::abs(gamma) > Policy<T>::maxGamma ||
        !std::isfinite(coulomb) || coulomb < T(0) || coulomb > Policy<T>::maxFriction ||
        !std::isfinite(attemptScale) || attemptScale <= T(0) || attemptScale > T(1)) {
      return false;
    }
    natural_ = std::sqrt(alpha);
    validation_ = validation;
    targetExcursion_ = target(false);
    // The beta floor compensates finite ring-up, not a physical prior.
    // 4*c/(pi*wn) is the small-angle first-harmonic dry-loss contribution.
    const T seed = T(1.1) *
                   (targetExcursion_ * std::max(beta, T(.08)) +
                    T(4) * coulomb / (pendulum::kPi<T> * natural_)) /
                   std::abs(gamma);
    amplitude_ = targetAmplitude_ =
        pendulum::clamp(seed * attemptScale, T(.25), Policy<T>::excitationSpeed);
    phase_ = validation ? T(.65) : T(0);
    periodPeak_ = previousAngle_ = velocity_ = T(0);
    sampleCount_ = lastIncrease_ = 0;
    increaseIntervals_ = uint32_t(
        std::ceil(pendulum::clamp(T(.5) / std::max(beta, T(.03)), T(2), T(6)) / Policy<T>::period));
    frequency_ = natural_ * T(.85);
    ready_ = true;
    return true;
  }
  T command(int index, T measuredAngle) {
    const int count = validation_ ? kValidationIntervals : kTrainIntervals;
    if (!ready_ || index < 0 || index >= count || uint32_t(index) != sampleCount_ ||
        !std::isfinite(measuredAngle) || std::abs(measuredAngle) > Policy<T>::maxAngle) {
      return std::numeric_limits<T>::quiet_NaN();
    }
    const T time = T(index) * Policy<T>::period;
    const bool second = index >= 300, decay = (index >= 200 && index < 300) || index >= 500;
    if (index == 300) {
      const T next = target(true);
      targetAmplitude_ *= next / targetExcursion_;
      targetExcursion_ = next;
      periodPeak_ = T(0);
      lastIncrease_ = sampleCount_;
    }
    const T local = second ? time - T(30) : time;
    if (validation_) {
      frequency_ =
          natural_ *
          (second ? T(.96) + T(.02) * std::sin(pendulum::kTwoPi<T> * local / T(11) + T(.9))
                  : T(.99) + T(.015) * std::sin(pendulum::kTwoPi<T> * local / T(13) + T(.6)));
    } else {
      frequency_ = natural_ *
                   (second ? T(.965) + T(.025) * std::sin(pendulum::kTwoPi<T> * local / T(14))
                    : local < T(8)
                        ? T(.85) + T(.14) * local / T(8)
                        : T(.99) + T(.02) * std::sin(pendulum::kTwoPi<T> * (local - T(8)) / T(11)));
    }
    if (sampleCount_) {
      velocity_ += T(.35) * ((measuredAngle - previousAngle_) / Policy<T>::period - velocity_);
    }
    previousAngle_ = measuredAngle;
    const T ratio =
        velocity_ * velocity_ / (T(2) * natural_ * natural_) + T(1) - std::cos(measuredAngle);
    const T excursion = std::acos(pendulum::clamp(T(1) - ratio, -T(1), T(1)));
    periodPeak_ = std::max(periodPeak_, excursion);
    if (sampleCount_ && phase_ + frequency_ * Policy<T>::period >= pendulum::kTwoPi<T>) {
      if (!decay) {
        if (periodPeak_ > targetExcursion_ * T(1.08)) {
          targetAmplitude_ *= T(.80);
        } else if (periodPeak_ < targetExcursion_ * T(.85) &&
                   sampleCount_ - lastIncrease_ >= increaseIntervals_) {
          targetAmplitude_ *= T(1.12);
          lastIncrease_ = sampleCount_;
        }
      }
      periodPeak_ = T(0);
    }
    if (excursion > T(.7330382858)) {
      targetAmplitude_ *= T(.70); // anticipated 42 degree soft guard
    }
    targetAmplitude_ = pendulum::clamp(targetAmplitude_, T(.15), Policy<T>::excitationSpeed);
    amplitude_ += pendulum::clamp(targetAmplitude_ - amplitude_, -T(2) * Policy<T>::period,
                                  T(.5) * Policy<T>::period);
    const T allowed = std::min(amplitude_, T(.8) * Policy<T>::excitationAcceleration / frequency_);
    T envelope = std::min(T(1), local / T(2));
    if (local > T(17)) {
      envelope = std::min(envelope, std::max(T(0), (T(20) - local) / T(3)));
    }
    const T value = decay ? T(0) : allowed * envelope * std::sin(phase_);
    phase_ += frequency_ * Policy<T>::period;
    phase_ -= pendulum::kTwoPi<T> * std::floor(phase_ / pendulum::kTwoPi<T>);
    ++sampleCount_;
    return value;
  }
  T naturalFrequency() const { return natural_; }
  T currentFrequency() const { return frequency_; }
  T currentAmplitude() const { return amplitude_; }
  T requestedExcursion() const { return targetExcursion_; }

private:
  T target(bool second) const {
    return (validation_ ? (second ? T(30) : T(20)) : (second ? T(35) : T(25))) * pendulum::kPi<T> /
           T(180);
  }
  T natural_ = T(0), frequency_ = T(0), phase_ = T(0), amplitude_ = T(0), targetAmplitude_ = T(0),
    targetExcursion_ = T(0), periodPeak_ = T(0);
  T previousAngle_ = T(0), velocity_ = T(0);
  uint32_t sampleCount_ = 0, lastIncrease_ = 0, increaseIntervals_ = 60;
  bool validation_ = false, ready_ = false;
};
} // namespace identification
#endif
