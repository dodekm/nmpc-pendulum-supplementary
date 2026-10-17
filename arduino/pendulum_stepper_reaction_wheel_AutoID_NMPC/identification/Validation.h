#ifndef ADAPTIVE_VALIDATION_H
#define ADAPTIVE_VALIDATION_H
#pragma once
#include "Fitter.h"
namespace identification {
template <class T> struct Validation {
  T rms = T(0), signalRms = T(0), peak = T(0), initialVelocity = T(0);
  T bandRms[kValidationBands]{}, bandSignal[kValidationBands]{};
  int bandSamples[kValidationBands]{};
  bool accepted = false;
};
// Cooperative fixed-storage prediction assessment. The caller must retain the
// immutable acquisition buffer until completion. Coefficients are copied/frozen.
// Only initial velocity is fitted to the reserved prefix; that prefix is then
// excluded from all scored errors. rms/peak are angular errors in radians.
template <class T> class Validator {
public:
  Validator() = default;
  Validator(const Validator &) = delete;
  Validator &operator=(const Validator &) = delete;
  Validation<T> result;
  bool begin(const Data<T, kValidationIntervals> &data, const FitResult<T> &fit) {
    active_ = false;
    result = Validation<T>();
    if (!fit.usable || !data.valid() || data.intervals != kValidationIntervals) {
      return false;
    }
    for (int j = 0; j < kShared; ++j) {
      if (!std::isfinite(fit.parameters[j])) {
        return false;
      }
      parameters_[j] = fit.parameters[j];
    }
    if (parameters_[0] < Policy<T>::minAlpha || parameters_[0] > Policy<T>::maxAlpha ||
        parameters_[1] < T(0) || parameters_[1] > Policy<T>::maxBeta ||
        std::abs(parameters_[2]) < Policy<T>::minGamma ||
        std::abs(parameters_[2]) > Policy<T>::maxGamma ||
        std::abs(parameters_[3]) > Policy<T>::maxBias || parameters_[4] < T(0) ||
        parameters_[4] > Policy<T>::maxFriction) {
      return false;
    }
    data_ = &data;
    prefixIteration_ = 0;
    errorSquare_ = signalSquare_ = T(0);
    resetPrediction(true);
    active_ = true;
    return true;
  }
  bool running() const { return active_; }
  void poll() {
    static_assert(Policy<T>::intervalsPerPoll > 0, "Validation budget must be positive.");
    for (int budget = 0; active_ && budget < Policy<T>::intervalsPerPoll; ++budget) {
      const T previous = point_ == 0 ? T(0) : data_->command[point_ - 1];
      if (!interval(state_, parameters_, data_->command[point_] - previous)) {
        active_ = false;
        return;
      }
      const T error = state_.angle - data_->angle[point_ + 1];
      // ONLY initial velocity is fitted to the reserved .5 s prefix, six
      // scalar Gauss-Newton steps. No plant coefficient uses held-out data.
      if (prefixIteration_ < kPrefixIterations) {
        jr_ += state_.sq[kShared] * error;
        jj_ += state_.sq[kShared] * state_.sq[kShared];
        if (++point_ == kPrefix) {
          if (!std::isfinite(jr_) || !std::isfinite(jj_) || !(jj_ > T(1e-8))) {
            active_ = false;
            return;
          }
          const T next = result.initialVelocity - jr_ / jj_;
          if (!std::isfinite(next)) {
            active_ = false;
            return;
          }
          result.initialVelocity =
              pendulum::clamp(next, -Policy<T>::maxInitialSpeed, Policy<T>::maxInitialSpeed);
          ++prefixIteration_;
          resetPrediction(prefixIteration_ < kPrefixIterations);
        }
      } else {
        if (point_ >= kPrefix) {
          const T square = error * error,
                  signal = data_->angle[point_ + 1] * data_->angle[point_ + 1];
          errorSquare_ += square;
          signalSquare_ += signal;
          result.peak = std::max(result.peak, std::abs(error));
          const int band = validationBand(point_);
          result.bandRms[band] += square;
          result.bandSignal[band] += signal;
          ++result.bandSamples[band];
        }
        if (++point_ == kValidationIntervals) {
          finish();
        }
      }
    }
  }

private:
  static constexpr int kPrefix = 5, kPrefixIterations = 6;
  void resetPrediction(bool sensitivity) {
    state_ = State<T>();
    state_.angle = data_->angle[0];
    state_.omega = result.initialVelocity;
    state_.sw[kShared] = sensitivity ? T(1) : T(0);
    point_ = 0;
    jr_ = jj_ = T(0);
  }
  void finish() {
    active_ = false;
    result.rms = std::sqrt(errorSquare_ / T(kValidationIntervals - kPrefix));
    result.signalRms = std::sqrt(signalSquare_ / T(kValidationIntervals - kPrefix));
    result.accepted = std::isfinite(result.rms) && std::isfinite(result.signalRms) &&
                      result.signalRms > Policy<T>::minSignal &&
                      result.rms < Policy<T>::maxValidationRms &&
                      result.rms < T(.005) + Policy<T>::maxRelativeValidation * result.signalRms &&
                      result.peak < T(.10);
    for (int band = 0; band < kValidationBands; ++band) {
      if (result.bandSamples[band] < 90) {
        result.accepted = false;
        continue;
      }
      result.bandRms[band] = std::sqrt(result.bandRms[band] / T(result.bandSamples[band]));
      result.bandSignal[band] = std::sqrt(result.bandSignal[band] / T(result.bandSamples[band]));
      if (!std::isfinite(result.bandRms[band]) || !std::isfinite(result.bandSignal[band]) ||
          result.bandRms[band] >= Policy<T>::maxValidationRms ||
          result.bandRms[band] >=
              T(.005) + Policy<T>::maxRelativeValidation * result.bandSignal[band]) {
        result.accepted = false;
      }
    }
  }
  const Data<T, kValidationIntervals> *data_ = nullptr;
  State<T> state_;
  T parameters_[kShared]{}, jr_ = T(0), jj_ = T(0), errorSquare_ = T(0), signalSquare_ = T(0);
  int point_ = 0, prefixIteration_ = 0;
  bool active_ = false;
};
// Host/test convenience wrapper; firmware polls the same validator cooperatively.
template <class T>
Validation<T> validate(const Data<T, kValidationIntervals> &data, const FitResult<T> &fit) {
  Validator<T> runner;
  if (runner.begin(data, fit)) {
    constexpr int work = kValidationIntervals + 6 * 5;
    constexpr int polls = (work + Policy<T>::intervalsPerPoll - 1) / Policy<T>::intervalsPerPoll;
    for (int i = 0; i < polls && runner.running(); ++i) {
      runner.poll();
    }
  }
  return runner.result;
}
} // namespace identification
#endif // ADAPTIVE_VALIDATION_H
