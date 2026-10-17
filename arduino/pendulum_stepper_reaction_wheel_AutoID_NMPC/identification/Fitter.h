#ifndef ADAPTIVE_FITTER_H
#define ADAPTIVE_FITTER_H
#pragma once
#include "Seed.h"
#include "SensitivityModel.h"
#include "numerics/CompensatedSum.h"
namespace identification {
enum class FitExit : uint8_t {
  Running,
  Converged,
  IterationLimit,
  Stagnation,
  NumericalFailure,
  InsufficientData,
  PoorInformation,
  PrecisionLimited
};
// parameters contains kShared physical coefficients followed by kWindows
// nuisance initial velocities. usable covers training/information admission;
// a separate frozen-coefficient validation is required before control begins.
template <class T> struct FitResult {
  T parameters[kVariables]{}, rms = T(0), gradient = T(0),
                              cost = std::numeric_limits<T>::infinity();
  T standardDeviation[kShared]{}, condition = std::numeric_limits<T>::infinity();
  T linearDecrease = T(0), scaledStep = std::numeric_limits<T>::infinity(), costResolution = T(0);
  int iterations = 0, accepted = 0, rejected = 0;
  FitExit exit = FitExit::InsufficientData;
  bool usable = false;
};
// Cooperative, fixed-storage robust nonlinear least squares. begin() borrows
// data until running() becomes false: its owner must keep the buffer alive and
// unchanged. poll() advances the fit without blocking acquisition servicing.
template <class T> class Fitter {
public:
  Fitter() = default;
  Fitter(const Fitter &) = delete;
  Fitter &operator=(const Fitter &) = delete;
  FitResult<T> result;
  bool begin(const Data<T, kTrainIntervals> &data, T stiffnessFactor = T(1)) {
    data_ = &data;
    result = FitResult<T>();
    if (!std::isfinite(stiffnessFactor) || !(stiffnessFactor > T(0)) || !data.valid() ||
        data.intervals != kTrainIntervals || !weakSeed(data, base_)) {
      return false;
    }
    base_[0] = pendulum::clamp(base_[0] * stiffnessFactor, Policy<T>::minAlpha * T(2),
                               Policy<T>::maxAlpha * T(.95));
    for (int j = 0; j < kVariables; ++j) {
      scale_[j] = j == 0             ? std::max(T(1), base_[0])
                  : j == 1           ? T(.1)
                  : j == 2           ? std::max(T(.01), std::abs(base_[2]))
                  : j == 3 || j == 4 ? T(.1)
                                     : T(.5);
    }
    lambda_ = T(.001);
    trials_ = 0;
    mode_ = Mode::Base;
    result.exit = FitExit::Running;
    startEvaluation(base_);
    return true;
  }
  bool running() const { return result.exit == FitExit::Running; }
  // At most intervalsPerPoll integration intervals; no blocking acquisition.
  void poll() {
    if (!running()) {
      return;
    }
    if (mode_ == Mode::Finalize) {
      finalize();
      return;
    }
    if (mode_ == Mode::Propose) {
      propose();
      return;
    }
    int budget = Policy<T>::intervalsPerPoll;
    while (budget-- && point_ < kTrainIntervals) {
      if (point_ % kWindowIntervals == 0) {
        state_ = State<T>();
        state_.angle = data_->angle[point_];
        state_.omega = evaluated_[kShared + point_ / kWindowIntervals];
        state_.sw[kShared] = T(1);
      }
      const T previous = point_ == 0 ? T(0) : data_->command[point_ - 1];
      if (!interval(state_, evaluated_, data_->command[point_] - previous)) {
        evaluationFailed();
        return;
      }
      const T residual = state_.angle - data_->angle[point_ + 1], a = std::abs(residual);
      const T weight = a <= Policy<T>::huber ? T(1) : Policy<T>::huber / a;
      costSum_.add(a <= Policy<T>::huber ? T(.5) * residual * residual
                                         : Policy<T>::huber * (a - T(.5) * Policy<T>::huber));
      squareSum_.add(residual * residual);
      cost_ = costSum_.value();
      square_ = squareSum_.value();
      T row[kVariables]{};
      for (int j = 0; j < kShared; ++j) {
        row[j] = state_.sq[j] * scale_[j];
      }
      const int local = kShared + point_ / kWindowIntervals;
      row[local] = state_.sq[kShared] * scale_[local];
      const T root = std::sqrt(weight);
      for (int j = 0; j < kVariables; ++j) {
        row[j] *= root;
      }
      if (!qr_.add(row, -root * residual) || !std::isfinite(cost_) || !std::isfinite(square_)) {
        evaluationFailed();
        return;
      }
      ++point_;
    }
    if (point_ != kTrainIntervals) {
      return;
    }
    if (mode_ == Mode::Base) {
      baseQR_ = qr_;
      baseCost_ = cost_;
      baseSquare_ = square_;
      T norm = T(0);
      bool anyActive = false;
      for (int j = 0; j < kVariables; ++j) {
        T g = T(0);
        for (int i = 0; i <= j; ++i) {
          g -= qr_.r[i][j] * qr_.z[i];
        }
        if (!std::isfinite(g)) {
          finalize(FitExit::NumericalFailure);
          return;
        }
        const T low = j == 0             ? Policy<T>::minAlpha
                      : j == 1 || j == 4 ? T(0)
                      : j == 2           ? -Policy<T>::maxGamma
                      : j == 3           ? -Policy<T>::maxBias
                                         : -Policy<T>::maxInitialSpeed;
        const T high = j == 0   ? Policy<T>::maxAlpha
                       : j == 1 ? Policy<T>::maxBeta
                       : j == 2 ? Policy<T>::maxGamma
                       : j == 3 ? Policy<T>::maxBias
                       : j == 4 ? Policy<T>::maxFriction
                                : Policy<T>::maxInitialSpeed;
        active_[j] = (base_[j] <= low && g > T(0)) || (base_[j] >= high && g < T(0));
        anyActive = anyActive || active_[j];
        if (active_[j]) {
          g = T(0);
        }
        norm = std::max(norm, std::abs(g));
      }
      result.gradient = norm;
      result.cost = baseCost_;
      if (!prepareLinearModel(anyActive)) {
        finalize(FitExit::NumericalFailure);
        return;
      }
      if (norm < T(1e-5)) {
        mode_ = Mode::Finalize;
        result.exit = FitExit::Running;
        return;
      }
      if (precisionLimitVerified()) {
        finalize(FitExit::PrecisionLimited);
        return;
      }
      if (result.iterations >= Policy<T>::maxIterations) {
        finalize(FitExit::IterationLimit);
        return;
      }
      propose();
    } else {
      const T actual = baseCost_ - cost_, ratio = predicted_ > T(0) ? actual / predicted_ : T(-1);
      if (actual > T(0) && ratio > T(1e-4)) {
        for (int j = 0; j < kVariables; ++j) {
          base_[j] = trial_[j];
        }
        ++result.accepted;
        ++result.iterations;
        trials_ = 0;
        lambda_ = std::max(T(1e-7), lambda_ * (ratio > T(.75) ? T(.3) : T(1)));
        mode_ = Mode::Base;
        startEvaluation(base_);
      } else {
        reject();
      }
    }
  }

private:
  enum class Mode { Base, Trial, Finalize, Propose };
  const Data<T, kTrainIntervals> *data_ = nullptr;
  T base_[kVariables]{}, trial_[kVariables]{}, scale_[kVariables]{};
  const T *evaluated_ = nullptr;
  T cost_ = T(0), square_ = T(0), baseCost_ = T(0), baseSquare_ = T(0), lambda_ = T(.001),
    predicted_ = T(0);
  int point_ = 0, trials_ = 0;
  Mode mode_ = Mode::Base;
  State<T> state_;
  pendulum::numerics::CompensatedSum<T> costSum_, squareSum_;
  StreamingQR<T, kVariables> qr_, baseQR_, linearQR_;
  bool active_[kVariables]{};
  bool prepareLinearModel(bool anyActive) {
    if (!anyActive) {
      linearQR_ = baseQR_;
      return true;
    }
    // Re-triangularize the compressed residual model with KKT-active columns
    // held fixed. Projecting an unconstrained full solution can otherwise
    // stall forever at c=0 even for a genuinely friction-free plant.
    linearQR_.clear();
    for (int i = 0; i < kVariables; ++i) {
      T row[kVariables]{};
      for (int j = 0; j < kVariables; ++j) {
        row[j] = active_[j] ? T(0) : baseQR_.r[i][j];
      }
      if (!linearQR_.add(row, baseQR_.z[i])) {
        return false;
      }
    }
    for (int j = 0; j < kVariables; ++j) {
      if (active_[j]) {
        T row[kVariables]{};
        row[j] = T(1);
        if (!linearQR_.add(row, T(0))) {
          return false;
        }
      }
    }
    return true;
  }
  void startEvaluation(const T *p) {
    evaluated_ = p;
    point_ = 0;
    cost_ = square_ = T(0);
    costSum_.clear();
    squareSum_.clear();
    qr_.clear();
  }
  void propose() {
    StreamingQR<T, kVariables> damped = linearQR_;
    T step[kVariables]{};
    for (int j = 0; j < kVariables; ++j) {
      T row[kVariables]{};
      T norm = T(0);
      for (int i = 0; i <= j; ++i) {
        norm += linearQR_.r[i][j] * linearQR_.r[i][j];
      }
      row[j] = std::sqrt(lambda_ * std::max(norm, T(1e-6)));
      if (!damped.add(row, T(0))) {
        finalize(FitExit::NumericalFailure);
        return;
      }
    }
    if (!damped.solve(step, T(1e-8))) {
      reject();
      return;
    }
    T length = T(0);
    for (int j = 0; j < kVariables; ++j) {
      length = std::max(length, std::abs(step[j]));
    }
    const T trust = length > T(2) ? T(2) / length : T(1);
    for (int j = 0; j < kVariables; ++j) {
      const T low = j == 0             ? Policy<T>::minAlpha
                    : j == 1 || j == 4 ? T(0)
                    : j == 2           ? -Policy<T>::maxGamma
                    : j == 3           ? -Policy<T>::maxBias
                                       : -Policy<T>::maxInitialSpeed;
      const T high = j == 0   ? Policy<T>::maxAlpha
                     : j == 1 ? Policy<T>::maxBeta
                     : j == 2 ? Policy<T>::maxGamma
                     : j == 3 ? Policy<T>::maxBias
                     : j == 4 ? Policy<T>::maxFriction
                              : Policy<T>::maxInitialSpeed;
      trial_[j] = pendulum::clamp(base_[j] + trust * scale_[j] * step[j], low, high);
      step[j] = (trial_[j] - base_[j]) / scale_[j];
    }
    predicted_ = T(0);
    for (int i = 0; i < kVariables; ++i) {
      T movement = T(0);
      for (int j = i; j < kVariables; ++j) {
        movement += baseQR_.r[i][j] * step[j];
      }
      predicted_ += baseQR_.z[i] * movement - T(.5) * movement * movement;
    }
    mode_ = Mode::Trial;
    startEvaluation(trial_);
  }
  bool precisionLimitVerified() {
    // Use the UNDAMPED local GN model: a huge damping parameter must never
    // manufacture a tiny step and a successful termination. Both independent
    // information gates and held-out prediction remain mandatory.
    T step[kVariables]{};
    if (!linearQR_.solve(step, T(1e-8))) {
      return false;
    }
    T maximum = T(0);
    pendulum::numerics::CompensatedSum<T> decrease;
    for (int j = 0; j < kVariables; ++j) {
      if (!std::isfinite(step[j])) {
        return false;
      }
      maximum = std::max(maximum, std::abs(step[j]));
    }
    for (int i = 0; i < kVariables; ++i) {
      T movement = T(0);
      for (int j = i; j < kVariables; ++j) {
        movement += baseQR_.r[i][j] * step[j];
      }
      decrease.add(baseQR_.z[i] * movement - T(.5) * movement * movement);
    }
    result.linearDecrease = decrease.value();
    result.scaledStep = maximum;
    result.costResolution =
        T(32) * std::numeric_limits<T>::epsilon() * std::max(baseCost_, T(1e-12));
    return std::isfinite(result.linearDecrease) && std::isfinite(maximum) &&
           result.linearDecrease > T(0) && result.linearDecrease <= result.costResolution &&
           maximum <= T(1e-3);
  }
  void reject() {
    ++result.rejected;
    if (++trials_ > 12 || lambda_ > T(1e8)) {
      finalize(FitExit::Stagnation);
      return;
    }
    lambda_ *= T(10);
    mode_ = Mode::Propose;
  }
  void evaluationFailed() {
    if (mode_ == Mode::Trial) {
      reject();
    } else {
      finalize(FitExit::NumericalFailure);
    }
  }
  void finalize(FitExit exit = FitExit::Converged) {
    result.exit = exit;
    result.cost = baseCost_;
    result.rms = std::sqrt(baseSquare_ / T(kTrainIntervals));
    for (int j = 0; j < kVariables; ++j) {
      result.parameters[j] = base_[j];
    }
    T inverse[kVariables][kVariables]{};
    T diagonalMax = T(0), diagonalMin = std::numeric_limits<T>::infinity();
    for (int j = 0; j < kVariables; ++j) {
      diagonalMax = std::max(diagonalMax, std::abs(baseQR_.r[j][j]));
      diagonalMin = std::min(diagonalMin, std::abs(baseQR_.r[j][j]));
    }
    if (!(diagonalMin > T(1e-6) * diagonalMax) || !(diagonalMax > T(0))) {
      result.exit = FitExit::PoorInformation;
      return;
    }
    for (int column = 0; column < kVariables; ++column) {
      for (int row = kVariables - 1; row >= 0; --row) {
        T b = row == column ? T(1) : T(0);
        for (int j = row + 1; j < kVariables; ++j) {
          b -= baseQR_.r[row][j] * inverse[j][column];
        }
        inverse[row][column] = b / baseQR_.r[row][row];
      }
    }
    T normR = T(0), normInverse = T(0);
    const T variance = baseSquare_ / T(kTrainIntervals - kVariables);
    for (int row = 0; row < kVariables; ++row) {
      T rsum = T(0), isum = T(0), v = T(0);
      for (int j = 0; j < kVariables; ++j) {
        rsum += std::abs(baseQR_.r[row][j]);
        isum += std::abs(inverse[row][j]);
        v += inverse[row][j] * inverse[row][j];
      }
      normR = std::max(normR, rsum);
      normInverse = std::max(normInverse, isum);
      if (row < kShared) {
        result.standardDeviation[row] = scale_[row] * std::sqrt(variance * v);
      }
    }
    result.condition = normR * normInverse;
    const T a = base_[0], b = base_[1], g = std::abs(base_[2]);
    const bool stationarity =
        std::isfinite(result.gradient) &&
        (result.gradient < T(1e-4) ||
         (exit == FitExit::PrecisionLimited && result.linearDecrease > T(0) &&
          result.linearDecrease <= result.costResolution && result.scaledStep <= T(1e-3)));
    result.usable =
        exit != FitExit::NumericalFailure && stationarity && a > Policy<T>::minAlpha * T(1.01) &&
        a < Policy<T>::maxAlpha * T(.99) && b < Policy<T>::maxBeta * T(.99) &&
        g > Policy<T>::minGamma && base_[4] >= T(0) && base_[4] < Policy<T>::maxFriction * T(.99) &&
        result.standardDeviation[4] < T(.05) && g < Policy<T>::maxGamma * T(.99) &&
        std::abs(base_[3]) < T(.08) * a && result.rms < Policy<T>::maxTrainingRms &&
        result.condition < Policy<T>::maxCondition && result.standardDeviation[0] < T(.15) * a &&
        result.standardDeviation[1] < T(.15) && result.standardDeviation[2] < T(.2) * g;
  }
};
} // namespace identification
#endif // ADAPTIVE_FITTER_H
