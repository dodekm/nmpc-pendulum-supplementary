#ifndef ADAPTIVE_CYCLESUPERVISOR_H
#define ADAPTIVE_CYCLESUPERVISOR_H
#pragma once
#include "ControllerValidation.h"

namespace pendulum {
enum class CyclePhase : uint8_t {
  Ascent = 1,
  UprightHold = 2,
  SwingDown = 3,
  DownwardSettling = 4,
  Complete = 5
};
template <class Scalar> struct CycleState {
  explicit CycleState(Scalar period = Scalar(.1)) : samplePeriod(period) {}
  const Scalar samplePeriod;
  uint32_t elapsedSteps = 0, uprightSamples = 0, downSamples = 0, returnStartSteps = 0;
  CyclePhase phase = CyclePhase::Ascent;
  bool returning = false, complete = false, timedOut = false;
  Scalar elapsed() const { return samplePeriod * Scalar(elapsedSteps); }
  void reset() {
    elapsedSteps = uprightSamples = downSamples = returnStartSteps = 0;
    phase = CyclePhase::Ascent;
    returning = complete = timedOut = false;
  }
};

// Dwell evidence counts intervals between valid samples, not samples themselves.
// A sensing gap discards continuity evidence. Integer comparisons avoid mixed
// precision and floating-point rounding at the 30 s / 5 s transitions.
template <class Scalar> class CycleSupervisor final {
public:
  CycleSupervisor(const ControllerConfig<Scalar> &config, Scalar samplePeriod)
      : config_(config), samplePeriod_(samplePeriod),
        valid_(isControllerConfigValid(config, samplePeriod)),
        uprightIntervals_(durationIntervalsOrZero(config.cycle_upright_dwell, samplePeriod)),
        downIntervals_(durationIntervalsOrZero(config.cycle_down_dwell, samplePeriod)),
        returnIntervals_(durationIntervalsOrZero(config.cycle_return_timeout, samplePeriod)) {}

  bool isValid() const { return valid_; }
  CycleSupervisor(ControllerConfig<Scalar> &&, Scalar) = delete;
  CycleSupervisor(const ControllerConfig<Scalar> &&, Scalar) = delete;

  bool advance(CycleState<Scalar> &cycle, const Matrix<Scalar, 3> &state, int elapsedIntervals,
               bool captured, Scalar referenceAngle, Scalar acceleration) const {
    if (!valid_ || cycle.samplePeriod != samplePeriod_ || !state.allFinite() ||
        !std::isfinite(referenceAngle) || !std::isfinite(acceleration) || elapsedIntervals < 1 ||
        elapsedIntervals > 100 || uint32_t(elapsedIntervals) > UINT32_MAX - cycle.elapsedSteps) {
      return false;
    }
    cycle.elapsedSteps += uint32_t(elapsedIntervals);
    if (cycle.complete) {
      return true;
    }
    if (!cycle.returning) {
      cycle.phase = captured ? CyclePhase::UprightHold : CyclePhase::Ascent;
      const Scalar angleError = state(1) - referenceAngle;
      const Scalar wrappedError = std::atan2(std::sin(angleError), std::cos(angleError));
      const bool quiet = captured &&
                         std::abs(wrappedError) <= config_.cycle_upright_angle_tolerance &&
                         std::abs(state(0)) <= config_.cycle_upright_angular_speed_tolerance;
      cycle.uprightSamples = quiet ? (elapsedIntervals == 1 ? cycle.uprightSamples + 1 : 1) : 0;
      if (cycle.uprightSamples > 0 && cycle.uprightSamples - 1 >= uprightIntervals_) {
        cycle.returning = true;
        cycle.phase = CyclePhase::SwingDown;
        cycle.returnStartSteps = cycle.elapsedSteps;
      }
      return true;
    }
    const bool quiet = std::abs(state(1)) <= config_.cycle_down_angle_tolerance &&
                       std::abs(state(0)) <= config_.cycle_down_angular_speed_tolerance &&
                       std::abs(state(2)) <= config_.cycle_down_wheel_speed_tolerance &&
                       std::abs(acceleration) <= config_.cycle_down_acceleration_tolerance;
    cycle.downSamples = quiet ? (elapsedIntervals == 1 ? cycle.downSamples + 1 : 1) : 0;
    cycle.phase = cycle.downSamples > 0 ? CyclePhase::DownwardSettling : CyclePhase::SwingDown;
    if (cycle.downSamples > 0 && cycle.downSamples - 1 >= downIntervals_) {
      cycle.complete = true;
      cycle.phase = CyclePhase::Complete;
    }
    cycle.timedOut =
        !cycle.complete && cycle.elapsedSteps - cycle.returnStartSteps >= returnIntervals_;
    return true;
  }

private:
  const ControllerConfig<Scalar> &config_;
  const Scalar samplePeriod_;
  const bool valid_;
  const uint32_t uprightIntervals_, downIntervals_, returnIntervals_;
};
} // namespace pendulum
#endif // ADAPTIVE_CYCLESUPERVISOR_H
