#ifndef ADAPTIVE_CONSTANTSPEEDPULSESCHEDULER_H
#define ADAPTIVE_CONSTANTSPEEDPULSESCHEDULER_H
#pragma once

#include <stdint.h>
#include "control/Precision.h"

// Constant-speed pulse phase, independent of GPIO and AccelStepper trajectories.
// Periods and phase use unsigned Q16 microseconds. A fractional period avoids
// both integer-us period bias and the reset-to-now polling-rate staircase.
// Callers serialize setPeriod() with poll(), as actuatorWrite() already does.
class ConstantSpeedPulseScheduler final {
public:
  static constexpr uint8_t FRACTION_BITS = 16;
  static constexpr uint64_t TICKS_PER_US = uint64_t(1) << FRACTION_BITS;
  static constexpr uint64_t MAX_PERIOD_TICKS = uint64_t(UINT32_MAX) << FRACTION_BITS;

  void setPeriod(uint64_t period_ticks, bool forward, uint32_t now_us) {
    // A period longer than the wrapping clock's range is not representable.
    if (period_ticks > MAX_PERIOD_TICKS) {
      period_ticks = 0;
    }
    if (period_ticks == 0 || period_ticks_ == 0 || forward != forward_) {
      // Never inherit idle time or an old-direction pulse on restart/reversal.
      period_ticks_ = period_ticks;
      phase_ticks_ = 0;
      pending_step_ = false;
      forward_ = forward;
      last_update_us_ = now_us;
      return;
    }

    // Account for time BEFORE the update at the OLD rate. Keep at most one
    // pending pulse; the setter never touches GPIO or emits a pulse itself.
    advance(now_us);
    if (period_ticks != period_ticks_) {
      // Preserve fractional progress through a pulse when the speed changes.
      // Floating point is used only at command updates, NOT in the timer ISR.
      using Scalar = pendulum::RuntimeScalar;
      const Scalar fraction =
          static_cast<Scalar>(phase_ticks_) / static_cast<Scalar>(period_ticks_);
      phase_ticks_ =
          static_cast<uint64_t>(fraction * static_cast<Scalar>(period_ticks) + Scalar(.5));
      if (phase_ticks_ >= period_ticks) {
        phase_ticks_ = period_ticks - 1;
      }
      period_ticks_ = period_ticks;
    }
  }

  bool poll(uint32_t now_us) {
    advance(now_us);
    const bool due = pending_step_;
    pending_step_ = false;
    return due; // At most one pulse per call; no catch-up loop.
  }

  // Software deadlines discarded after delayed service or excess demand.
  // This is NOT a count of physical motor steps lost and is not shaft feedback.
  uint32_t discardedPulseCount() const { return discarded_pulses_; }

private:
  void advance(uint32_t now_us) {
    // Unsigned subtraction handles micros() rollover, provided active calls
    // are separated by less than one complete 32-bit clock cycle (~71.6 min).
    const uint32_t elapsed_us = now_us - last_update_us_;
    last_update_us_ = now_us;
    if (period_ticks_ == 0) {
      return;
    }

    // phase < period <= MAX_PERIOD_TICKS, so the addition cannot overflow.
    phase_ticks_ += uint64_t(elapsed_us) << FRACTION_BITS;
    if (phase_ticks_ < period_ticks_) {
      return;
    }
    phase_ticks_ -= period_ticks_;
    if (pending_step_ || phase_ticks_ >= period_ticks_) {
      // Service was too late for all deadlines. Emit only one pulse and restart
      // its phase, rather than accelerating to discharge an accumulated queue.
      const uint64_t discarded = phase_ticks_ / period_ticks_ + (pending_step_ ? 1U : 0U);
      const uint32_t remaining = UINT32_MAX - discarded_pulses_;
      discarded_pulses_ += discarded >= remaining ? remaining : static_cast<uint32_t>(discarded);
      phase_ticks_ = 0;
    }
    pending_step_ = true;
  }

  uint64_t period_ticks_ = 0;
  uint64_t phase_ticks_ = 0;
  uint32_t last_update_us_ = 0;
  // Aligned 32-bit volatile snapshot can be read by the STM32 foreground code.
  // Writes occur only in the ISR or while that ISR is excluded by the caller.
  volatile uint32_t discarded_pulses_ = 0;
  bool forward_ = true;
  bool pending_step_ = false;
};
#endif // ADAPTIVE_CONSTANTSPEEDPULSESCHEDULER_H
