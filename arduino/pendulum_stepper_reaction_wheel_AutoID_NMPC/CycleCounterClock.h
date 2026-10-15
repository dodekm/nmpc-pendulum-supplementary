#ifndef ADAPTIVE_CYCLECOUNTERCLOCK_H
#define ADAPTIVE_CYCLECOUNTERCLOCK_H
#pragma once
#include <stdint.h>

// Integer-only DWT cycle-counter extension. Callers serialize update().
// While enabled, update at least once per 2^32 CPU cycles (42.95 s at 100 MHz).
// The 30 us step ISR supplies that service even when the foreground is busy.
// Unlike Arduino micros(), this does not depend on a serviced SysTick interrupt.
class CycleCounterClock final {
public:
  bool reset(uint32_t cpuHz, uint32_t cycles) {
    if (cpuHz < 1000000U || cpuHz % 1000000U != 0) {
      return false;
    }
    cyclesPerMicrosecond_ = cpuHz / 1000000U;
    previousCycles_ = cycles;
    fractionalCycles_ = microseconds_ = 0;
    return true;
  }
  uint32_t update(uint32_t cycles) {
    const uint32_t elapsedCycles = cycles - previousCycles_;
    previousCycles_ = cycles;
    microseconds_ += elapsedCycles / cyclesPerMicrosecond_;
    fractionalCycles_ += elapsedCycles % cyclesPerMicrosecond_;
    if (fractionalCycles_ >= cyclesPerMicrosecond_) {
      ++microseconds_;
      fractionalCycles_ -= cyclesPerMicrosecond_;
    }
    return microseconds_;
  }

private:
  uint32_t cyclesPerMicrosecond_ = 1, previousCycles_ = 0;
  uint32_t fractionalCycles_ = 0, microseconds_ = 0;
};
#endif // ADAPTIVE_CYCLECOUNTERCLOCK_H
