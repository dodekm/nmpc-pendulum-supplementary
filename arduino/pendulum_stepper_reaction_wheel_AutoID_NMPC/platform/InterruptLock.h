#ifndef ADAPTIVE_INTERRUPTLOCK_H
#define ADAPTIVE_INTERRUPTLOCK_H
#pragma once
#include <Arduino.h>
#include <stdint.h>

// STM32/CMSIS critical section. Restore the PREVIOUS mask, including from
// ISR/nested callers; never unconditionally re-enable interrupts.
class InterruptLock final {
public:
  InterruptLock() : previous_mask_(__get_PRIMASK()) { __disable_irq(); }
  ~InterruptLock() { __set_PRIMASK(previous_mask_); }
  InterruptLock(const InterruptLock &) = delete;
  InterruptLock &operator=(const InterruptLock &) = delete;

private:
  const uint32_t previous_mask_;
};
#endif // ADAPTIVE_INTERRUPTLOCK_H
