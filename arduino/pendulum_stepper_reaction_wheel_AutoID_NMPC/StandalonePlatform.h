#ifndef ADAPTIVE_STANDALONEPLATFORM_H
#define ADAPTIVE_STANDALONEPLATFORM_H
#pragma once
#include <Arduino.h>
#if !defined(ARDUINO_NUCLEO_F446ZE) && !defined(ARDUINO_NUCLEO_F413ZH)
#error "Select the reviewed NUCLEO-F446ZE or NUCLEO-F413ZH platform."
#endif

#undef B0
#undef B1
#undef F
#undef NP
#undef min
#undef max
#include "PendulumShield.h"
#include "InterruptLock.h"

// NUCLEO-F413ZH PC13 is active HIGH (MB1137). Do not infer boot polarity.
namespace standalone_platform {
inline uint32_t coreClockHz() {
  return SystemCoreClock;
}
inline bool buttonPressed() {
  return digitalRead(USER_BTN) == HIGH;
}
inline void configureButton(void (*interruptHandler)()) {
  pinMode(USER_BTN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(USER_BTN), interruptHandler, CHANGE);
  // Preempt the TIM2 step ISR (priority 1).
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
}
} // namespace standalone_platform
#endif // ADAPTIVE_STANDALONEPLATFORM_H
