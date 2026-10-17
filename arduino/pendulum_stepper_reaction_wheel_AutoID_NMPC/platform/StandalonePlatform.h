#ifndef ADAPTIVE_STANDALONEPLATFORM_H
#define ADAPTIVE_STANDALONEPLATFORM_H
#pragma once
#include <Arduino.h>
#if !defined(ARDUINO_NUCLEO_F446ZE)
#error "Select the NUCLEO-F446ZE platform used by both demonstrated rigs."
#endif

#undef B0
#undef B1
#undef F
#undef NP
#undef min
#undef max
#include "platform/PendulumShield.h"
#include "platform/InterruptLock.h"

// The NUCLEO-F446ZE user button is active HIGH. Do not infer boot polarity.
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
