#ifndef ADAPTIVE_COMMANDWATCHDOG_H
#define ADAPTIVE_COMMANDWATCHDOG_H
#pragma once
#include <stdint.h>

// Pure deadline arithmetic, independently host-tested across millis rollover.
class CommandWatchdog final {
public:
  static constexpr uint32_t TimeoutMs = 2000;
  void update(uint32_t now, bool moving) {
    last_ = now;
    active_ = moving;
  }
  bool expired(uint32_t now) const {
    return active_ && static_cast<uint32_t>(now - last_) >= TimeoutMs;
  }
  void stop() { active_ = false; }

private:
  volatile uint32_t last_ = 0;
  volatile bool active_ = false;
};
#endif // ADAPTIVE_COMMANDWATCHDOG_H
