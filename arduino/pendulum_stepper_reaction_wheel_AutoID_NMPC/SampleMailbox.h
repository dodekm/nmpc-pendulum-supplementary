#ifndef ADAPTIVE_SAMPLEMAILBOX_H
#define ADAPTIVE_SAMPLEMAILBOX_H
#pragma once
#include "InterruptLock.h"
#include <stdint.h>

enum class SampleStatus : uint8_t { Valid, SensorReadError, MagnetError };
struct PendulumSample {
  float angle = 0;
  uint32_t tick = 0;
  SampleStatus status = SampleStatus::SensorReadError;
  uint32_t acquisitionUs = 0;
};

// Latest-value mailbox, not a queue. Value, acquisition tick and quality are
// consumed together; dropped samples have a persistent saturating count.
class SampleMailbox final {
public:
  void publish(float value, uint32_t tick, SampleStatus status, uint32_t acquisitionUs = 0) {
    const InterruptLock lock;
    if (available_ && missed_count_ != UINT32_MAX) {
      ++missed_count_;
    }
    value_ = value;
    tick_ = tick;
    status_ = status;
    acquisition_us_ = acquisitionUs;
    available_ = true;
  }
  bool available() const { return available_; }
  bool consume(PendulumSample &result) {
    const InterruptLock lock;
    if (!available_) {
      return false;
    }
    result.angle = value_;
    result.tick = tick_;
    result.status = status_;
    result.acquisitionUs = acquisition_us_;
    available_ = false;
    return true;
  }
  uint32_t missedCount() const {
    const InterruptLock lock;
    return missed_count_;
  }
  void reset() {
    const InterruptLock lock;
    available_ = false;
    missed_count_ = 0;
    value_ = 0;
    tick_ = 0;
    status_ = SampleStatus::SensorReadError;
    acquisition_us_ = 0;
  }

private:
  volatile float value_ = 0;
  volatile uint32_t tick_ = 0;
  volatile uint32_t acquisition_us_ = 0;
  volatile uint32_t missed_count_ = 0;
  volatile SampleStatus status_ = SampleStatus::SensorReadError;
  volatile bool available_ = false;
};
#endif // ADAPTIVE_SAMPLEMAILBOX_H
