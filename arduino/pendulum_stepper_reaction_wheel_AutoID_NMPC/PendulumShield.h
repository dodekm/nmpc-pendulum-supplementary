#ifndef ADAPTIVE_PENDULUMSHIELD_H
#define ADAPTIVE_PENDULUMSHIELD_H

#pragma once

#include <Arduino.h>
#include "src/AS5600/AS5600.h"
#include "src/AccelStepper/AccelStepper.h"
#include "ConstantSpeedPulseScheduler.h"
#include "SampleMailbox.h"
#include "CommandWatchdog.h"
#include "control/Precision.h"
#include "HardwareConfig.h"

// AccelStepper changes DIR immediately before raising STEP.  This wrapper
// provides driver-specific DIR setup time, pulse width, and pulse polarity.
class DatasheetTimedStepper final : public AccelStepper {
public:
  DatasheetTimedStepper(uint8_t step_pin, uint8_t dir_pin)
      : AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin), step_pin_(step_pin),
        dir_pin_(dir_pin) {}

  void configureSignals(uint8_t dir_setup_us, uint8_t step_active_us, bool step_active_low,
                        bool direction_inverted);
  void initializeSignalPins();

  // Dedicated timer-polled velocity mode. Do not mix with inherited run(),
  // runSpeed(), moveTo(), or acceleration/position trajectory methods.
  // setConstantSpeed() must be called with the step timer interrupt excluded.
  void setConstantSpeed(float steps_per_second);
  bool runConstantSpeed();
  uint32_t discardedPulseCount() const { return pulse_scheduler_.discardedPulseCount(); }

protected:
  void step1(long step) override;

private:
  const uint8_t step_pin_;
  const uint8_t dir_pin_;
  uint8_t dir_setup_us_ = 1;
  uint8_t step_active_us_ = 3;
  uint8_t step_active_level_ = HIGH;
  uint8_t step_inactive_level_ = LOW;
  bool direction_inverted_ = false;
  ConstantSpeedPulseScheduler pulse_scheduler_;
};

class PendulumClass {

public:
  enum class RuntimeFault : uint8_t {
    None,
    NotInitialized,
    SensorRead,
    Magnet,
    InvalidCommand,
    UnsupportedPulseRate,
    CommandTimeout,
    Communication
  };
  enum class DriverType : uint8_t {
    A4988,
    DRV8825,
    TMC2208,
    TMC2209,
    // Opto-isolated TB6600 driver module.  Its microstep/current DIP switches
    // remain manual; the value passed to begin() must match them.
    TB6600,
    // Opto-isolated HY-DIV268N-5A module.  Microstepping is selected by the
    // module's SW3/SW4 switches, not by the MCU microstep pins.
    HY_DIV268N
  };

  enum class ControlInterface : uint8_t {
    // Native single-ended logic for socketed drivers; common-cathode for the
    // external TB6600 and HY-DIV268N modules.
    DriverDefault,
    // PUL-/DIR-/EN- are common; buffered MCU outputs drive the '+' inputs.
    CommonCathode,
    // PUL+/DIR+/EN+ are common; open-collector outputs drive the '-' inputs.
    CommonAnode
  };

  enum class BeginError : uint8_t {
    None,
    InvalidSamplingPeriod,
    UnsupportedDriverType,
    UnsupportedControlInterface,
    UnsupportedMicrostepMode,
    SensorInitializationFailed,
    MagnetNotDetected,
    ClockInitializationFailed
  };

  PendulumClass() : stepper(STEP_PIN, DIR_PIN), TimerSample(TIM1), TimerStepper(TIM2) {}
  bool begin(float sample_period_s, DriverType driver_type, uint16_t microsteps,
             ControlInterface control_interface = ControlInterface::DriverDefault);
  // Default preserves the copied driver's release-at-zero API. This new
  // application explicitly requests stationary holding for its known-v model.
  bool actuatorWrite(float, bool holdAtZero = false);

  float sensorReadRad(void);
  float sensorReadRadSampled(void);
  bool isSampleAvailable() { return y_sampler.available(); }
  bool readSample(PendulumSample &sample) { return y_sampler.consume(sample); }
  uint32_t getMissedSampleCount() const { return y_sampler.missedCount(); }
  bool initialized() const { return initialized_; }
  RuntimeFault getRuntimeFault() const { return runtime_fault_; }
  void failStop(RuntimeFault fault);
  void enforceCommandWatchdog();
  void calibrate(void);
  float getStepperSpeed() {
    return stepper.speed() / float(STEPS_PER_REV) * pendulum::kTwoPi<float> / float(microsteps_);
  }
  float getStepperPosition() {
    return float(stepper.currentPosition()) / float(STEPS_PER_REV) * pendulum::kTwoPi<float> /
           float(microsteps_);
  }
  uint32_t controlTimeUs() const;
  // Scheduling diagnostic only, NOT detection of mechanically missed steps.
  uint32_t getDiscardedPulseDeadlineCount() const { return stepper.discardedPulseCount(); }
  DriverType getDriverType() const { return driver_type_; }
  ControlInterface getControlInterface() const { return control_interface_; }
  uint16_t getMicrostepMode() const { return microsteps_; }
  bool isMicrostepModeSoftwareControlled() const { return microstep_mode_software_controlled_; }
  BeginError getLastBeginError() const { return last_begin_error_; }

  unsigned long getTick() { return tick; }
  void (*sample_callback)() = NULL;

protected:
  void sampleTick();
  SampleMailbox y_sampler;
  volatile uint32_t tick = 0;
  DatasheetTimedStepper stepper;

private:
  static constexpr float STEPPER_TS = 3e1;
  static constexpr int STEPS_PER_REV = 200;

  static constexpr int STEP_PIN = adaptive_hardware::stepPin;
  static constexpr int DIR_PIN = adaptive_hardware::directionPin;
  static constexpr int EN_PIN = adaptive_hardware::enablePin;

  static constexpr int MS1_PIN = adaptive_hardware::ms1Pin;
  static constexpr int MS2_PIN = adaptive_hardware::ms2Pin;
  static constexpr int MS3_DIAG_PIN = adaptive_hardware::ms3Pin;

  bool configureMicrostepping(DriverType driver_type, uint16_t microsteps);
  void setDriverEnabled(bool enabled);

  DriverType driver_type_ = DriverType::A4988;
  ControlInterface control_interface_ = ControlInterface::DriverDefault;
  BeginError last_begin_error_ = BeginError::None;
  uint16_t microsteps_ = 1;
  uint8_t driver_enable_level_ = adaptive_hardware::commonAnode ? HIGH : LOW;
  uint8_t driver_disable_level_ = adaptive_hardware::commonAnode ? LOW : HIGH;
  uint8_t enable_to_step_delay_us_ = 1;
  bool microstep_mode_software_controlled_ = true;
  bool driver_enabled_ = false;
  volatile bool initialized_ = false;
  volatile RuntimeFault runtime_fault_ = RuntimeFault::NotInitialized;
  CommandWatchdog command_watchdog_;
  int32_t _zero = 0;

  AS5600 as5600;
  HardwareTimer TimerSample;
  HardwareTimer TimerStepper;
};

extern PendulumClass &PendulumShield;
#endif // ADAPTIVE_PENDULUMSHIELD_H
