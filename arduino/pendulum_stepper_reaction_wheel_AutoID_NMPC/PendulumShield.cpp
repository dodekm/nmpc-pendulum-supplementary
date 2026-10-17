#include "platform/PendulumShield.h"

#include "platform/CycleCounterClock.h"

namespace {
CycleCounterClock pulseClock;
uint32_t monotonicMicroseconds() {
  const InterruptLock lock;
  return pulseClock.update(DWT->CYCCNT);
}

struct MicrostepPinConfiguration {
  bool ms1;
  bool ms2;
  bool ms3;
  uint8_t output_mask;
};

constexpr uint8_t MS1_OUTPUT = 1U << 0;
constexpr uint8_t MS2_OUTPUT = 1U << 1;
constexpr uint8_t MS3_OUTPUT = 1U << 2;
constexpr uint8_t ALL_MS_OUTPUTS = MS1_OUTPUT | MS2_OUTPUT | MS3_OUTPUT;

struct DriverSignalConfiguration {
  uint8_t enable_level;
  uint8_t disable_level;
  uint8_t enable_to_step_delay_us;
  uint8_t direction_setup_us;
  uint8_t step_active_us;
  bool step_active_low;
  bool direction_inverted;
};

bool getDriverSignalConfiguration(PendulumClass::DriverType driver_type,
                                  PendulumClass::ControlInterface requested_interface,
                                  DriverSignalConfiguration &configuration) {
  using ControlInterface = PendulumClass::ControlInterface;
  using DriverType = PendulumClass::DriverType;

  if (driver_type == DriverType::HY_DIV268N) {
    ControlInterface resolved = requested_interface;
    if (resolved == ControlInterface::DriverDefault) {
      resolved = ControlInterface::CommonCathode;
    }
    if (resolved == ControlInterface::CommonCathode) {
      // The EN optocoupler is an "offline" input: asserted means disabled.
      // With the '-' terminals common, the optocouplers are active HIGH.
      // The module manuals do not specify complete input timing limits; 5 us
      // pulse and DIR setup times provide a conservative interface margin.
      configuration = {LOW, HIGH, 5, 5, 5, false, false};
      return true;
    }
    if (resolved == ControlInterface::CommonAnode) {
      // With the '+' terminals common, an open-collector LOW asserts an
      // optocoupler input, so all three control signals are inverted.
      configuration = {HIGH, LOW, 5, 5, 5, true, true};
      return true;
    }
    return false;
  }

  if (requested_interface != ControlInterface::DriverDefault) {
    return false;
  }

  switch (driver_type) {
  case DriverType::A4988:
    configuration = {LOW, HIGH, 1, 1, 3, false, false};
    return true;

  case DriverType::HY_DIV268N:
    break;
  }

  return false;
}

bool getMicrostepPinConfiguration(PendulumClass::DriverType driver_type, uint16_t microsteps,
                                  MicrostepPinConfiguration &configuration) {
  using DriverType = PendulumClass::DriverType;

  switch (driver_type) {
  case DriverType::A4988:
    // Allegro A4988: MS1, MS2, MS3.
    switch (microsteps) {
    case 1:
      configuration = {false, false, false, ALL_MS_OUTPUTS};
      return true;
    case 2:
      configuration = {true, false, false, ALL_MS_OUTPUTS};
      return true;
    case 4:
      configuration = {false, true, false, ALL_MS_OUTPUTS};
      return true;
    case 8:
      configuration = {true, true, false, ALL_MS_OUTPUTS};
      return true;
    case 16:
      configuration = {true, true, true, ALL_MS_OUTPUTS};
      return true;
    default:
      return false;
    }

  case DriverType::HY_DIV268N:
    // The value is used for kinematic scaling only.  The physical setting
    // must match SW3/SW4 on the module; no MCU pin selects it.
    switch (microsteps) {
    case 1:
    case 2:
    case 8:
    case 16:
      configuration = {false, false, false, 0};
      return true;
    default:
      return false;
    }
  }

  return false;
}

} // namespace

void DatasheetTimedStepper::setConstantSpeed(float steps_per_second) {
  // Match actuatorWrite's existing stopped-speed convention. Base setSpeed
  // retains speed()/direction/max-speed bookkeeping, but its integer period
  // is NOT used by the dedicated pulse scheduler.
  if (!isfinite(steps_per_second) || fabsf(steps_per_second) < 1.0f) {
    steps_per_second = 0.0f;
  }
  AccelStepper::setSpeed(steps_per_second);
  const float commanded_speed = AccelStepper::speed();
  uint64_t period_ticks = 0;
  if (commanded_speed != 0.0f) {
    using Scalar = pendulum::RuntimeScalar;
    const Scalar period = Scalar(1000000) * Scalar(ConstantSpeedPulseScheduler::TICKS_PER_US) /
                          std::abs(Scalar(commanded_speed));
    period_ticks = static_cast<uint64_t>(period + Scalar(.5));
  }
  pulse_scheduler_.setPeriod(period_ticks, _direction == DIRECTION_CW, monotonicMicroseconds());
}

bool DatasheetTimedStepper::runConstantSpeed() {
  if (!pulse_scheduler_.poll(monotonicMicroseconds())) {
    return false;
  }
  // These inherited methods update the emitted-pulse position and invoke the
  // UNCHANGED virtual step1(): electrical DIR setup/pulse widths are preserved.
  if (_direction == DIRECTION_CW) {
    stepForward();
  } else {
    stepBackward();
  }
  return true;
}

void DatasheetTimedStepper::step1(long step) {
  (void)step;

  const bool direction = _direction != direction_inverted_;
  digitalWrite(dir_pin_, direction ? HIGH : LOW);
  delayMicroseconds(dir_setup_us_);
  digitalWrite(step_pin_, step_active_level_);
  delayMicroseconds(step_active_us_);
  digitalWrite(step_pin_, step_inactive_level_);
}

void DatasheetTimedStepper::configureSignals(uint8_t dir_setup_us, uint8_t step_active_us,
                                             bool step_active_low, bool direction_inverted) {
  dir_setup_us_ = dir_setup_us;
  step_active_us_ = step_active_us;
  step_active_level_ = step_active_low ? LOW : HIGH;
  step_inactive_level_ = step_active_low ? HIGH : LOW;
  direction_inverted_ = direction_inverted;
}

void DatasheetTimedStepper::initializeSignalPins() {
  digitalWrite(step_pin_, step_inactive_level_);
  digitalWrite(dir_pin_, direction_inverted_ ? HIGH : LOW);
}

bool PendulumClass::begin(float sample_period_s, DriverType driver_type, uint16_t microsteps,
                          ControlInterface control_interface) {
  pendulum::verifyBuildPrecision(static_cast<pendulum::RuntimeScalar *>(nullptr));
  if (initialized_) {
    TimerSample.pause();
    TimerStepper.pause();
    failStop(RuntimeFault::NotInitialized);
  }
  initialized_ = false;

  using Scalar = pendulum::RuntimeScalar;
  const Scalar samplePeriodUs = Scalar(sample_period_s) * Scalar(1000000);
  // Strict comparison with 2^32 prevents an out-of-range float->uint32 cast:
  // UINT32_MAX itself rounds up to 2^32 in binary32.
  if (!std::isfinite(samplePeriodUs) || samplePeriodUs < Scalar(1) ||
      samplePeriodUs >= Scalar(4294967296ULL)) {
    return false;
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __DSB();
  __ISB();
  const uint32_t before = DWT->CYCCNT;
  for (unsigned i = 0; i < 32; ++i) {
    __NOP();
  }
  if (DWT->CYCCNT == before || !pulseClock.reset(SystemCoreClock, DWT->CYCCNT)) {
    return false;
  }

  DriverSignalConfiguration signal_configuration{};
  if (!getDriverSignalConfiguration(driver_type, control_interface, signal_configuration)) {
    return false;
  }

  driver_enable_level_ = signal_configuration.enable_level;
  driver_disable_level_ = signal_configuration.disable_level;
  enable_to_step_delay_us_ = signal_configuration.enable_to_step_delay_us;
  stepper.configureSignals(
      signal_configuration.direction_setup_us, signal_configuration.step_active_us,
      signal_configuration.step_active_low, signal_configuration.direction_inverted);

  // Put the power stage in its driver-specific disabled state before changing
  // any mode or pulse pins.  Writing first also avoids an enable glitch when
  // pinMode changes the GPIO from its reset input state to an output.
  digitalWrite(EN_PIN, driver_disable_level_);
  pinMode(EN_PIN, OUTPUT);
  driver_enabled_ = false;
  stepper.setConstantSpeed(0.0f);

  if (!configureMicrostepping(driver_type, microsteps)) {
    return false;
  }

  stepper.initializeSignalPins();

  Wire.begin();
  Wire.setClock(400000);
  if (!as5600.begin()) {
    return false;
  }
  if (!as5600.detectMagnet()) {
    return false;
  }
  as5600.setOffset(90);
  calibrate();
  if (runtime_fault_ == RuntimeFault::SensorRead) {
    return false;
  }
  delay(200);

  // At most one pulse per poll: do not advertise an unreachable rate.
  stepper.setMaxSpeed(1000000.0f / STEPPER_TS);
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  tick = 0;
  y_sampler.reset();
  runtime_fault_ = RuntimeFault::None;
  initialized_ = true;

  TimerStepper.pause();
  TimerStepper.setOverflow(STEPPER_TS, MICROSEC_FORMAT);
  TimerStepper.attachInterrupt(stepperCallback);
  TimerStepper.refresh();
  TimerStepper.resume();

  TimerSample.pause();
  TimerSample.setOverflow(static_cast<uint32_t>(samplePeriodUs), MICROSEC_FORMAT);
  TimerSample.attachInterrupt(samplerCallback);
  TimerSample.refresh();
  TimerSample.resume();

  actuatorWrite(0);

  return true;
}

bool PendulumClass::configureMicrostepping(DriverType driver_type, uint16_t microsteps) {
  MicrostepPinConfiguration configuration{};
  if (!getMicrostepPinConfiguration(driver_type, microsteps, configuration)) {
    return false;
  }

  const auto configure_pin = [](uint8_t pin, uint8_t mask, bool level, uint8_t output_mask) {
    if ((output_mask & mask) != 0U) {
      digitalWrite(pin, level ? HIGH : LOW);
      pinMode(pin, OUTPUT);
    } else {
      // The external module uses manual DIP switches, not MCU mode outputs.
      pinMode(pin, INPUT);
      digitalWrite(pin, LOW); // Disable the Arduino input pull-up.
    }
  };

  configure_pin(MS1_PIN, MS1_OUTPUT, configuration.ms1, configuration.output_mask);
  configure_pin(MS2_PIN, MS2_OUTPUT, configuration.ms2, configuration.output_mask);
  configure_pin(MS3_DIAG_PIN, MS3_OUTPUT, configuration.ms3, configuration.output_mask);

  delayMicroseconds(10);
  microsteps_ = microsteps;
  return true;
}

void PendulumClass::setDriverEnabled(bool enabled) {
  digitalWrite(EN_PIN, enabled ? driver_enable_level_ : driver_disable_level_);
  driver_enabled_ = enabled;
}

void PendulumClass::calibrate(void) {
  _zero = as5600.getCumulativePosition();
  if (as5600.lastError() != AS5600_OK) {
    failStop(RuntimeFault::SensorRead);
  }
}

bool PendulumClass::actuatorWrite(float rad_s) {
  if (!isfinite(rad_s)) {
    failStop(RuntimeFault::InvalidCommand);
    return false;
  }
  float steps_s = 0.0f;
  const float rotations_s = rad_s / pendulum::kTwoPi<float>;
  steps_s = rotations_s * static_cast<float>(STEPS_PER_REV) * static_cast<float>(microsteps_);
  if (!isfinite(steps_s) || fabsf(steps_s) > 1000000.0f / STEPPER_TS) {
    failStop(RuntimeFault::UnsupportedPulseRate);
    return false;
  }

  const bool should_move = isfinite(steps_s) && fabsf(steps_s) >= 1.0f;

  // TimerStepper calls runConstantSpeed() from an interrupt. Update speed and enable
  // atomically so the ISR cannot emit a pulse with a half-updated command.
  const InterruptLock lock;
  if (should_move) {
    if (!initialized_ || runtime_fault_ != RuntimeFault::None) {
      return false;
    }
    stepper.setConstantSpeed(steps_s);
    if (!driver_enabled_) {
      setDriverEnabled(true);
      delayMicroseconds(enable_to_step_delay_us_);
    }
  } else {
    stepper.setConstantSpeed(0.0f);
    setDriverEnabled(false);
    // An explicit zero permits recovery from communication/command faults,
    // not a lost encoder origin or a latched sensor/magnet fault.
    if (initialized_ && runtime_fault_ != RuntimeFault::SensorRead &&
        runtime_fault_ != RuntimeFault::Magnet) {
      runtime_fault_ = RuntimeFault::None;
    }
  }
  command_watchdog_.update(static_cast<uint32_t>(millis()), should_move);
  return true;
}

void PendulumClass::failStop(RuntimeFault fault) {
  const InterruptLock lock;
  if (runtime_fault_ != RuntimeFault::SensorRead && runtime_fault_ != RuntimeFault::Magnet) {
    runtime_fault_ = fault;
  }
  stepper.setConstantSpeed(0);
  command_watchdog_.stop();
  setDriverEnabled(false);
}

void PendulumClass::enforceCommandWatchdog() {
  const InterruptLock lock;
  if (command_watchdog_.expired(static_cast<uint32_t>(millis()))) {
    failStop(RuntimeFault::CommandTimeout);
  }
}

float PendulumClass::sensorReadRad() {
  int32_t angle = as5600.getCumulativePosition();
  if (as5600.lastError() != AS5600_OK) {
    failStop(RuntimeFault::SensorRead);
    return NAN;
  }
  const bool magnet = as5600.detectMagnet();
  if (as5600.lastError() != AS5600_OK || !magnet) {
    failStop(RuntimeFault::Magnet);
    return NAN;
  }
  // Promote before subtraction: avoid signed int32 overflow after many turns.
  float rad = -static_cast<float>(static_cast<int64_t>(angle) - _zero) * AS5600_RAW_TO_RADIANS;
  return rad;
}

void PendulumClass::sampleTick() {
  const float angle = sensorReadRad();
  const uint32_t acquisition_tick = tick + 1U;
  SampleStatus status = SampleStatus::Valid;
  if (!isfinite(angle)) {
    status = runtime_fault_ == RuntimeFault::Magnet ? SampleStatus::MagnetError
                                                    : SampleStatus::SensorReadError;
  }
  y_sampler.publish(angle, acquisition_tick, status, controlTimeUs());
  tick = acquisition_tick;
}

uint32_t PendulumClass::controlTimeUs() const {
  return monotonicMicroseconds();
}

namespace {
PendulumClass pendulumShield;
} // namespace

PendulumClass &PendulumShield = pendulumShield;

void PendulumClass::samplerCallback() {
  PendulumShield.sampleTick();
}

void PendulumClass::stepperCallback() {
  PendulumShield.enforceCommandWatchdog();
  PendulumShield.stepper.runConstantSpeed();
}
