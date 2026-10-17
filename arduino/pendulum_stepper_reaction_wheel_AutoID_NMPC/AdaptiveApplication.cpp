#include <platform/StandalonePlatform.h>
#include "AdaptiveApplication.h"
#include "application/Application.h"
#include "platform/HardwareConfig.h"
#include "telemetry/ApplicationDiagnostics.h"
#include "identification/Validation.h"
#include "identification/GuidedExcitation.h"
#include "identification/RecordKind.h"
#include "control/TerminalDesign.h"
#include "control/CycleController.h"
#include "control/RunningStatistics.h"
#include "control/RestDetector.h"
#include "control/ModelTuning.h"
#include <optional>

namespace adaptive_application {
namespace detail {
namespace {
volatile bool emergency = false;
volatile uint32_t pressEpoch = 0;
void buttonInterrupt() {
  if (standalone_platform::buttonPressed()) {
    ++pressEpoch;
    emergency = true;
    PendulumShield.failStop(PendulumClass::RuntimeFault::Communication);
  }
}
} // namespace

inline void Application::initialize() {
  if (initialized_) {
    return;
  }
  initialized_ = true;
  pendulum::verifyBuildPrecision(static_cast<T *>(nullptr));
  adaptive_hardware::verifyBuildConfiguration(
      static_cast<adaptive_hardware::BuildConfiguration *>(nullptr));
  // Select physical-rig tuning before authority checks, terminal design and
  // numerical preflight. Generic MPC defaults remain hardware-independent.
  config_.u_limit_swing = T(adaptive_hardware::controlAccelerationScale);
  config_.u_limit_stabilize = T(adaptive_hardware::controlAccelerationScale);
  digitalWrite(adaptive_hardware::enablePin, adaptive_hardware::disabledLevel);
  pinMode(adaptive_hardware::enablePin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  standalone_platform::configureButton(buttonInterrupt);
  Serial.begin(250000);
#if ADAPTIVE_DRIVER_PROFILE == 0
  const auto driver = PendulumClass::DriverType::A4988;
  const auto interface = PendulumClass::ControlInterface::DriverDefault;
#else
  const auto driver = PendulumClass::DriverType::HY_DIV268N;
  const auto interface = adaptive_hardware::commonAnode
                             ? PendulumClass::ControlInterface::CommonAnode
                             : PendulumClass::ControlInterface::CommonCathode;
#endif
  lastSampleMs_ = phaseStartMs_ = millis();
  if (!PendulumShield.begin(float(Limits::period), driver, adaptive_hardware::microsteps,
                            interface)) {
    enterStoppedState("initialization_failure", Phase::HardwareFault);
    return;
  }
  if (emergency || standalone_platform::buttonPressed()) {
    enterStoppedState("button_held_at_boot", Phase::EmergencyStop);
    return;
  }
  if (!publishSpeedCommand(T(0))) {
    return;
  }
  emitLifecycleEvent("boot_auto_identification_downward_stopped_required");
}

inline Status Application::status() const {
  return {phase_,      committed_,   attempts_, successfulCycles_, model_.alpha,
          model_.beta, model_.gamma, best_.rms, validation_.rms};
}

inline void Application::poll() {
  drainDiagnostics();
  const uint32_t now = millis();
  bool led = isActuationPhase();
  if (phase_ == Phase::EmergencyStop) {
    led = now % 400 < 200;
  } else if (phase_ == Phase::HardwareFault) {
    led = now % 1000 < 100 || (now % 1000 >= 250 && now % 1000 < 350);
  } else if (phase_ == Phase::IdentificationRejected || phase_ == Phase::ManualReturn) {
    led = now % 1000 < 150;
  } else if (!isActuationPhase()) {
    led = now % 1000 < 50;
  }
  digitalWrite(LED_BUILTIN, led ? HIGH : LOW);
  if (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'x' || c == 'X') {
      emergency = true;
    } else {
      diagnostics_.handleInput(c, phase_, attempts_, committed_);
    }
  }
  observeRestartButton(now);
  const bool manual = requiresManualAcknowledgement(phase_);
  if ((emergency || standalone_platform::buttonPressed()) && !manual &&
      phase_ != Phase::HardwareFault) {
    enterStoppedState("emergency_stop", Phase::EmergencyStop);
    return;
  }
  if (phase_ == Phase::HardwareFault) {
    return;
  }
  const auto fault = PendulumShield.getRuntimeFault();
  if (fault == PendulumClass::RuntimeFault::SensorRead ||
      fault == PendulumClass::RuntimeFault::Magnet) {
    enterStoppedState("sensor_origin_uncertain_reset_required", Phase::HardwareFault);
    return;
  }
  if (!manual && fault != PendulumClass::RuntimeFault::None) {
    enterStoppedState("driver_fault", Phase::ManualReturn);
    return;
  }
  if (uint32_t(now - lastSampleMs_) > 500) {
    enterStoppedState("sample_timeout_reset_required", Phase::HardwareFault);
    return;
  }
  if (!processPendingSample(now, manual)) {
    return;
  }
  if (phase_ == Phase::Fitting) {
    pollIdentificationFit(now);
  }
  if (phase_ == Phase::ValidationProcessing) {
    pollValidation(now);
  }
  if (phase_ == Phase::DesignUpright || phase_ == Phase::DesignDownward) {
    pollTerminalDesign(now);
  }
  if (phase_ == Phase::ModelPreflight) {
    pollModelPreflight(now);
  }
  // Re-read BOTH lifecycle and clock after potentially state-changing work.
  // stop() can stamp a newer millis() value; subtracting the entry timestamp
  // would wrap and overwrite the actual rejection with a false timeout.
  const bool stopped = isStopped(phase_);
  const uint32_t afterWork = millis();
  if (!isActuationPhase() && !stopped &&
      uint32_t(afterWork - phaseStartMs_) > startupPhaseTimeoutMs()) {
    enterStoppedState(phase_ == Phase::ValidationRest || phase_ == Phase::ResonanceRest
                          ? "identification_settling_timeout"
                          : "startup_stage_timeout",
                      Phase::IdentificationRejected);
  }
  if (now - lastStatusMs_ >= 1000) {
    lastStatusMs_ = now;
    reportLifecycle();
  }
}

inline bool Application::controlWorkBudgetExpired(const void *context) {
  const auto &clock = *static_cast<const ControlBudgetContext *>(context);
  return uint32_t(PendulumShield.controlTimeUs() - clock.sampleReadUs) >=
         pendulum::ControlTuning<T>::controlWorkBudgetUs;
}

inline bool Application::processPendingSample(uint32_t now, bool manual) {
  PendulumSample sample;
  if (PendulumShield.readSample(sample)) {
    const uint32_t steps = sample.tick - lastTick_;
    if (sample.status != SampleStatus::Valid || !std::isfinite(sample.angle) || steps != 1) {
      enterStoppedState("sample_epoch_or_quality_failure", Phase::HardwareFault);
      return false;
    }
    lastTick_ = sample.tick;
    acquisitionUs_ = sample.acquisitionUs;
    lastSampleMs_ = now;
    latest_ = T(sample.angle);
    const T angle = latest_ - origin_;
    measuredOmega_ = (T(3) * angle - T(4) * previousAngle_ + olderAngle_) / (T(2) * Limits::period);
    olderAngle_ = previousAngle_;
    previousAngle_ = angle;
    if (originFrozen_ && std::abs(angle) > T(7) * pendulum::kPi<T> / T(6) && !manual) {
      enterStoppedState("rollover_manual_unwind", Phase::ManualReturn);
      return false;
    }
    rest_.observeRest(angle, 1, now);
    if (!originFrozen_) {
      if (havePrevious_ && std::abs(latest_ - previous_) > T(.005)) {
        zero_ = pendulum::RunningStatistics<T>();
      }
      if (!zero_.add(latest_)) {
        enterStoppedState("invalid_origin_statistics", Phase::HardwareFault);
        return false;
      }
      previous_ = latest_;
      havePrevious_ = true;
    }
    if (manual) {
      if (restartReady_ && rest_.restReady(now) && !standalone_platform::buttonPressed()) {
        restartAfterManualAcknowledgement(now);
      }
    } else if (isAcquiring(phase_)) {
      processAcquisitionSample(angle, now);
    } else if (phase_ == Phase::BrakeToRetry) {
      if (!publishSpeedCommand(identification::limitedVelocity(T(0), speed_))) {
        return false;
      }
      if (speed_ == T(0)) {
        transitionTo(Phase::StartupRest, now);
      }
    } else if (phase_ == Phase::Control || phase_ == Phase::SettlingControl) {
      processControlSample(angle, now);
    } else {
      advanceRestPhase(now);
    }
  }
  return true;
}

inline bool Application::isActuationPhase() const {
  return phaseMayActuate(phase_);
}

inline uint32_t Application::startupPhaseTimeoutMs() const {
  if (phase_ != Phase::ResonanceRest && phase_ != Phase::ValidationRest) {
    return 180000;
  }
  const auto &fit = phase_ == Phase::ResonanceRest ? coarse_ : best_;
  if (!fit.usable) {
    return 180000;
  }
  const T restRadius =
      std::min(pendulum::RestPolicy<T>::angle(),
               pendulum::RestPolicy<T>::angularSpeed() / std::sqrt(fit.parameters[0]));
  const T ratio = std::max(T(1), Limits::maxAngle / restRadius);
  const T seconds = T(10) + T(4) * std::log(ratio) / std::max(fit.parameters[1], T(.01));
  return uint32_t(pendulum::clamp(seconds, T(180), T(600)) * T(1000));
}

inline void Application::transitionTo(Phase p, uint32_t now) {
  phase_ = p;
  phaseStartMs_ = now;
  rest_.discardRest();
}

inline bool Application::publishSpeedCommand(T velocity) {
  const uint32_t epoch = pressEpoch;
  bool ok = false;
  T acknowledged = T(0);
  uint32_t acknowledgedUs = 0;
  {
    InterruptLock lock;
    if (!emergency && !standalone_platform::buttonPressed() && epoch == pressEpoch &&
        PendulumShield.getRuntimeFault() == PendulumClass::RuntimeFault::None &&
        std::isfinite(velocity) && (velocity == T(0) || PendulumShield.getTick() == lastTick_)) {
      ok = PendulumShield.actuatorWrite(float(velocity));
      if (ok) {
        acknowledged = T(PendulumShield.getStepperSpeed());
        ok = std::isfinite(acknowledged);
        acknowledgedUs = PendulumShield.controlTimeUs();
      }
    }
  }
  if (!ok || emergency || standalone_platform::buttonPressed() || epoch != pressEpoch) {
    enterStoppedState("command_or_emergency_failure", Phase::EmergencyStop);
    return false;
  }
  speed_ = acknowledged;
  lastAcknowledgedUs_ = acknowledgedUs;
  return true;
}

inline void Application::beginAcquisitionRecord(identification::RecordKind kind) {
  acquisitionData_.intervals = 0;
  acquisitionData_.angle[0] = latest_ - origin_;
  speed_ = T(0);
  diagnostics_.beginIdentification(++acquisitionGeneration_, attempts_, uint32_t(kind),
                                   identification::kTrainIntervals, acquisitionUs_,
                                   acquisitionData_.angle[0]);
}

inline void Application::startCoarseTraining(uint32_t now) {
  guidedPass_ = false;
  guidedPeak_ = T(0);
  pulseCount_ = PendulumShield.getDiscardedPulseDeadlineCount();
  ++attempts_;
  beginAcquisitionRecord(identification::RecordKind::Coarse);
  transitionTo(Phase::Training, now);
  emitLifecycleEvent("training_chirp_start");
  publishExcitationCommand(false);
}

inline void Application::startValidationAcquisition(uint32_t now) {
  if (!validationGuide_.begin(best_.parameters[0], best_.parameters[1], best_.parameters[2],
                              amplitude_, true, best_.parameters[4])) {
    enterStoppedState("validation_design_invalid", Phase::IdentificationRejected);
    return;
  }
  beginAcquisitionRecord(identification::RecordKind::Validation);
  transitionTo(Phase::Validation, now);
  diagnostics_.rememberDesign(validationGuide_, best_, 1, attempts_, acquisitionGeneration_);
  emitLifecycleEvent("independent_validation_chirp_start");
  publishExcitationCommand(true);
}

inline void Application::startGuidedTraining(uint32_t now) {
  if (!guide_.begin(coarse_.parameters[0], coarse_.parameters[1], coarse_.parameters[2], amplitude_,
                    false, coarse_.parameters[4])) {
    enterStoppedState("resonance_design_invalid", Phase::IdentificationRejected);
    return;
  }
  beginAcquisitionRecord(identification::RecordKind::Guided);
  guidedPass_ = true;
  guidedPeak_ = T(0);
  transitionTo(Phase::ResonanceTraining, now);
  diagnostics_.rememberDesign(guide_, coarse_, 2, attempts_, acquisitionGeneration_);
  emitLifecycleEvent("resonance_training_start");
  publishExcitationCommand(false);
}

inline void Application::publishExcitationCommand(bool validation) {
  const T requested =
      validation    ? validationGuide_.command(acquisitionData_.intervals, latest_ - origin_)
      : guidedPass_ ? guide_.command(acquisitionData_.intervals, latest_ - origin_)
                    : identification::coarseChirp<T>(acquisitionData_.intervals, amplitude_);
  if (!std::isfinite(requested)) {
    enterStoppedState("invalid_excitation_request", Phase::IdentificationRejected);
    return;
  }
  const T command = identification::limitedVelocity(requested, speed_);
  if (!publishSpeedCommand(command)) {
    return;
  }
  acquisitionData_.command[acquisitionData_.intervals] = speed_;
}

inline void Application::processAcquisitionSample(T angle, uint32_t now) {
  const bool validating = phase_ == Phase::Validation;
  if (PendulumShield.getDiscardedPulseDeadlineCount() != pulseCount_) {
    enterStoppedState("pulse_service_loss_manual_wheel_check", Phase::ManualReturn);
    return;
  }
  if (std::abs(angle) > Limits::maxAngle) {
    abortIdentification("excitation_angle_guard", now, true);
    return;
  }
  ++acquisitionData_.intervals;
  if (guidedPass_ && !validating) {
    guidedPeak_ = std::max(guidedPeak_, std::abs(angle));
  }
  acquisitionData_.angle[acquisitionData_.intervals] = angle;
  diagnostics_.identificationSample(uint32_t(acquisitionData_.intervals), acquisitionUs_, angle,
                                    speed_, lastAcknowledgedUs_);
  const int end =
      validating ? identification::kValidationIntervals : identification::kTrainIntervals;
  if (acquisitionData_.intervals == end) {
    diagnostics_.endIdentification();
    // Both chirps taper to zero; normal finishing is not an abrupt high-RPM stop.
    if (!publishSpeedCommand(T(0))) {
      return;
    }
    if (validating) {
      if (!validator_.begin(acquisitionData_, best_)) {
        abortIdentification("invalid_validation_record_or_model", now, false);
        return;
      }
      transitionTo(Phase::ValidationProcessing, now);
      emitLifecycleEvent("validation_prediction_assessment_motor_stopped");
    } else {
      seed_ = 0;
      best_ = identification::FitResult<T>();
      if (!fitter_.begin(acquisitionData_)) {
        abortIdentification("insufficient_excitation_or_seed", now, false);
        return;
      }
      transitionTo(Phase::Fitting, now);
      emitLifecycleEvent("training_complete_motor_stopped_fit_start");
    }
  } else {
    publishExcitationCommand(validating);
  }
}

inline void Application::advanceRestPhase(uint32_t now) {
  // Fitting needs no current pendulum rest. Independent validation uses a
  // reserved prefix to estimate its initial velocity, not plant parameters.
  if (phase_ == Phase::ValidationRest) {
    if (rest_.restReady(now) && speed_ == T(0)) {
      startValidationAcquisition(now);
    }
    return;
  }
  if (phase_ == Phase::ResonanceRest) {
    if (rest_.restReady(now) && speed_ == T(0)) {
      startGuidedTraining(now);
    }
    return;
  }
  if (phase_ != Phase::DesignRest && !rest_.restReady(now)) {
    return;
  }
  if (phase_ == Phase::StartupRest) {
    if (zero_.count() < 21 || std::sqrt(zero_.sampleVariance()) > T(.005)) {
      return;
    }
    if (!originFrozen_) {
      origin_ = zero_.mean();
    }
    originFrozen_ = true;
    pulseCount_ = PendulumShield.getDiscardedPulseDeadlineCount();
    startCoarseTraining(now);
  } else if (phase_ == Phase::DesignRest) {
    model_.alpha = best_.parameters[0];
    model_.beta = best_.parameters[1];
    model_.gamma = best_.parameters[2];
    model_.coulomb = best_.parameters[4];
    model_.samplePeriod = Limits::period;
    // A finite fit alone is not enough to hold a capture-sized lean within
    // the selected acceleration envelope. This gate is conservative/local.
    if (!model_.isValid() ||
        std::abs(model_.gamma) * config_.u_limit_stabilize <
            T(1.1) * model_.alpha * std::sin(config_.cycle_upright_angle_tolerance)) {
      abortIdentification("insufficient_capture_authority", now, false);
      return;
    }
    // Instantaneous separatrix damping is NOT a necessary feasibility
    // condition. Report it; the full-cycle motor-stopped preflight decides.
    if (T(1.1) * model_.beta * T(2) * std::sqrt(model_.alpha) >
        std::abs(model_.gamma) * config_.u_limit_swing) {
      emitLifecycleEvent("high_viscous_load_warning_use_nmpc_preflight");
    }
    if (!pendulum::configureIdentifiedModel(model_, config_)) {
      abortIdentification("model_tuning_invalid", now, false);
      return;
    }
    if (!design_.begin(model_, config_.hold, config_.R_stabilize, true)) {
      abortIdentification("terminal_design_failed", now, false);
      return;
    }
    transitionTo(Phase::DesignUpright, now);
  } else if (phase_ == Phase::Cooldown) {
    startControlCycle(now);
  }
}

inline void Application::pollIdentificationFit(uint32_t now) {
  const uint32_t started = PendulumShield.controlTimeUs();
  fitter_.poll();
  computeUs_ = PendulumShield.controlTimeUs() - started;
  peakComputeUs_ = std::max(peakComputeUs_, computeUs_);
  if (fitter_.running()) {
    return;
  }
  diagnostics_.reportFit(fitter_.result, unsigned(seed_), guidedPass_, attempts_,
                         acquisitionGeneration_);
  if (fitter_.result.usable && fitter_.result.cost < best_.cost) {
    best_ = fitter_.result;
  }
  if (++seed_ < 3) {
    const T factor = seed_ == 1 ? T(.7) : T(1.4);
    if (!fitter_.begin(acquisitionData_, factor)) {
      abortIdentification("seed_failure", now, false);
      return;
    }
  } else if (!best_.usable) {
    abortIdentification("fit_information_or_quality_rejected", now, false);
  } else {
    if (!guidedPass_) {
      coarse_ = best_;
      transitionTo(Phase::ResonanceRest, now);
      emitLifecycleEvent("coarse_model_ready_for_resonance_design");
    } else {
      if (guidedPeak_ < T(.1745329252)) {
        abortIdentification("guided_excitation_too_weak", now, false);
        return;
      }
      diagnostics_.requestRefinementReport(coarse_, best_, guidedPeak_, attempts_,
                                           acquisitionGeneration_);
      transitionTo(Phase::ValidationRest, now);
      emitLifecycleEvent("refined_model_selected_without_validation_data");
    }
    reportLifecycle();
  }
}

inline void Application::pollValidation(uint32_t now) {
  const uint32_t started = PendulumShield.controlTimeUs();
  validator_.poll();
  computeUs_ = PendulumShield.controlTimeUs() - started;
  peakComputeUs_ = std::max(peakComputeUs_, computeUs_);
  if (validator_.running()) {
    return;
  }
  validation_ = validator_.result;
  diagnostics_.requestValidationReport(validation_, attempts_, acquisitionGeneration_);
  if (!validation_.accepted) {
    abortIdentification("independent_model_validation_rejected", now, false);
    return;
  }
  transitionTo(Phase::DesignRest, now);
  emitLifecycleEvent("independent_model_validation_passed");
}

inline void Application::pollTerminalDesign(uint32_t now) {
  design_.poll();
  if (!design_.complete) {
    return;
  }
  if (!design_.valid) {
    abortIdentification("nonstabilizing_terminal_design", now, false);
    return;
  }
  if (phase_ == Phase::DesignUpright) {
    uprightTail_ = design_.tail;
    if (!design_.begin(model_, config_.down, config_.R_return, false)) {
      abortIdentification("downward_design_failed", now, false);
      return;
    }
    transitionTo(Phase::DesignDownward, now);
  } else {
    downwardTail_ = design_.tail;
    controller_.emplace(model_, config_, uprightTail_, downwardTail_);
    diagnostics_.requestControlConfiguration();
    if (!controller_->resetAtRest(T(0), best_.parameters[3])) {
      enterStoppedState("preflight_initialization_failed", Phase::IdentificationRejected);
      return;
    }
    virtualState_.setZero();
    virtualSamples_ = 0;
    transitionTo(Phase::ModelPreflight, now);
    emitLifecycleEvent("nominal_nmpc_preflight_start_motor_stopped");
  }
}

inline void Application::pollModelPreflight(uint32_t now) {
  // A model-based feasibility screen, NOT proof of physical reliability.
  // One virtual control update per poll; never writes the hardware command.
  const uint32_t begin = PendulumShield.controlTimeUs();
  if (++virtualSamples_ > 4000 || !controller_->processMeasurement(virtualState_(1)) ||
      std::abs(controller_->velocity) > T(adaptive_hardware::wheelSpeedGuard) ||
      std::abs(virtualState_(1)) > T(7) * pendulum::kPi<T> / T(6)) {
    enterStoppedState("nominal_nmpc_preflight_rejected", Phase::IdentificationRejected);
    return;
  }
  pendulum::Matrix<T, 3> next;
  if (!pendulum::integrateRK4(virtualState_, controller_->acceleration, model_, best_.parameters[3],
                              T(0), next)) {
    enterStoppedState("nominal_preflight_nonfinite", Phase::IdentificationRejected);
    return;
  }
  virtualState_ = next;
  computeUs_ = PendulumShield.controlTimeUs() - begin;
  peakComputeUs_ = std::max(peakComputeUs_, computeUs_);
  if (controller_->cycle.complete) {
    if (!controller_->beginValidatedSettling(latest_ - origin_, measuredOmega_, best_.parameters[3],
                                             speed_ == T(0))) {
      enterStoppedState("handover_state_invalid", Phase::ManualReturn);
      return;
    }
    committed_ = true;
    diagnostics_.requestControlConfiguration();
    startupSettling_ = true;
    controlStartMs_ = now;
    lastCommandUs_ = lastAcknowledgedUs_;
    lastControlReadUs_ = acquisitionUs_;
    controlTimingReady_ = true;
    transitionTo(Phase::SettlingControl, now);
    emitLifecycleEvent("validated_model_committed_controlled_settling");
    reportLifecycle();
  }
}

inline void Application::startControlCycle(uint32_t now) {
  if (!committed_ || !controller_ || !rest_.restReady(now)) {
    return;
  }
  if (!controller_->resetAtRest(latest_ - origin_, best_.parameters[3])) {
    enterStoppedState("controller_rest_reset_failed", Phase::ManualReturn);
    return;
  }
  if (!publishSpeedCommand(T(0))) {
    return;
  }
  lastCommandUs_ = lastAcknowledgedUs_;
  lastControlReadUs_ = acquisitionUs_;
  controlTimingReady_ = true;
  controlStartMs_ = now;
  recovering_ = false;
  startupSettling_ = false;
  ++controlCycleNumber_;
  lastReason_ = "none"; // prior ID rejection remains in its event, not an active control fault
  transitionTo(Phase::Control, now);
  emitLifecycleEvent("identified_model_nmpc_cycle_start");
}

inline void Application::processControlSample(T angle, uint32_t now) {
  const uint32_t start = PendulumShield.controlTimeUs();
  if (uint32_t(now - controlStartMs_) > 400000) {
    enterStoppedState("control_timeout", Phase::ManualReturn);
    return;
  }
  if (!controller_->cycle.returning && uint32_t(now - controlStartMs_) >= 240000) {
    if (!controller_->requestReturnToDown()) {
      enterStoppedState("return_failed", Phase::ManualReturn);
      return;
    }
    recovering_ = true;
  }
  pendulum::MeasurementTiming<T> timing;
  if (!controlTimingReady_ ||
      !pendulum::MeasurementTiming<T>::fromTimestamps(
          lastControlReadUs_, acquisitionUs_, lastAcknowledgedUs_, Limits::period, timing)) {
    enterStoppedState("control_timestamp_invalid", Phase::ManualReturn);
    return;
  }
  // A failed solve may follow an already committed EKF measurement. Advance
  // chronology even without a new command: the old acknowledgement must not
  // replay its speed increment in the NEXT interval.
  lastControlReadUs_ = acquisitionUs_;
  const ControlBudgetContext clock{acquisitionUs_};
  const pendulum::SolverWorkBudget budget{controlWorkBudgetExpired, &clock};
  const bool solved = controller_->processMeasurement(angle, 1, T(-1), &timing, budget);
  computeUs_ = PendulumShield.controlTimeUs() - start;
  peakComputeUs_ = std::max(peakComputeUs_, computeUs_);
  // Recovery must not bypass late-command rejection, even if the solve is
  // unusable. The budget is cooperative; a single evaluation can overrun it.
  if (PendulumShield.getTick() != lastTick_ ||
      uint32_t(PendulumShield.controlTimeUs() - acquisitionUs_) >= 100000U) {
    recordControlDiagnostics(angle, controller_->velocity, controller_->acceleration, speed_, T(1),
                             PendulumShield.controlTimeUs(), 0, false);
    enterStoppedState("computation_missed_sample_deadline", Phase::ManualReturn);
    return;
  }
  if (!solved) {
    recordControlDiagnostics(angle, controller_->velocity, controller_->acceleration, speed_, T(1),
                             PendulumShield.controlTimeUs(), 0, false);
    if (!recovering_ && controller_->requestReturnToDown()) {
      recovering_ = true;
      emitLifecycleEvent("numerical_return_requested");
      return;
    }
    enterStoppedState("nmpc_or_ekf_failure", Phase::ManualReturn);
    return;
  }
  if (std::abs(controller_->estimator.state(1)) > T(7) * pendulum::kPi<T> / T(6) ||
      std::abs(controller_->velocity) > T(adaptive_hardware::wheelSpeedGuard)) {
    recordControlDiagnostics(angle, controller_->velocity, controller_->acceleration, speed_, T(1),
                             PendulumShield.controlTimeUs(), 0, false);
    enterStoppedState("control_angle_or_speed_guard", Phase::ManualReturn);
    return;
  }
  const uint32_t commandUs = PendulumShield.controlTimeUs();
  const uint32_t intervalUs = uint32_t(commandUs - lastCommandUs_);
  const T plannedSpeed = controller_->velocity, plannedAcceleration = controller_->acceleration;
  T timingScale = T(1);
  if (!controller_->cycle.complete) {
    const T dt = std::min(Limits::period, T(uint32_t(commandUs - lastCommandUs_)) * T(1e-6));
    const T original = controller_->acceleration;
    timingScale = dt / Limits::period;
    controller_->acceleration *= timingScale;
    controller_->velocity += (controller_->acceleration - original) * Limits::period;
  }
  const T previousSpeed = speed_;
  if (!publishSpeedCommand(controller_->velocity)) {
    recordControlDiagnostics(angle, plannedSpeed, plannedAcceleration, previousSpeed, timingScale,
                             PendulumShield.controlTimeUs(), intervalUs, false);
    return;
  }
  // Account for the driver's sub-one-step/s zero-speed deadband. These
  // values are software acknowledgements, NOT measured rotor feedback.
  controller_->velocity = speed_;
  controller_->acceleration = (speed_ - previousSpeed) / Limits::period;
  recordControlDiagnostics(angle, plannedSpeed, plannedAcceleration, previousSpeed, timingScale,
                           lastAcknowledgedUs_, intervalUs, true);
  lastCommandUs_ = lastAcknowledgedUs_;
  if (controller_->cycle.complete) {
    if (startupSettling_) {
      transitionTo(Phase::Cooldown, now);
      emitLifecycleEvent("identified_model_downward_settled");
      return;
    }
    if (!recovering_) {
      ++successfulCycles_;
      consecutiveRecoveries_ = 0;
    } else if (++consecutiveRecoveries_ >= 2) {
      enterStoppedState("recovery_limit", Phase::ManualReturn);
      return;
    }
    transitionTo(Phase::Cooldown, now);
    emitLifecycleEvent(recovering_ ? "controlled_return_complete" : "cycle_complete");
  }
}

inline void Application::abortIdentification(const char *reason, uint32_t now,
                                             bool reduceAmplitude) {
  diagnostics_.endIdentification();
  committed_ = false;
  lastReason_ = reason;
  emitLifecycleEvent(reason);
  // Repeated identification never silently invokes a nominal controller.
  if (attempts_ < 3) {
    if (reduceAmplitude) {
      amplitude_ *= T(.5);
    }
    transitionTo(speed_ == T(0) ? Phase::StartupRest : Phase::BrakeToRetry, now);
    originFrozen_ = true;
    emitLifecycleEvent("bounded_identification_retry");
  } else {
    enterStoppedState(reason, Phase::IdentificationRejected);
  }
}

inline void Application::enterStoppedState(const char *reason, Phase phase) {
  diagnostics_.endIdentification();
  lastReason_ = reason;
  PendulumShield.failStop(PendulumClass::RuntimeFault::Communication);
  speed_ = T(0);
  phase_ = phase;
  phaseStartMs_ = millis();
  restartReady_ = false;
  releaseSeen_ = false;
  restartPressSeen_ = false;
  rest_.discardRest();
  emitLifecycleEvent(reason);
}

inline void Application::observeRestartButton(uint32_t now) {
  const bool pressed = standalone_platform::buttonPressed();
  const bool manual = requiresManualAcknowledgement(phase_);
  if (!manual) {
    return;
  }
  if (pressed != buttonCandidate_) {
    buttonCandidate_ = pressed;
    buttonChangeMs_ = now;
  }
  if (uint32_t(now - buttonChangeMs_) < 20) {
    return;
  }
  if (!pressed) {
    if (restartPressSeen_ && releaseSeen_) {
      restartReady_ = true;
    }
    releaseSeen_ = true;
  } else if (releaseSeen_) {
    restartPressSeen_ = true;
  }
}

inline void Application::restartAfterManualAcknowledgement(uint32_t now) {
  const uint32_t epoch = pressEpoch;
  {
    InterruptLock lock;
    if (!standalone_platform::buttonPressed() && epoch == pressEpoch) {
      emergency = false;
    }
  }
  if (emergency || standalone_platform::buttonPressed()) {
    return;
  }
  if (!PendulumShield.actuatorWrite(0.f)) {
    enterStoppedState("restart_zero_failed", Phase::HardwareFault);
    return;
  }
  // Clear recoverable driver faults without enabling, then confirm the released
  // zero command through the atomic emergency-checked publication path.
  if (!publishSpeedCommand(T(0))) {
    return;
  }
  attempts_ = 0;
  consecutiveRecoveries_ = 0;
  restartReady_ = false;
  transitionTo(committed_ ? Phase::Cooldown : Phase::StartupRest, now);
  emitLifecycleEvent("manual_stopped_wheel_acknowledgement");
}

inline void Application::emitLifecycleEvent(const char *reason) {
  diagnostics_.event(reason, phase_, attempts_);
}

inline void Application::reportLifecycle() {
  diagnostics_.report({model_, best_, validation_, phase_, committed_, attempts_,
                       controlCycleNumber_, rest_.consecutiveRestSamples(),
                       uint32_t(millis() - phaseStartMs_), startupPhaseTimeoutMs(), computeUs_,
                       peakComputeUs_, virtualSamples_, fitter_.result.exit, lastReason_, latest_,
                       origin_, speed_});
}

inline void Application::drainDiagnostics() {
  diagnostics_.drain(Serial, {config_, controller_ ? &*controller_ : nullptr, origin_,
                              originFrozen_, standalone_platform::coreClockHz()});
}

inline void Application::recordControlDiagnostics(T rawPhi, T plannedSpeed, T plannedAcceleration,
                                                  T previousSpeed, T timingScale,
                                                  uint32_t publishedUs, uint32_t intervalUs,
                                                  bool issued) {
  if (!controller_) {
    return;
  }
  diagnostics_.recordControl(*controller_,
                             {startupSettling_ ? 0 : controlCycleNumber_, lastTick_, acquisitionUs_,
                              computeUs_, PendulumShield.getMissedSampleCount(),
                              PendulumShield.getDiscardedPulseDeadlineCount()},
                             rawPhi, plannedSpeed, plannedAcceleration, previousSpeed, speed_,
                             timingScale, publishedUs, intervalUs, issued);
}

namespace {
Application application;
} // namespace
} // namespace detail

void initialize() {
  detail::application.initialize();
}
void poll() {
  detail::application.poll();
}
Status status() {
  return detail::application.status();
}
} // namespace adaptive_application
