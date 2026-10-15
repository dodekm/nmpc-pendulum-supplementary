#include <StandalonePlatform.h>
#include "AdaptiveApplication.h"
#include "HardwareConfig.h"
#include "ApplicationDiagnostics.h"
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
namespace {
using T = pendulum::RuntimeScalar;
using Controller = pendulum::CycleController<T>;
using Limits = identification::Policy<T>;
volatile bool emergency = false;
volatile uint32_t pressEpoch = 0;
void buttonInterrupt() {
  if (standalone_platform::buttonPressed()) {
    ++pressEpoch;
    emergency = true;
    PendulumShield.failStop(PendulumClass::RuntimeFault::Communication);
  }
}
class Application {
  friend struct TestAccess;

public:
  Application() = default;
  Application(const Application &) = delete;
  Application &operator=(const Application &) = delete;
  ~Application() = default;
  void initialize() {
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
      stop("initialization_failure", Phase::HardwareFault);
      return;
    }
    if (emergency || standalone_platform::buttonPressed()) {
      stop("button_held_at_boot", Phase::EmergencyStop);
      return;
    }
    if (!write(T(0))) {
      return;
    }
    event("boot_auto_identification_downward_stopped_required");
  }
  Status status() const {
    return {phase_,      committed_,   attempts_, successfulCycles_, model_.alpha,
            model_.beta, model_.gamma, best_.rms, validation_.rms};
  }
  void poll() {
    drain();
    const uint32_t now = millis();
    bool led = isMoving();
    if (phase_ == Phase::EmergencyStop) {
      led = now % 400 < 200;
    } else if (phase_ == Phase::HardwareFault) {
      led = now % 1000 < 100 || (now % 1000 >= 250 && now % 1000 < 350);
    } else if (phase_ == Phase::IdentificationRejected || phase_ == Phase::ManualReturn) {
      led = now % 1000 < 150;
    } else if (!isMoving()) {
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
    observeButton(now);
    const bool manual = requiresManualAcknowledgement(phase_);
    if ((emergency || standalone_platform::buttonPressed()) && !manual &&
        phase_ != Phase::HardwareFault) {
      stop("emergency_stop", Phase::EmergencyStop);
      return;
    }
    if (phase_ == Phase::HardwareFault) {
      return;
    }
    const auto fault = PendulumShield.getRuntimeFault();
    if (fault == PendulumClass::RuntimeFault::SensorRead ||
        fault == PendulumClass::RuntimeFault::Magnet) {
      stop("sensor_origin_uncertain_reset_required", Phase::HardwareFault);
      return;
    }
    if (!manual && fault != PendulumClass::RuntimeFault::None) {
      stop("driver_fault", Phase::ManualReturn);
      return;
    }
    if (uint32_t(now - lastSampleMs_) > 500) {
      stop("sample_timeout_reset_required", Phase::HardwareFault);
      return;
    }
    if (!processFreshSample(now, manual)) {
      return;
    }
    if (phase_ == Phase::Fitting) {
      fitPoll(now);
    }
    if (phase_ == Phase::ValidationProcessing) {
      validationPoll(now);
    }
    if (phase_ == Phase::DesignUpright || phase_ == Phase::DesignDownward) {
      designPoll(now);
    }
    if (phase_ == Phase::ModelPreflight) {
      preflightPoll(now);
    }
    // Re-read BOTH lifecycle and clock after potentially state-changing work.
    // stop() can stamp a newer millis() value; subtracting the entry timestamp
    // would wrap and overwrite the actual rejection with a false timeout.
    const bool stopped = isStopped(phase_);
    const uint32_t afterWork = millis();
    if (!isMoving() && !stopped && uint32_t(afterWork - phaseStartMs_) > startupTimeLimit()) {
      stop(phase_ == Phase::ValidationRest || phase_ == Phase::ResonanceRest
               ? "identification_settling_timeout"
               : "startup_stage_timeout",
           Phase::IdentificationRejected);
    }
    if (now - lastStatusMs_ >= 1000) {
      lastStatusMs_ = now;
      report();
    }
  }

private:
  struct ControlBudgetContext { uint32_t sampleReadUs; };
  static bool controlBudgetExpired(const void *context) {
    const auto &clock = *static_cast<const ControlBudgetContext *>(context);
    return uint32_t(PendulumShield.controlTimeUs() - clock.sampleReadUs) >=
           pendulum::ControlTuning<T>::controlWorkBudgetUs;
  }
  bool processFreshSample(uint32_t now, bool manual) {
    PendulumSample sample;
    if (PendulumShield.readSample(sample)) {
      const uint32_t steps = sample.tick - lastTick_;
      if (sample.status != SampleStatus::Valid || !std::isfinite(sample.angle) || steps != 1) {
        stop("sample_epoch_or_quality_failure", Phase::HardwareFault);
        return false;
      }
      lastTick_ = sample.tick;
      acquisitionUs_ = sample.acquisitionUs;
      lastSampleMs_ = now;
      latest_ = T(sample.angle);
      const T angle = latest_ - origin_;
      measuredOmega_ =
          (T(3) * angle - T(4) * previousAngle_ + olderAngle_) / (T(2) * Limits::period);
      olderAngle_ = previousAngle_;
      previousAngle_ = angle;
      if (originFrozen_ && std::abs(angle) > T(7) * pendulum::kPi<T> / T(6) && !manual) {
        stop("rollover_manual_unwind", Phase::ManualReturn);
        return false;
      }
      rest_.observeRest(angle, 1, now);
      if (!originFrozen_) {
        if (havePrevious_ && std::abs(latest_ - previous_) > T(.005)) {
          zero_ = pendulum::RunningStatistics<T>();
        }
        if (!zero_.add(latest_)) {
          stop("invalid_origin_statistics", Phase::HardwareFault);
          return false;
        }
        previous_ = latest_;
        havePrevious_ = true;
      }
      if (manual) {
        if (restartReady_ && rest_.restReady(now) && !standalone_platform::buttonPressed()) {
          restart(now);
        }
      } else if (isAcquiring(phase_)) {
        acquire(angle, now);
      } else if (phase_ == Phase::BrakeToRetry) {
        if (!write(identification::limitedVelocity(T(0), speed_))) {
          return false;
        }
        if (speed_ == T(0)) {
          transition(Phase::StartupRest, now);
        }
      } else if (phase_ == Phase::Control || phase_ == Phase::SettlingControl) {
        control(angle, now);
      } else {
        advanceRest(now);
      }
    }
    return true;
  }
  bool isMoving() const { return phaseMayActuate(phase_); }
  uint32_t startupTimeLimit() const {
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
  void transition(Phase p, uint32_t now) {
    phase_ = p;
    phaseStartMs_ = now;
    rest_.discardRest();
  }
  bool write(T velocity) {
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
      stop("command_or_emergency_failure", Phase::EmergencyStop);
      return false;
    }
    speed_ = acknowledged;
    lastAcknowledgedUs_ = acknowledgedUs;
    return true;
  }
  void beginAcquisitionRecord(identification::RecordKind kind) {
    acquisitionData_.intervals = 0;
    acquisitionData_.angle[0] = latest_ - origin_;
    speed_ = T(0);
    diagnostics_.beginIdentification(++acquisitionGeneration_, attempts_, uint32_t(kind),
                                     identification::kTrainIntervals, acquisitionUs_,
                                     acquisitionData_.angle[0]);
  }
  void startTraining(uint32_t now) {
    guidedPass_ = false;
    guidedPeak_ = T(0);
    pulseCount_ = PendulumShield.getDiscardedPulseDeadlineCount();
    ++attempts_;
    beginAcquisitionRecord(identification::RecordKind::Coarse);
    transition(Phase::Training, now);
    event("training_chirp_start");
    issueChirp(false);
  }
  void startValidation(uint32_t now) {
    if (!validationGuide_.begin(best_.parameters[0], best_.parameters[1], best_.parameters[2],
                                amplitude_, true, best_.parameters[4])) {
      stop("validation_design_invalid", Phase::IdentificationRejected);
      return;
    }
    beginAcquisitionRecord(identification::RecordKind::Validation);
    transition(Phase::Validation, now);
    diagnostics_.rememberDesign(validationGuide_, best_, 1, attempts_, acquisitionGeneration_);
    event("independent_validation_chirp_start");
    issueChirp(true);
  }
  void startResonanceTraining(uint32_t now) {
    if (!guide_.begin(coarse_.parameters[0], coarse_.parameters[1], coarse_.parameters[2],
                      amplitude_, false, coarse_.parameters[4])) {
      stop("resonance_design_invalid", Phase::IdentificationRejected);
      return;
    }
    beginAcquisitionRecord(identification::RecordKind::Guided);
    guidedPass_ = true;
    guidedPeak_ = T(0);
    transition(Phase::ResonanceTraining, now);
    diagnostics_.rememberDesign(guide_, coarse_, 2, attempts_, acquisitionGeneration_);
    event("resonance_training_start");
    issueChirp(false);
  }
  void issueChirp(bool validation) {
    const T requested =
        validation    ? validationGuide_.command(acquisitionData_.intervals, latest_ - origin_)
        : guidedPass_ ? guide_.command(acquisitionData_.intervals, latest_ - origin_)
                      : identification::coarseChirp<T>(acquisitionData_.intervals, amplitude_);
    if (!std::isfinite(requested)) {
      stop("invalid_excitation_request", Phase::IdentificationRejected);
      return;
    }
    const T command = identification::limitedVelocity(requested, speed_);
    if (!write(command)) {
      return;
    }
    acquisitionData_.command[acquisitionData_.intervals] = speed_;
  }
  void acquire(T angle, uint32_t now) {
    const bool validating = phase_ == Phase::Validation;
    if (PendulumShield.getDiscardedPulseDeadlineCount() != pulseCount_) {
      stop("pulse_service_loss_manual_wheel_check", Phase::ManualReturn);
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
      if (!write(T(0))) {
        return;
      }
      if (validating) {
        if (!validator_.begin(acquisitionData_, best_)) {
          abortIdentification("invalid_validation_record_or_model", now, false);
          return;
        }
        transition(Phase::ValidationProcessing, now);
        event("validation_prediction_assessment_motor_stopped");
      } else {
        seed_ = 0;
        best_ = identification::FitResult<T>();
        if (!fitter_.begin(acquisitionData_)) {
          abortIdentification("insufficient_excitation_or_seed", now, false);
          return;
        }
        transition(Phase::Fitting, now);
        event("training_complete_motor_stopped_fit_start");
      }
    } else {
      issueChirp(validating);
    }
  }
  void advanceRest(uint32_t now) {
    // Fitting needs no current pendulum rest. Independent validation uses a
    // reserved prefix to estimate its initial velocity, not plant parameters.
    if (phase_ == Phase::ValidationRest) {
      if (rest_.restReady(now) && speed_ == T(0)) {
        startValidation(now);
      }
      return;
    }
    if (phase_ == Phase::ResonanceRest) {
      if (rest_.restReady(now) && speed_ == T(0)) {
        startResonanceTraining(now);
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
      startTraining(now);
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
        event("high_viscous_load_warning_use_nmpc_preflight");
      }
      if (!pendulum::configureIdentifiedModel(model_, config_)) {
        abortIdentification("model_tuning_invalid", now, false);
        return;
      }
      if (!design_.begin(model_, config_.hold, config_.R_stabilize, true)) {
        abortIdentification("terminal_design_failed", now, false);
        return;
      }
      transition(Phase::DesignUpright, now);
    } else if (phase_ == Phase::Cooldown) {
      startControl(now);
    }
  }
  void fitPoll(uint32_t now) {
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
        transition(Phase::ResonanceRest, now);
        event("coarse_model_ready_for_resonance_design");
      } else {
        if (guidedPeak_ < T(.1745329252)) {
          abortIdentification("guided_excitation_too_weak", now, false);
          return;
        }
        diagnostics_.requestRefinementReport(coarse_, best_, guidedPeak_, attempts_,
                                             acquisitionGeneration_);
        transition(Phase::ValidationRest, now);
        event("refined_model_selected_without_validation_data");
      }
      report();
    }
  }
  void validationPoll(uint32_t now) {
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
    transition(Phase::DesignRest, now);
    event("independent_model_validation_passed");
  }
  void designPoll(uint32_t now) {
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
      transition(Phase::DesignDownward, now);
    } else {
      downwardTail_ = design_.tail;
      controller_.emplace(model_, config_, uprightTail_, downwardTail_);
      diagnostics_.requestControlConfiguration();
      if (!controller_->resetAtRest(T(0), best_.parameters[3])) {
        stop("preflight_initialization_failed", Phase::IdentificationRejected);
        return;
      }
      virtualState_.setZero();
      virtualSamples_ = 0;
      transition(Phase::ModelPreflight, now);
      event("nominal_nmpc_preflight_start_motor_stopped");
    }
  }
  void preflightPoll(uint32_t now) {
    // A model-based feasibility screen, NOT proof of physical reliability.
    // One virtual control update per poll; never writes the hardware command.
    const uint32_t begin = PendulumShield.controlTimeUs();
    if (++virtualSamples_ > 4000 || !controller_->processMeasurement(virtualState_(1)) ||
        std::abs(controller_->velocity) > T(adaptive_hardware::wheelSpeedGuard) ||
        std::abs(virtualState_(1)) > T(7) * pendulum::kPi<T> / T(6)) {
      stop("nominal_nmpc_preflight_rejected", Phase::IdentificationRejected);
      return;
    }
    pendulum::Matrix<T, 3> next;
    if (!pendulum::integrateRK4(virtualState_, controller_->acceleration, model_,
                                best_.parameters[3], T(0), next)) {
      stop("nominal_preflight_nonfinite", Phase::IdentificationRejected);
      return;
    }
    virtualState_ = next;
    computeUs_ = PendulumShield.controlTimeUs() - begin;
    peakComputeUs_ = std::max(peakComputeUs_, computeUs_);
    if (controller_->cycle.complete) {
      if (!controller_->beginValidatedSettling(latest_ - origin_, measuredOmega_,
                                               best_.parameters[3], speed_ == T(0))) {
        stop("handover_state_invalid", Phase::ManualReturn);
        return;
      }
      committed_ = true;
      diagnostics_.requestControlConfiguration();
      startupSettling_ = true;
      controlStartMs_ = now;
      lastCommandUs_ = lastAcknowledgedUs_;
      lastControlReadUs_ = acquisitionUs_;
      controlTimingReady_ = true;
      transition(Phase::SettlingControl, now);
      event("validated_model_committed_controlled_settling");
      report();
    }
  }
  void startControl(uint32_t now) {
    if (!committed_ || !controller_ || !rest_.restReady(now)) {
      return;
    }
    if (!controller_->resetAtRest(latest_ - origin_, best_.parameters[3])) {
      stop("controller_rest_reset_failed", Phase::ManualReturn);
      return;
    }
    if (!write(T(0))) {
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
    transition(Phase::Control, now);
    event("identified_model_nmpc_cycle_start");
  }
  void control(T angle, uint32_t now) {
    const uint32_t start = PendulumShield.controlTimeUs();
    if (uint32_t(now - controlStartMs_) > 400000) {
      stop("control_timeout", Phase::ManualReturn);
      return;
    }
    if (!controller_->cycle.returning && uint32_t(now - controlStartMs_) >= 240000) {
      if (!controller_->requestReturnToDown()) {
        stop("return_failed", Phase::ManualReturn);
        return;
      }
      recovering_ = true;
    }
    pendulum::MeasurementTiming<T> timing;
    if (!controlTimingReady_ ||
        !pendulum::MeasurementTiming<T>::fromTimestamps(lastControlReadUs_, acquisitionUs_,
                                                       lastAcknowledgedUs_, Limits::period, timing)) {
      stop("control_timestamp_invalid", Phase::ManualReturn);
      return;
    }
    // A failed solve may follow an already committed EKF measurement. Advance
    // chronology even without a new command: the old acknowledgement must not
    // replay its speed increment in the NEXT interval.
    lastControlReadUs_ = acquisitionUs_;
    const ControlBudgetContext clock{acquisitionUs_};
    const pendulum::SolverWorkBudget budget{controlBudgetExpired, &clock};
    const bool solved = controller_->processMeasurement(angle, 1, T(-1), &timing, budget);
    computeUs_ = PendulumShield.controlTimeUs() - start;
    peakComputeUs_ = std::max(peakComputeUs_, computeUs_);
    // Recovery must not bypass late-command rejection, even if the solve is
    // unusable. The budget is cooperative; a single evaluation can overrun it.
    if (PendulumShield.getTick() != lastTick_ ||
        uint32_t(PendulumShield.controlTimeUs() - acquisitionUs_) >= 100000U) {
      recordControl(angle, controller_->velocity, controller_->acceleration, speed_, T(1),
                    PendulumShield.controlTimeUs(), 0, false);
      stop("computation_missed_sample_deadline", Phase::ManualReturn);
      return;
    }
    if (!solved) {
      recordControl(angle, controller_->velocity, controller_->acceleration, speed_, T(1),
                    PendulumShield.controlTimeUs(), 0, false);
      if (!recovering_ && controller_->requestReturnToDown()) {
        recovering_ = true;
        event("numerical_return_requested");
        return;
      }
      stop("nmpc_or_ekf_failure", Phase::ManualReturn);
      return;
    }
    if (std::abs(controller_->estimator.state(1)) > T(7) * pendulum::kPi<T> / T(6) ||
        std::abs(controller_->velocity) > T(adaptive_hardware::wheelSpeedGuard)) {
      recordControl(angle, controller_->velocity, controller_->acceleration, speed_, T(1),
                    PendulumShield.controlTimeUs(), 0, false);
      stop("control_angle_or_speed_guard", Phase::ManualReturn);
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
    if (!write(controller_->velocity)) {
      recordControl(angle, plannedSpeed, plannedAcceleration, previousSpeed, timingScale,
                    PendulumShield.controlTimeUs(), intervalUs, false);
      return;
    }
    // Account for the driver's sub-one-step/s zero-speed deadband. These
    // values are software acknowledgements, NOT measured rotor feedback.
    controller_->velocity = speed_;
    controller_->acceleration = (speed_ - previousSpeed) / Limits::period;
    recordControl(angle, plannedSpeed, plannedAcceleration, previousSpeed, timingScale,
                  lastAcknowledgedUs_, intervalUs, true);
    lastCommandUs_ = lastAcknowledgedUs_;
    if (controller_->cycle.complete) {
      if (startupSettling_) {
        transition(Phase::Cooldown, now);
        event("identified_model_downward_settled");
        return;
      }
      if (!recovering_) {
        ++successfulCycles_;
        consecutiveRecoveries_ = 0;
      } else if (++consecutiveRecoveries_ >= 2) {
        stop("recovery_limit", Phase::ManualReturn);
        return;
      }
      transition(Phase::Cooldown, now);
      event(recovering_ ? "controlled_return_complete" : "cycle_complete");
    }
  }
  void abortIdentification(const char *reason, uint32_t now, bool reduceAmplitude) {
    diagnostics_.endIdentification();
    committed_ = false;
    lastReason_ = reason;
    event(reason);
    // Repeated identification never silently invokes a nominal controller.
    if (attempts_ < 3) {
      if (reduceAmplitude) {
        amplitude_ *= T(.5);
      }
      transition(speed_ == T(0) ? Phase::StartupRest : Phase::BrakeToRetry, now);
      originFrozen_ = true;
      event("bounded_identification_retry");
    } else {
      stop(reason, Phase::IdentificationRejected);
    }
  }
  void stop(const char *reason, Phase phase) {
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
    event(reason);
  }
  void observeButton(uint32_t now) {
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
  void restart(uint32_t now) {
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
      stop("restart_zero_failed", Phase::HardwareFault);
      return;
    }
    // Clear recoverable driver faults without enabling, then restore holding
    // through the atomic emergency-checked publication path.
    if (!write(T(0))) {
      return;
    }
    attempts_ = 0;
    consecutiveRecoveries_ = 0;
    restartReady_ = false;
    transition(committed_ ? Phase::Cooldown : Phase::StartupRest, now);
    event("manual_stopped_wheel_acknowledgement");
  }
  void event(const char *reason) { diagnostics_.event(reason, phase_, attempts_); }
  void report() {
    diagnostics_.report({model_, best_, validation_, phase_, committed_, attempts_,
                         controlCycleNumber_, rest_.consecutiveRestSamples(),
                         uint32_t(millis() - phaseStartMs_), startupTimeLimit(), computeUs_,
                         peakComputeUs_, virtualSamples_, fitter_.result.exit, lastReason_, latest_,
                         origin_, speed_});
  }
  void drain() {
    diagnostics_.drain(Serial, {config_, controller_ ? &*controller_ : nullptr, origin_,
                                originFrozen_, standalone_platform::coreClockHz()});
  }
  void recordControl(T rawPhi, T plannedSpeed, T plannedAcceleration, T previousSpeed,
                     T timingScale, uint32_t publishedUs, uint32_t intervalUs, bool issued) {
    if (!controller_) {
      return;
    }
    diagnostics_.recordControl(*controller_,
                               {startupSettling_ ? 0 : controlCycleNumber_, lastTick_,
                                acquisitionUs_, computeUs_, PendulumShield.getMissedSampleCount(),
                                PendulumShield.getDiscardedPulseDeadlineCount()},
                               rawPhi, plannedSpeed, plannedAcceleration, previousSpeed, speed_,
                               timingScale, publishedUs, intervalUs, issued);
  }
  std::optional<Controller> controller_;
  pendulum::Matrix<T, 3> virtualState_ = pendulum::Matrix<T, 3>::Zero();
  uint32_t virtualSamples_ = 0;
  identification::Data<T, identification::kTrainIntervals> acquisitionData_;
  // Training, guided training, and validation are sequential. Reuse one solver
  // buffer only after its consumer finishes; telemetry owns no trajectory copy.
  static_assert(identification::kTrainIntervals == identification::kValidationIntervals,
                "Shared acquisition buffer requires matching record capacities.");
  ApplicationDiagnostics<T> diagnostics_;
  identification::Fitter<T> fitter_;
  identification::FitResult<T> best_;
  identification::FitResult<T> coarse_;
  identification::GuidedExcitation<T> guide_, validationGuide_;
  identification::Validation<T> validation_;
  identification::Validator<T> validator_;
  pendulum::TerminalDesign<T> design_;
  pendulum::ModelParameters<T> model_;
  pendulum::ControllerConfig<T> config_;
  pendulum::Matrix<T, 3, 3> uprightTail_, downwardTail_;
  pendulum::RestDetector<T> rest_;
  pendulum::RunningStatistics<T> zero_;
  uint32_t controlCycleNumber_ = 0, acquisitionUs_ = 0;
  uint32_t lastAcknowledgedUs_ = 0, acquisitionGeneration_ = 0;
  uint32_t lastControlReadUs_ = 0;
  Phase phase_ = Phase::StartupRest;
  uint32_t lastTick_ = 0, lastSampleMs_ = 0, phaseStartMs_ = 0, controlStartMs_ = 0,
           lastCommandUs_ = 0, computeUs_ = 0, lastStatusMs_ = 0;
  uint32_t buttonChangeMs_ = 0, pulseCount_ = 0, peakComputeUs_ = 0;
  const char *lastReason_ = "none";
  T origin_ = T(0), latest_ = T(0), previous_ = T(0), speed_ = T(0), amplitude_ = T(1);
  T previousAngle_ = T(0), olderAngle_ = T(0), measuredOmega_ = T(0);
  T guidedPeak_ = T(0);
  unsigned attempts_ = 0, successfulCycles_ = 0, consecutiveRecoveries_ = 0;
  int seed_ = 0;
  bool initialized_ = false, committed_ = false, originFrozen_ = false, havePrevious_ = false,
       recovering_ = false, startupSettling_ = false, controlTimingReady_ = false;
  bool guidedPass_ = false;
  bool restartReady_ = false, releaseSeen_ = false, restartPressSeen_ = false,
       buttonCandidate_ = false;
};
Application application;
} // namespace
void initialize() {
  application.initialize();
}
void poll() {
  application.poll();
}
Status status() {
  return application.status();
}
} // namespace adaptive_application
