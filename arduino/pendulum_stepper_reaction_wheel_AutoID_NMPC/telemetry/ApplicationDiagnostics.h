#ifndef ADAPTIVE_APPLICATIONDIAGNOSTICS_H
#define ADAPTIVE_APPLICATIONDIAGNOSTICS_H
#include "application/Lifecycle.h"
#include "application/BuildInfo.h"
#include "platform/HardwareConfig.h"
#include "telemetry/ControlDiagnostics.h"
#include "telemetry/IdentificationStream.h"
#include "control/CycleController.h"
#include "identification/GuidedExcitation.h"
#include "identification/Validation.h"
#include <cstring>
namespace adaptive_application {
template <class T> struct DiagnosticContext {
  const pendulum::ControllerConfig<T> &config;
  const pendulum::CycleController<T> *controller;
  T encoderOrigin;
  bool calibrationReady;
  uint32_t coreClockHz;
};
template <class T> struct LifecycleSnapshot {
  const pendulum::ModelParameters<T> &model;
  const identification::FitResult<T> &fit;
  const identification::Validation<T> &validation;
  Phase phase;
  bool modelCommitted;
  unsigned attempts;
  uint32_t cycle, restSamples, stageElapsedMs, stageLimitMs;
  uint32_t computeUs, peakComputeUs, virtualSamples;
  identification::FitExit fitExit;
  const char *reason;
  T latestAngle, encoderOrigin, speedCommand;
};
struct ControlSampleMetadata {
  uint32_t cycle, tick, readUs, solveUs, missedSamples, discardedPulses;
};
// Owns all diagnostic storage/serialization, but no lifecycle, model updates,
// hardware commands or clocks. Contexts are short-lived read-only views.
// SerialPort is injected; the numeric/formatting component has no GPIO/UART dependency.
template <class T> class ApplicationDiagnostics final {
  using Controller = pendulum::CycleController<T>;
  using Limits = identification::Policy<T>;

public:
  ApplicationDiagnostics() = default;
  ApplicationDiagnostics(const ApplicationDiagnostics &) = delete;
  ApplicationDiagnostics &operator=(const ApplicationDiagnostics &) = delete;
  uint32_t droppedReports() const { return droppedTelemetry_; }
  size_t pendingControlSamples() const { return diagnosticRecorder_.pending(); }
  void recordFrame(control_diagnostics::Frame<T> frame) {
    if (diagnosticRecorder_.push(frame) && diagnosticStreaming_ &&
        diagnosticOverwritten_ < UINT32_MAX) {
      ++diagnosticOverwritten_;
    }
  }
  void requestControlConfiguration() { pendingControlConfiguration_ = true; }
  void requestValidationReport(const identification::Validation<T> &record, unsigned attempt,
                               uint32_t generation) {
    if (pendingValidation_) {
      countDroppedReport();
    }
    validationReport_ = record;
    validationAttempt_ = attempt;
    validationGeneration_ = generation;
    pendingValidation_ = true;
  }
  void requestRefinementReport(const identification::FitResult<T> &coarse,
                               const identification::FitResult<T> &refined, T peak,
                               unsigned attempt, uint32_t generation) {
    if (pendingRefinement_) {
      countDroppedReport();
    }
    refinement_ = {peak,
                   coarse.parameters[1],
                   refined.parameters[1],
                   coarse.standardDeviation[1],
                   refined.standardDeviation[1],
                   coarse.standardDeviation[2],
                   refined.standardDeviation[2]};
    refinementAttempt_ = attempt;
    refinementGeneration_ = generation;
    pendingRefinement_ = true;
  }
  void handleInput(int command, Phase phase, unsigned attempt, bool modelCommitted) {
    if (command == 'd' || command == 'D') {
      diagnosticStreaming_ = !diagnosticStreaming_;
      event(diagnosticStreaming_ ? "control_diagnostics_enabled" : "control_diagnostics_paused",
            phase, attempt);
    } else if (command == 'h' || command == 'H') {
      pendingBuildInfo_ = pendingDiagnosticHeader_ = true;
      pendingControlConfiguration_ = modelCommitted;
    } else if (command == 'p' || command == 'P') {
      diagnosticRecorder_.replayHistory();
      diagnosticStreaming_ = true;
      event("control_diagnostics_history_replay", phase, attempt);
    }
  }
  void beginIdentification(uint32_t generation, unsigned attempt, uint32_t kind, uint32_t expected,
                           uint32_t readUs, T angle) {
    identificationStream_.begin(generation, attempt, kind, expected, readUs, angle);
  }
  void identificationSample(uint32_t index, uint32_t readUs, T angle, T heldSpeed, uint32_t ackUs) {
    identificationStream_.sample(index, readUs, angle, heldSpeed, ackUs);
  }
  void endIdentification() { identificationStream_.end(); }
  template <class SerialPort> void drain(SerialPort &serial, const DiagnosticContext<T> &context) {
    const int available = serial.availableForWrite();
    if (available <= 0) {
      return;
    }
    flushEvent();
    flushIdentificationStream();
    flushIdentificationTelemetry();
    flushAdvancedDiagnostics(context);
    const auto chunk = telemetry_.nextChunk(size_t(available));
    if (chunk.size && !telemetry_.consume(serial.write(chunk.data, chunk.size))) {
      countDroppedReport();
    }
  }
  void event(const char *reason, Phase phase, unsigned attempt) {
    // Own deferred text. A caller may reuse its buffer immediately after return.
    char copy[sizeof pendingEventText_];
    size_t length = 0;
    standalone_telemetry::Writer writer(copy, sizeof copy);
    writer.text(reason);
    if (!writer.finish(length)) {
      countDroppedReport();
      return;
    }
    if (pendingEvent_) {
      countDroppedReport();
    }
    std::memcpy(pendingEventText_, copy, length + 1);
    pendingEvent_ = true;
    pendingEventPhase_ = uint32_t(phase);
    pendingEventAttempt_ = attempt;
    flushEvent();
  }
  void report(const LifecycleSnapshot<T> &context) {
    char text[768];
    size_t n = 0;
    standalone_telemetry::Writer w(text, sizeof text);
    w.text("adaptive,phase=")
        .integer(uint32_t(context.phase))
        .text(",committed=")
        .integer(context.modelCommitted ? 1U : 0U)
        .text(",alpha=")
        .number(context.model.alpha)
        .text(",beta=")
        .number(context.model.beta)
        .text(",gamma=")
        .number(context.model.gamma)
        .text(",coulomb=")
        .number(context.model.coulomb)
        .text(",train_rms=")
        .number(context.fit.rms)
        .text(",validation_rms=")
        .number(context.validation.rms)
        .text(",condition=")
        .number(context.fit.condition)
        .text(",fit_exit=")
        .integer(uint32_t(context.fitExit))
        .text(",reason=")
        .text(context.reason)
        .text(",angle=")
        .number(context.latestAngle - context.encoderOrigin)
        .text(",wheel_command=")
        .number(context.speedCommand)
        .text(",attempt=")
        .integer(context.attempts)
        .text(",cycle=")
        .integer(context.cycle)
        .text(",diagnostics_overwritten=")
        .integer(diagnosticOverwritten_)
        .text(",rest_samples=")
        .integer(context.restSamples)
        .text(",stage_elapsed_ms=")
        .integer(context.stageElapsedMs)
        .text(",stage_limit_ms=")
        .integer(context.stageLimitMs)
        .text(",compute_us=")
        .integer(context.computeUs)
        .text(",compute_peak_us=")
        .integer(context.peakComputeUs)
        .text(",virtual_samples=")
        .integer(context.virtualSamples)
        .text(",tx_dropped=")
        .integer(droppedTelemetry_)
        .text(",identification_stream_dropped=")
        .integer(identificationStream_.dropped())
        .text("\n");
    if (!w.finish(n) || !telemetry_.queueText(text)) {
      countDroppedReport();
    }
  }
  void reportFit(const identification::FitResult<T> &r, unsigned seed, bool guidedPass,
                 unsigned attempt, uint32_t generation) {
    char text[768];
    size_t n = 0;
    standalone_telemetry::Writer w(text, sizeof text);
    w.text("id_fit,seed=")
        .integer(uint32_t(seed))
        .text(",attempt=")
        .integer(attempt)
        .text(",generation=")
        .integer(generation)
        .text(",pass=")
        .integer(guidedPass ? 1U : 0U)
        .text(",iterations=")
        .integer(uint32_t(r.iterations))
        .text(",accepted=")
        .integer(uint32_t(r.accepted))
        .text(",rejected=")
        .integer(uint32_t(r.rejected))
        .text(",exit=")
        .integer(uint32_t(r.exit))
        .text(",usable=")
        .integer(r.usable ? 1U : 0U)
        .text(",gradient=")
        .number(r.gradient)
        .text(",gn_decrease=")
        .number(r.linearDecrease)
        .text(",scaled_step=")
        .number(r.scaledStep)
        .text(",cost_resolution=")
        .number(r.costResolution)
        .text(",alpha=")
        .number(r.parameters[0])
        .text(",beta=")
        .number(r.parameters[1])
        .text(",gamma=")
        .number(r.parameters[2])
        .text(",bias=")
        .number(r.parameters[3])
        .text(",coulomb=")
        .number(r.parameters[4])
        .text(",rms=")
        .number(r.rms)
        .text(",condition=")
        .number(r.condition)
        .text(",sd_alpha=")
        .number(r.standardDeviation[0])
        .text(",sd_beta=")
        .number(r.standardDeviation[1])
        .text(",sd_gamma=")
        .number(r.standardDeviation[2])
        .text(",sd_coulomb=")
        .number(r.standardDeviation[4])
        .text("\n");
    if (!w.finish(n) || !telemetry_.queueText(text)) {
      countDroppedReport();
    }
  }
  void recordControl(const Controller &c, const ControlSampleMetadata &metadata, T rawPhi,
                     T plannedSpeed, T plannedAcceleration, T previousSpeed, T sentSpeed,
                     T timingScale, uint32_t publishedUs, uint32_t intervalUs, bool issued) {
    control_diagnostics::Frame<T> f;
    f.cycle = metadata.cycle;
    f.tick = metadata.tick;
    f.time = c.cycle.elapsed();
    f.phase = uint32_t(c.cycle.phase);
    f.rawPhi = rawPhi;
    f.phi = c.estimator.state(1);
    f.omega = c.estimator.state(0);
    f.disturbance = c.estimator.state(3);
    f.reference = c.referenceAngle;
    f.angleError = c.angleError;
    f.energy = c.currentEnergy;
    f.energyReference = c.energyReference;
    f.relativeEnergy = c.relativeEnergyError;
    f.membership = c.stabilizeWeight;
    f.gates = c.captureGateMask;
    f.captureSamples = c.captureSamples;
    f.captured = c.captured ? 1U : 0U;
    f.preview = c.previewWeight;
    f.terminal = c.terminalWeight;
    f.previousSpeed = previousSpeed;
    f.plannedSpeed = plannedSpeed;
    f.sentSpeed = issued ? sentSpeed : previousSpeed;
    f.plannedAcceleration = plannedAcceleration;
    f.sentAcceleration = issued ? c.acceleration : T(0);
    f.timingScale = timingScale;
    f.decision = c.plan(0);
    f.delayUs = uint32_t(publishedUs - metadata.readUs);
    f.sampleReadUs = metadata.readUs;
    f.commandAckUs = publishedUs;
    f.solveUs = metadata.solveUs;
    f.intervalUs = intervalUs;
    f.exit = uint32_t(c.solver.exit);
    f.iterations = uint32_t(c.solver.iterations);
    f.gradient = c.solver.gradientNorm;
    f.cost = c.solver.cost;
    f.derivativeEvaluations = uint32_t(c.solver.derivativeEvaluations);
    f.costEvaluations = uint32_t(c.solver.costEvaluations);
    f.nextPhi = c.solver.firstPrediction(1);
    f.nextOmega = c.solver.firstPrediction(0);
    f.terminalPhi = c.solver.lastPrediction(1);
    f.terminalOmega = c.solver.lastPrediction(0);
    f.fault = uint32_t(c.fault);
    f.issued = issued ? 1U : 0U;
    f.missed = metadata.missedSamples;
    f.pulses = metadata.discardedPulses;
    f.wheelState = c.estimator.state(2);
    f.innovation = c.innovation.innovation;
    f.innovationVariance = c.innovation.variance;
    f.converged = c.solver.converged ? 1U : 0U;
    if (diagnosticRecorder_.push(f) && diagnosticStreaming_ &&
        diagnosticOverwritten_ < UINT32_MAX) {
      ++diagnosticOverwritten_;
    }
  }
  void rememberDesign(const identification::GuidedExcitation<T> &guide,
                      const identification::FitResult<T> &fit, uint32_t kind, unsigned attempt,
                      uint32_t generation) {
    if (pendingDesign_) {
      countDroppedReport();
    }
    designAttempt_ = attempt;
    designGeneration_ = generation;
    designKind_ = kind;
    designNatural_ = guide.naturalFrequency();
    designAmplitude_ = guide.currentAmplitude();
    designFriction_ = fit.parameters[4];
    for (int j = 0; j < 3; ++j) {
      designParameters_[j] = fit.parameters[j];
    }
    pendingDesign_ = true;
  }

private:
  void countDroppedReport() {
    if (droppedTelemetry_ < UINT32_MAX) {
      ++droppedTelemetry_;
    }
  }
  void flushEvent() {
    if (!pendingEvent_ || !telemetry_.textQueueAvailable()) {
      return;
    }
    char text[180];
    size_t n = 0;
    standalone_telemetry::Writer w(text, sizeof text);
    w.text("event,")
        .text(pendingEventText_)
        .text(",phase=")
        .integer(pendingEventPhase_)
        .text(",attempt=")
        .integer(pendingEventAttempt_)
        .text("\n");
    if (w.finish(n) && telemetry_.queueText(text)) {
      pendingEvent_ = false;
    }
  }
  void flushAdvancedDiagnostics(const DiagnosticContext<T> &context) {
    if (!telemetry_.textQueueAvailable()) {
      return;
    }
    if (pendingDiagnosticHeader_) {
      if (telemetry_.queueText(control_diagnostics::schema)) {
        pendingDiagnosticHeader_ = false;
      }
      return;
    }
    char text[1024];
    size_t n = 0;
    standalone_telemetry::Writer w(text, sizeof text);
    if (pendingBuildInfo_) {
      w.text("diagnostics_build,version=")
          .integer(adaptive_build::diagnosticVersion)
          .text(",driver_profile=")
          .integer(ADAPTIVE_DRIVER_PROFILE)
          .text(",candidate=")
          .text(adaptive_build::candidate)
          .text(",u_swing=")
          .number(context.config.u_limit_swing)
          .text(",u_hold=")
          .number(context.config.u_limit_stabilize)
          .text(",id_acceleration=")
          .number(Limits::excitationAcceleration)
          .text(",wheel_speed_guard=")
          .number(T(adaptive_hardware::wheelSpeedGuard))
          .text(",microsteps=")
          .integer(ADAPTIVE_MICROSTEPS)
          .text(",hold_at_zero=")
          .integer(ADAPTIVE_HOLD_AT_ZERO)
          .text(",single_precision=")
          .integer(NMPC_USE_SINGLE_PRECISION)
          .text(",source_fingerprint=")
          .text(adaptive_build::fingerprint)
          .text(",core_clock_hz=")
          .integer(context.coreClockHz)
          .text(",Ts=")
          .number(Limits::period)
          .text(",streaming=")
          .integer(diagnosticStreaming_ ? 1U : 0U)
          .text(",encoder_origin_rad=")
          .number(context.encoderOrigin)
          .text(",calibration_ready=")
          .integer(context.calibrationReady ? 1U : 0U)
          .text(",identification_protocol=2,id_queue_packets=8,control_history_samples=16")
          .text("\n");
      if (w.finish(n) && telemetry_.queueText(text)) {
        pendingBuildInfo_ = false;
      }
      return;
    }
    if (pendingControlConfiguration_ && context.controller) {
      const auto &c = *context.controller;
      w.text("control_config,active_horizon=")
          .integer(uint32_t(context.config.activeHorizon))
          .text(",energy_weight=")
          .number(context.config.energy_cost_weight)
          .text(",R_swing=")
          .number(context.config.R_swing)
          .text(",R_hold=")
          .number(context.config.R_stabilize)
          .text(",swing_wheel_weight=")
          .number(context.config.swing(2, 2))
          .text(",nonlinear_wheel_weight=")
          .number(context.config.wheel_nonlinear_weight)
          .text(",u_swing=")
          .number(context.config.u_limit_swing)
          .text(",u_hold=")
          .number(context.config.u_limit_stabilize)
          .text(",blend_outer_rad=")
          .number(context.config.angle_blend_high)
          .text(",capture_angle_rad=")
          .number(context.config.momentum_capture_angle_tolerance)
          .text(",capture_speed=")
          .number(pendulum::ControlTuning<T>::captureAngularSpeed)
          .text(",capture_membership=")
          .number(pendulum::ControlTuning<T>::captureMembership)
          .text(",capture_samples=")
          .integer(pendulum::ControlTuning<T>::captureSamples)
          .text(",max_iterations=")
          .integer(uint32_t(context.config.maxIterations))
          .text(",swing_angle_weight=")
          .number(context.config.swing(1, 1))
          .text(",solver_regularization=")
          .number(c.solverSettings.regularization)
          .text(",ekf_disturbance_variance=")
          .number(pendulum::ControlTuning<T>::disturbanceProcessVariance)
          .text(",ekf_velocity_variance=")
          .number(pendulum::ControlTuning<T>::velocityProcessVariance)
          .text(",ekf_angle_variance=")
          .number(pendulum::ControlTuning<T>::angleProcessVariance)
          .text(",holding_speed_penalty_fraction=")
          .number(pendulum::ControlTuning<T>::holdingSpeedPenaltyFraction)
          .text(",max_backtracks=")
          .integer(uint32_t(c.solverSettings.maxBacktracks))
          .text(",gradient_tolerance=")
          .number(c.solverSettings.gradientTolerance)
          .text(",control_work_budget_us=")
          .integer(pendulum::ControlTuning<T>::controlWorkBudgetUs)
          .text(
              ",timing_model=acknowledged_jump,speed_cost_latch_protected=1,holding_cost_latched=1")
          .text(",candidate=")
          .text(adaptive_build::candidate)
          .text(",planning_model=previous_ack_delay")
          .text(",alpha=")
          .number(c.model.alpha)
          .text(",beta=")
          .number(c.model.beta)
          .text(",gamma=")
          .number(c.model.gamma)
          .text(",coulomb=")
          .number(c.model.coulomb)
          .text("\n");
      if (w.finish(n) && telemetry_.queueText(text)) {
        pendingControlConfiguration_ = false;
      }
      return;
    }
    if (!diagnosticStreaming_) {
      return;
    }
    const auto *frame = diagnosticRecorder_.front();
    if (frame && control_diagnostics::format(text, sizeof text, *frame, n) &&
        telemetry_.queueText(text)) {
      diagnosticRecorder_.consume();
    }
  }
  void flushIdentificationTelemetry() {
    if ((!pendingDesign_ && !pendingValidation_ && !pendingRefinement_) ||
        !telemetry_.textQueueAvailable()) {
      return;
    }
    char text[768];
    size_t n = 0;
    standalone_telemetry::Writer w(text, sizeof text);
    if (pendingDesign_) {
      w.text("id_design,kind=")
          .integer(designKind_)
          .text(",alpha=")
          .number(designParameters_[0])
          .text(",beta=")
          .number(designParameters_[1])
          .text(",gamma=")
          .number(designParameters_[2])
          .text(",coulomb=")
          .number(designFriction_)
          .text(",attempt=")
          .integer(designAttempt_)
          .text(",generation=")
          .integer(designGeneration_)
          .text(",natural_rad_s=")
          .number(designNatural_)
          .text(",initial_speed_amplitude=")
          .number(designAmplitude_)
          .text(",duration_s=")
          .integer(60U)
          .text(",target_first_deg=")
          .integer(designKind_ == 1 ? 20U : 25U)
          .text(",target_second_deg=")
          .integer(designKind_ == 1 ? 30U : 35U)
          .text(",angle_guard_rad=")
          .number(Limits::maxAngle)
          .text("\n");
    } else if (pendingRefinement_) {
      w.text("id_refinement,peak_angle_rad=")
          .number(refinement_.peak)
          .text(",attempt=")
          .integer(refinementAttempt_)
          .text(",generation=")
          .integer(refinementGeneration_)
          .text(",coarse_beta=")
          .number(refinement_.coarseBeta)
          .text(",refined_beta=")
          .number(refinement_.refinedBeta)
          .text(",coarse_sd_beta=")
          .number(refinement_.coarseSdBeta)
          .text(",refined_sd_beta=")
          .number(refinement_.refinedSdBeta)
          .text(",coarse_sd_gamma=")
          .number(refinement_.coarseSdGamma)
          .text(",refined_sd_gamma=")
          .number(refinement_.refinedSdGamma)
          .text("\n");
    } else {
      w.text("id_validation,accepted=")
          .integer(validationReport_.accepted ? 1U : 0U)
          .text(",attempt=")
          .integer(validationAttempt_)
          .text(",generation=")
          .integer(validationGeneration_)
          .text(",rms=")
          .number(validationReport_.rms)
          .text(",peak=")
          .number(validationReport_.peak)
          .text(",initial_velocity=")
          .number(validationReport_.initialVelocity);
      for (int b = 0; b < identification::kValidationBands; ++b) {
        w.text(",band")
            .integer(uint32_t(b))
            .text("_rms=")
            .number(validationReport_.bandRms[b])
            .text(",band")
            .integer(uint32_t(b))
            .text("_signal=")
            .number(validationReport_.bandSignal[b]);
      }
      w.text("\n");
    }
    if (w.finish(n) && telemetry_.queueText(text)) {
      if (pendingDesign_) {
        pendingDesign_ = false;
      } else if (pendingRefinement_) {
        pendingRefinement_ = false;
      } else {
        pendingValidation_ = false;
      }
    }
  }
  void flushIdentificationStream() {
    if (!identificationStream_.pending() || !telemetry_.textQueueAvailable()) {
      return;
    }
    char text[256];
    if (identificationStream_.format(text, sizeof text) && telemetry_.queueText(text)) {
      identificationStream_.consume();
    }
  }

  standalone_telemetry::TextQueue<1024> telemetry_;
  control_diagnostics::Recorder<T> diagnosticRecorder_;
  identification_stream::Stream<T> identificationStream_;
  bool diagnosticStreaming_ = ADAPTIVE_CONTROL_DIAGNOSTICS != 0;
  bool pendingDiagnosticHeader_ = true, pendingBuildInfo_ = true,
       pendingControlConfiguration_ = false;
  bool pendingDesign_ = false, pendingValidation_ = false, pendingRefinement_ = false;
  uint32_t diagnosticOverwritten_ = 0, droppedTelemetry_ = 0;
  char pendingEventText_[96]{};
  bool pendingEvent_ = false;
  uint32_t pendingEventPhase_ = 0, pendingEventAttempt_ = 0, designKind_ = 0;
  struct RefinementReport {
    T peak = T(0), coarseBeta = T(0), refinedBeta = T(0), coarseSdBeta = T(0), refinedSdBeta = T(0),
      coarseSdGamma = T(0), refinedSdGamma = T(0);
  } refinement_;
  identification::Validation<T> validationReport_;
  uint32_t designAttempt_ = 0, designGeneration_ = 0, refinementAttempt_ = 0,
           refinementGeneration_ = 0, validationAttempt_ = 0, validationGeneration_ = 0;
  T designParameters_[3]{}, designNatural_ = T(0), designAmplitude_ = T(0), designFriction_ = T(0);
};
} // namespace adaptive_application
#endif
