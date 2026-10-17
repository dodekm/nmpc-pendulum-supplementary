#ifndef ADAPTIVE_APPLICATION_APPLICATION_H
#define ADAPTIVE_APPLICATION_APPLICATION_H
#pragma once

#include "Lifecycle.h"
#include "../telemetry/ApplicationDiagnostics.h"
#include "../identification/Validation.h"
#include "../identification/GuidedExcitation.h"
#include "../identification/RecordKind.h"
#include "../control/TerminalDesign.h"
#include "../control/CycleController.h"
#include "../control/RunningStatistics.h"
#include "../control/RestDetector.h"
#include <optional>

namespace adaptive_application {
namespace detail {

using T = pendulum::RuntimeScalar;
using Controller = pendulum::CycleController<T>;
using Limits = identification::Policy<T>;

// Private composition root, not a public controller API. Owns each workspace
// exactly once and coordinates their lifetimes through the explicit Phase contract.
// Hardware interaction remains in the single application translation unit.
class Application final {
  friend struct TestAccess;

public:
  Application() = default;
  Application(const Application &) = delete;
  Application &operator=(const Application &) = delete;
  ~Application() = default;

  void initialize();
  void poll();
  Status status() const;

private:
  struct ControlBudgetContext {
    uint32_t sampleReadUs;
  };

  // Sampling and lifecycle: preserve sample chronology before advancing a stage.
  bool processPendingSample(uint32_t now, bool manual);
  bool isActuationPhase() const;
  uint32_t startupPhaseTimeoutMs() const;
  void transitionTo(Phase p, uint32_t now);
  void advanceRestPhase(uint32_t now);

  // One emergency-checked publication boundary owns every acknowledged command.
  bool publishSpeedCommand(T velocity);

  // Identification: coarse acquisition, guided refinement, independent validation.
  void beginAcquisitionRecord(identification::RecordKind kind);
  void startCoarseTraining(uint32_t now);
  void startValidationAcquisition(uint32_t now);
  void startGuidedTraining(uint32_t now);
  void publishExcitationCommand(bool validation);
  void processAcquisitionSample(T angle, uint32_t now);
  void pollIdentificationFit(uint32_t now);
  void pollValidation(uint32_t now);
  void abortIdentification(const char *reason, uint32_t now, bool reduceAmplitude);

  // Controller preparation and operation share the same identified model.
  static bool controlWorkBudgetExpired(const void *context);
  void pollTerminalDesign(uint32_t now);
  void pollModelPreflight(uint32_t now);
  void startControlCycle(uint32_t now);
  void processControlSample(T angle, uint32_t now);

  // Stopping/restart is centralized; faults never bypass manual acknowledgement.
  void enterStoppedState(const char *reason, Phase phase);
  void observeRestartButton(uint32_t now);
  void restartAfterManualAcknowledgement(uint32_t now);

  // Diagnostics receives read-only snapshots and never owns workflow decisions.
  void emitLifecycleEvent(const char *reason);
  void reportLifecycle();
  void drainDiagnostics();
  void recordControlDiagnostics(T rawPhi, T plannedSpeed, T plannedAcceleration, T previousSpeed,
                                T timingScale, uint32_t publishedUs, uint32_t intervalUs,
                                bool issued);

  // Declaration order is intentional: preserve construction order and one copy
  // of every numerical/acquisition workspace across the sequential stages.
  std::optional<Controller> controller_;
  pendulum::Matrix<T, 3> virtualState_ = pendulum::Matrix<T, 3>::Zero();
  uint32_t virtualSamples_ = 0;

  // Training, guided training, and validation are sequential. Reuse one solver
  // buffer only after its consumer finishes; telemetry owns no trajectory copy.
  identification::Data<T, identification::kTrainIntervals> acquisitionData_;
  static_assert(identification::kTrainIntervals == identification::kValidationIntervals,
                "Shared acquisition buffer requires matching record capacities.");
  ApplicationDiagnostics<T> diagnostics_;
  identification::Fitter<T> fitter_;
  identification::FitResult<T> best_;
  identification::FitResult<T> coarse_;
  identification::GuidedExcitation<T> guide_;
  identification::GuidedExcitation<T> validationGuide_;
  identification::Validation<T> validation_;
  identification::Validator<T> validator_;
  pendulum::TerminalDesign<T> design_;
  pendulum::ModelParameters<T> model_;
  pendulum::ControllerConfig<T> config_;
  pendulum::Matrix<T, 3, 3> uprightTail_;
  pendulum::Matrix<T, 3, 3> downwardTail_;
  pendulum::RestDetector<T> rest_;
  pendulum::RunningStatistics<T> zero_;

  // Sample, command, and lifecycle clocks have distinct meanings and must not
  // be interchanged. Unsigned arithmetic intentionally supports clock rollover.
  uint32_t controlCycleNumber_ = 0;
  uint32_t acquisitionUs_ = 0;
  uint32_t lastAcknowledgedUs_ = 0;
  uint32_t acquisitionGeneration_ = 0;
  uint32_t lastControlReadUs_ = 0;
  Phase phase_ = Phase::StartupRest;
  uint32_t lastTick_ = 0;
  uint32_t lastSampleMs_ = 0;
  uint32_t phaseStartMs_ = 0;
  uint32_t controlStartMs_ = 0;
  uint32_t lastCommandUs_ = 0;
  uint32_t computeUs_ = 0;
  uint32_t lastStatusMs_ = 0;
  uint32_t buttonChangeMs_ = 0;
  uint32_t pulseCount_ = 0;
  uint32_t peakComputeUs_ = 0;
  const char *lastReason_ = "none";

  // Encoder origin is frozen before excitation; speed_ is the software
  // acknowledgement, not a mechanically measured rotor velocity.
  T origin_ = T(0);
  T latest_ = T(0);
  T previous_ = T(0);
  T speed_ = T(0);
  T amplitude_ = T(1);
  T previousAngle_ = T(0);
  T olderAngle_ = T(0);
  T measuredOmega_ = T(0);
  T guidedPeak_ = T(0);

  unsigned attempts_ = 0;
  unsigned successfulCycles_ = 0;
  unsigned consecutiveRecoveries_ = 0;
  int seed_ = 0;
  bool initialized_ = false;
  bool committed_ = false;
  bool originFrozen_ = false;
  bool havePrevious_ = false;
  bool recovering_ = false;
  bool startupSettling_ = false;
  bool controlTimingReady_ = false;
  bool guidedPass_ = false;
  bool restartReady_ = false;
  bool releaseSeen_ = false;
  bool restartPressSeen_ = false;
  bool buttonCandidate_ = false;
};

} // namespace detail
} // namespace adaptive_application

#endif // ADAPTIVE_APPLICATION_APPLICATION_H
