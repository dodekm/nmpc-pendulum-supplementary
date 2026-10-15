#ifndef ADAPTIVE_ADAPTIVEAPPLICATION_H
#define ADAPTIVE_ADAPTIVEAPPLICATION_H
#pragma once
#include "control/Precision.h"
#include <cstdint>
namespace adaptive_application {
// These explicit IDs are the Serial protocol contract, not ordinal comparisons.
enum class Phase : uint8_t {
  StartupRest = 0,
  Training = 1,
  Fitting = 2,
  ValidationRest = 3,
  Validation = 4,
  DesignRest = 5,
  DesignUpright = 6,
  DesignDownward = 7,
  Control = 8,
  Cooldown = 9,
  IdentificationRejected = 10,
  ManualReturn = 11,
  EmergencyStop = 12,
  HardwareFault = 13,
  BrakeToRetry = 14,
  SettlingControl = 15,
  ModelPreflight = 16,
  ResonanceRest = 17,
  ResonanceTraining = 18,
  ValidationProcessing = 19
};
constexpr bool isAcquiring(Phase phase) {
  return phase == Phase::Training || phase == Phase::ResonanceTraining ||
         phase == Phase::Validation;
}
constexpr bool requiresManualAcknowledgement(Phase phase) {
  return phase == Phase::ManualReturn || phase == Phase::EmergencyStop ||
         phase == Phase::IdentificationRejected;
}
constexpr bool isStopped(Phase phase) {
  return requiresManualAcknowledgement(phase) || phase == Phase::HardwareFault;
}
constexpr bool phaseMayActuate(Phase phase) {
  return isAcquiring(phase) || phase == Phase::Control || phase == Phase::BrakeToRetry ||
         phase == Phase::SettlingControl;
}
struct Status {
  Phase phase;
  bool modelCommitted;
  unsigned attempts, successfulCycles;
  pendulum::RuntimeScalar alpha, beta, gamma, fitRms, validationRms;
};
void initialize();
void poll();
Status status();
} // namespace adaptive_application
#endif // ADAPTIVE_ADAPTIVEAPPLICATION_H
