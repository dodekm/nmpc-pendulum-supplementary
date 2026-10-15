#ifndef ADAPTIVE_CYCLECONTROLLER_H
#define ADAPTIVE_CYCLECONTROLLER_H
#pragma once
#include "ControllerConfig.h"
#include "GaussNewton.h"
#include "AugmentedEKF.h"
#include "CycleSupervisor.h"

namespace pendulum {
enum class ControlFault : uint8_t {
  None,
  InvalidMeasurement,
  Estimator,
  Preview,
  Solver,
  ReturnTimeout,
  Duration,
  InvalidCommand,
  InvalidConfiguration
};
template <class Scalar> class CycleController {
  using Tuning = ControlTuning<Scalar>;

public:
  CycleController(const ModelParameters<Scalar> &identifiedModel,
                  const ControllerConfig<Scalar> &settings, const Matrix<Scalar, 3, 3> &uprightTail,
                  const Matrix<Scalar, 3, 3> &downwardTail)
      : config(settings), model(identifiedModel), uprightTail_(uprightTail),
        downwardTail_(downwardTail) {}
  CycleController(const CycleController &) = delete;
  CycleController &operator=(const CycleController &) = delete;
  const ControllerConfig<Scalar> config;
  const ModelParameters<Scalar> model;
  AugmentedEKF<Scalar> estimator;
  InnovationDiagnostics<Scalar> innovation;
  InputPlan<Scalar> plan = InputPlan<Scalar>::Zero();
  CycleState<Scalar> cycle{model.samplePeriod};
  SolverResult<Scalar> solver;
  SolverSettings<Scalar> solverSettings;
  ControlFault fault = ControlFault::None;
  Scalar acceleration = Scalar(0), velocity = Scalar(0), uprightLift = kPi<Scalar>;
  Scalar referenceAngle = uprightLift, stabilizeWeight = Scalar(0), terminalWeight = Scalar(0),
         previewWeight = Scalar(0);
  Scalar momentumWeight = Scalar(0), managedVelocity = Scalar(0), latchedVelocity = Scalar(0),
         wheelReference = Scalar(0);
  bool captured = false, unloading = false;
  uint32_t captureSamples = 0, unloadSamples = 0, sampleCount = 0;
  Scalar currentEnergy = Scalar(0), relativeEnergyError = Scalar(0), angleError = Scalar(0),
         energyReference = Scalar(0);
  uint32_t captureGateMask = 0; // bit 0 angle, bit 1 speed, bit 2 membership

  // Only the application may establish physical rest. Never call this while
  // moving: wheel speed is commanded, not independently measured.
  bool resetAtRest(Scalar measuredAngle, Scalar initialDisturbance = Scalar(0)) {
    if (!supervisor_.isValid() || !model.isValid() || !std::isfinite(measuredAngle) ||
        !std::isfinite(initialDisturbance) ||
        std::abs(measuredAngle) > config.cycle_down_angle_tolerance) {
      return false;
    }
    initializeState(measuredAngle, initialDisturbance);
    return true;
  }

  // Different contract from resetAtRest: the pendulum may still oscillate,
  // but the driver has acknowledged zero wheel command and the model has
  // passed independent validation. Only the downward problem is activated.
  bool beginValidatedSettling(Scalar angle, Scalar omega, Scalar disturbance,
                              bool zeroWheelAcknowledged) {
    if (!zeroWheelAcknowledged || !supervisor_.isValid() || !model.isValid() ||
        !std::isfinite(angle) || std::abs(angle) > kPi<Scalar> / Scalar(2) ||
        !std::isfinite(omega) || std::abs(omega) > Scalar(10) || !std::isfinite(disturbance)) {
      return false;
    }
    initializeState(angle, disturbance);
    estimator.state(0) = omega;
    estimator.covariance(0, 0) = Scalar(.01);
    estimator.covariance(1, 1) = Scalar(5e-6);
    return requestReturnToDown();
  }

  // Reuse the existing downward problem and solver, on the NEXT fresh sample.
  // Preview/solver failures occur after the EKF has committed, before any new
  // actuator command. Do not assimilate that measurement twice or replay its
  // previous speed increment during the next prediction.
  bool requestReturnToDown() {
    const bool failedSolve = fault == ControlFault::Preview || fault == ControlFault::Solver;
    if ((fault != ControlFault::None && !failedSolve) || cycle.returning || cycle.complete ||
        cycle.timedOut || !estimator.state.allFinite() ||
        !isCovarianceValid(estimator.covariance) || !std::isfinite(acceleration) ||
        !std::isfinite(velocity)) {
      return false;
    }
    if (failedSolve) {
      acceleration = Scalar(0);
    }
    fault = ControlFault::None;
    plan.setZero();
    cycle.returning = true;
    cycle.phase = CyclePhase::SwingDown;
    cycle.returnStartSteps = cycle.elapsedSteps;
    cycle.uprightSamples = cycle.downSamples = 0;
    captured = unloading = false;
    captureSamples = unloadSamples = 0;
    return true;
  }

  bool processMeasurement(Scalar angle, int elapsedIntervals = 1,
                          Scalar commandInterval = Scalar(-1),
                          const MeasurementTiming<Scalar> *timing = nullptr,
                          const SolverWorkBudget &budget = SolverWorkBudget()) {
    if (fault != ControlFault::None || cycle.complete) {
      return false;
    }
    if (!supervisor_.isValid() || !model.isValid()) {
      return fail(ControlFault::InvalidConfiguration);
    }
    if (!std::isfinite(commandInterval) ||
        (commandInterval < Scalar(0) && commandInterval != Scalar(-1))) {
      return fail(ControlFault::InvalidCommand);
    }
    if (!std::isfinite(angle) || elapsedIntervals < 1 || elapsedIntervals > 100) {
      return fail(ControlFault::InvalidMeasurement);
    }
    if (cycle.elapsedSteps >= uint32_t(config.maxSamples) ||
        uint32_t(elapsedIntervals) > uint32_t(config.maxSamples) - cycle.elapsedSteps) {
      return fail(ControlFault::Duration);
    }
    if (!estimator.step(acceleration, angle, model, elapsedIntervals, innovation, timing)) {
      return fail(ControlFault::Estimator);
    }
    ++sampleCount;
    MPCProblem<Scalar> problem;
    const Scalar planningDelay = timing && timing->commandOccurred ? timing->commandDelay : Scalar(0);
    if (!prepareProblem(estimator.state.template head<3>(), estimator.state(3), elapsedIntervals,
                        problem, planningDelay)) {
      return false;
    }
    InputPlan<Scalar> warmPlan = shiftInputPlan(plan);
    for (int j = config.activeHorizon; j < kHorizon; ++j) {
      warmPlan(j) = Scalar(0);
    }
    solverSettings.maxIterations = config.maxIterations;
    solver = solveGaussNewton(problem, warmPlan, solverSettings, budget);
    if (!solver.usable || !warmPlan.allFinite()) {
      return fail(ControlFault::Solver);
    }
    plan = warmPlan;
    Scalar applied = problem.accelerationScale * std::tanh(plan(0) / problem.accelerationScale);
    applied = clamp(applied, -problem.accelerationScale, problem.accelerationScale);
    if (cycle.complete) {
      applied = -velocity / model.samplePeriod;
    } else if (commandInterval >= 0) {
      applied *= Scalar(std::min(model.samplePeriod, commandInterval) / model.samplePeriod);
    }
    if (!std::isfinite(applied)) {
      return fail(ControlFault::InvalidCommand);
    }
    acceleration = applied;
    velocity += applied * model.samplePeriod;
    if (cycle.complete) {
      velocity = Scalar(0);
    }
    if (!std::isfinite(velocity)) {
      return fail(ControlFault::InvalidCommand);
    }
    return true;
  }

  // Exposed for frozen-problem/supervisor tests; all parameters are fixed
  // BEFORE solve. Calling it advances the supervisor once, not the AugmentedEKF.
  bool prepareProblem(const Matrix<Scalar, 3> &state, Scalar disturbanceAcceleration,
                      int elapsedIntervals, MPCProblem<Scalar> &problem,
                      Scalar planningDelay = Scalar(0)) {
    if (!state.allFinite() || !std::isfinite(disturbanceAcceleration) || elapsedIntervals < 1 ||
        elapsedIntervals > 100 || sampleCount == 0 || !std::isfinite(planningDelay) ||
        planningDelay < Scalar(0) || planningDelay > model.samplePeriod * Scalar(1.5)) {
      return fail(ControlFault::InvalidMeasurement);
    }
    const Scalar pi = kPi<Scalar>;
    const Scalar wrapped = wrapAngle(state(1) - pi);
    if (std::abs(wrapped) <= pi / Scalar(2)) {
      uprightLift = state(1) - wrapped;
    }
    const Scalar lean =
        -std::asin(clamp(disturbanceAcceleration / model.alpha, -leanClampSine_, leanClampSine_));
    referenceAngle = uprightLift + lean;
    if (!supervisor_.advance(cycle, state, elapsedIntervals, captured, referenceAngle,
                             acceleration)) {
      return fail(ControlFault::InvalidConfiguration);
    }
    if (cycle.timedOut) {
      return fail(ControlFault::ReturnTimeout);
    }
    problem.model = model;
    problem.activeHorizon = config.activeHorizon;
    problem.initialState = state;
    // Use the last acknowledged latency as a causal estimate of the next
    // publication delay. Capture gates remain tied to the measured posterior;
    // only the optimization initial state advances to expected actuation time.
    // This never advances/commits the EKF or changes nominal MPC step lengths.
    if (planningDelay > Scalar(0) &&
        !integrateFreeFlowRK4(state, model, disturbanceAcceleration, planningDelay,
                              problem.initialState)) return fail(ControlFault::Preview);
    problem.disturbanceAcceleration = disturbanceAcceleration;
    problem.wheelSpeedScale = config.wheel_cost_scale;
    problem.accelerationScale = std::min(config.u_limit_swing, config.u_limit_stabilize);
    if (cycle.returning) {
      referenceAngle = -lean;
      stabilizeWeight = Scalar(1);
      previewWeight = Scalar(0);
      terminalWeight = config.cycle_return_terminal_scale;
      for (int stageIndex = 0; stageIndex < kHorizon; ++stageIndex) {
        problem.referenceState[stageIndex] << Scalar(0), referenceAngle, Scalar(0);
        problem.stateWeight[stageIndex] = config.down;
        problem.inputWeight(stageIndex) = config.R_return;
        problem.energyWeight(stageIndex) = Scalar(0);
        problem.wheelSpeedWeight(stageIndex) = config.wheel_nonlinear_weight;
        problem.energyReference(stageIndex) = model.alpha * (Scalar(1) - std::cos(referenceAngle));
      }
      problem.stateWeight[config.activeHorizon - 1] += terminalWeight * downwardTail_;
      for (int j = config.activeHorizon; j < kHorizon; ++j) {
        problem.inputWeight(j) = Scalar(1);
      }
      captured = false;
      momentumWeight = Scalar(0);
      unloading = false;
      wheelReference = Scalar(0);
      angleError = wrapAngle(state(1) - referenceAngle);
      captureGateMask = 0;
      currentEnergy =
          Scalar(.5) * state(0) * state(0) + model.alpha * (Scalar(1) - std::cos(state(1)));
      relativeEnergyError =
          std::abs(currentEnergy - Scalar(2) * model.alpha) / (Scalar(2) * model.alpha);
      energyReference = problem.energyReference(0);
      return true;
    }
    Scalar relativeEnergy;
    stabilizeWeight = membership(state, referenceAngle, relativeEnergy, &currentEnergy);
    relativeEnergyError = relativeEnergy;
    const Scalar error = wrapAngle(state(1) - referenceAngle);
    const bool candidate = stabilizeWeight >= Tuning::captureMembership &&
                           std::abs(error) <= config.momentum_capture_angle_tolerance &&
                           std::abs(state(0)) <= Tuning::captureAngularSpeed;
    angleError = error;
    captureGateMask = (std::abs(error) <= config.momentum_capture_angle_tolerance ? 1U : 0U) |
                      (std::abs(state(0)) <= Tuning::captureAngularSpeed ? 2U : 0U) |
                      (stabilizeWeight >= Tuning::captureMembership ? 4U : 0U);
    captureSamples = candidate ? (elapsedIntervals == 1 ? captureSamples + 1 : 1) : 0;
    if (!captured && captureSamples >= Tuning::captureSamples) {
      captured = true;
      latchedVelocity = state(2);
      managedVelocity = latchedVelocity;
      unloadSamples = 0;
    }
    if (captured && (std::abs(error) > degreesToRadians(Tuning::lostCaptureAngleDegrees) ||
                     relativeEnergy > Tuning::lostCaptureRelativeEnergy)) {
      captured = false;
      captureSamples = 0;
    }
    // Capture confirmation is hysteretic. A small excursion outside the narrow
    // membership window must NOT silently remove angular-velocity feedback and
    // revert a confirmed holding task to energy pumping. Only the explicit
    // lost-capture criteria above (or a requested return) release this latch.
    const Scalar holdingCostWeight = captured ? Scalar(1) : stabilizeWeight;
    const Scalar swing = Scalar(1) - holdingCostWeight;
    const bool quiet = captured && stabilizeWeight >= Tuning::unloadingMembership &&
                       std::abs(error) <= degreesToRadians(Tuning::unloadingAngleDegrees) &&
                       std::abs(state(0)) <= Tuning::unloadingAngularSpeed;
    unloadSamples = quiet ? (elapsedIntervals == 1 ? unloadSamples + 1 : 1) : 0;
    const uint32_t required = unloadDwellIntervals_;
    unloading = captured && config.momentum_unload_rate > Scalar(0) && unloadSamples >= required &&
                std::abs(managedVelocity) > Scalar(0);
    if (unloading) {
      managedVelocity = std::copysign(
          std::max(std::abs(managedVelocity) - config.momentum_unload_rate * model.samplePeriod,
                   Scalar(0)),
          managedVelocity);
    }
    const Scalar weightStep = model.samplePeriod / config.momentum_weight_ramp_time;
    momentumWeight =
        clamp(momentumWeight + (captured ? weightStep : -weightStep), Scalar(0), Scalar(1));
    wheelReference = captured ? managedVelocity : momentumWeight * managedVelocity;
    Matrix<Scalar, 3, 3> hold = config.hold;
    hold(2, 2) += momentumWeight * (config.momentum_wheel_weight - config.hold(2, 2));
    const Matrix<Scalar, 3, 3> stateWeight = swing * config.swing + holdingCostWeight * hold;
    const Scalar inputWeight = swing * config.R_swing + holdingCostWeight * config.R_stabilize;
    problem.accelerationScale =
        config.u_limit_swing + (config.u_limit_stabilize - config.u_limit_swing) * holdingCostWeight;
    previewWeight = Scalar(0);
    Matrix<Scalar, 3> preview = problem.initialState;
    const InputPlan<Scalar> shifted = shiftInputPlan(plan);
    for (int stageIndex = 0; stageIndex < config.activeHorizon; ++stageIndex) {
      Matrix<Scalar, 3> next;
      if (!integrateRK4(preview, shifted(stageIndex), model, disturbanceAcceleration,
                        problem.accelerationScale, next)) {
        return fail(ControlFault::Preview);
      }
      preview = next;
    }
    Scalar previewEnergy;
    previewWeight = membership(preview, referenceAngle, previewEnergy);
    if (std::abs(preview(1) - referenceAngle) > pi / Scalar(2)) {
      previewWeight = Scalar(0);
    }
    terminalWeight = config.terminal_scale * std::max(holdingCostWeight, previewWeight);
    // Membership alone is not a confirmed capture. Retain the high-speed cost
    // through the three-sample dwell, then fade it with the existing momentum
    // weight ramp into the established holding/unloading policy.
    const Scalar wheelPenaltyBlend = captured
        ? std::max(Tuning::holdingSpeedPenaltyFraction, Scalar(1) - holdingCostWeight * momentumWeight)
        : Scalar(1);
    for (int stageIndex = 0; stageIndex < kHorizon; ++stageIndex) {
      problem.referenceState[stageIndex] << Scalar(0), referenceAngle, wheelReference;
      problem.stateWeight[stageIndex] = stateWeight;
      problem.inputWeight(stageIndex) = inputWeight;
      problem.energyWeight(stageIndex) = swing * config.energy_cost_weight;
      problem.wheelSpeedWeight(stageIndex) = config.wheel_nonlinear_weight * wheelPenaltyBlend;
      problem.energyReference(stageIndex) =
          std::min(Scalar(2) * model.alpha, Scalar(sampleCount - 1 + uint32_t(stageIndex)) *
                                                config.energy_reference_increment);
    }
    problem.stateWeight[config.activeHorizon - 1] += terminalWeight * uprightTail_;
    for (int j = config.activeHorizon; j < kHorizon; ++j) {
      problem.inputWeight(j) = Scalar(1);
    }
    if (!captured) {
      problem.referenceState[config.activeHorizon - 1](2) = state(2);
    }
    energyReference = problem.energyReference(0);
    return true;
  }

  static InputPlan<Scalar> shiftInputPlan(const InputPlan<Scalar> &inputPlan) {
    InputPlan<Scalar> next;
    for (int stageIndex = 0; stageIndex < kHorizon - 1; ++stageIndex) {
      next(stageIndex) = inputPlan(stageIndex + 1);
    }
    next(kHorizon - 1) = inputPlan(kHorizon - 1);
    return next;
  }
  static Scalar degreesToRadians(Scalar degrees) {
    return degrees * Scalar(3.1415926535897932384626433832795 / 180.);
  }
  static Scalar wrapAngle(Scalar value) { return std::atan2(std::sin(value), std::cos(value)); }

private:
  const Matrix<Scalar, 3, 3> uprightTail_, downwardTail_;
  void initializeState(Scalar measuredAngle, Scalar disturbance) {
    estimator = AugmentedEKF<Scalar>();
    estimator.state(1) = measuredAngle;
    estimator.state(3) = disturbance;
    innovation = InnovationDiagnostics<Scalar>();
    plan.setZero();
    cycle.reset();
    solver = SolverResult<Scalar>();
    solverSettings = SolverSettings<Scalar>();
    fault = ControlFault::None;
    acceleration = velocity = Scalar(0);
    uprightLift = kPi<Scalar>;
    referenceAngle = uprightLift;
    stabilizeWeight = terminalWeight = previewWeight = Scalar(0);
    momentumWeight = managedVelocity = latchedVelocity = wheelReference = Scalar(0);
    captured = unloading = false;
    captureSamples = unloadSamples = sampleCount = 0;
    currentEnergy = relativeEnergyError = angleError = energyReference = Scalar(0);
    captureGateMask = 0;
  }
  bool fail(ControlFault why) {
    fault = why;
    return false;
  }
  Scalar membership(const Matrix<Scalar, 3> &state, Scalar referenceState, Scalar &relative,
                    Scalar *energyOutput = nullptr) const {
    Scalar energy =
        Scalar(.5) * state(0) * state(0) + model.alpha * (Scalar(1) - std::cos(state(1)));
    if (energyOutput) {
      *energyOutput = energy;
    }
    relative = std::abs(energy - Scalar(2) * model.alpha) / (Scalar(2) * model.alpha);
    Scalar energyBlend = clamp((relative - Tuning::energyBlendLow) /
                                   (config.energy_blend_high - Tuning::energyBlendLow),
                               Scalar(0), Scalar(1));
    Scalar angleBlend = clamp((blendInnerCosine_ - std::cos(state(1) - referenceState)) /
                                  (blendInnerCosine_ - blendOuterCosine_),
                              Scalar(0), Scalar(1));
    return (Scalar(1) - energyBlend * energyBlend * (Scalar(3) - Scalar(2) * energyBlend)) *
           (Scalar(1) - angleBlend * angleBlend * (Scalar(3) - Scalar(2) * angleBlend));
  }
  const Scalar leanClampSine_ = std::sin(degreesToRadians(Tuning::disturbanceLeanLimitDegrees));
  const Scalar blendInnerCosine_ = std::cos(degreesToRadians(Tuning::angleBlendLowDegrees));
  const Scalar blendOuterCosine_ = std::cos(config.angle_blend_high);
  CycleSupervisor<Scalar> supervisor_{config, model.samplePeriod};
  const uint32_t unloadDwellIntervals_ =
      durationIntervalsOrZero(config.momentum_unload_dwell, model.samplePeriod);
};
} // namespace pendulum
#endif // ADAPTIVE_CYCLECONTROLLER_H
