#ifndef ADAPTIVE_CONTROLDIAGNOSTICS_H
#define ADAPTIVE_CONTROLDIAGNOSTICS_H
#include "StandaloneTelemetry.h"
namespace control_diagnostics {
inline constexpr char schema[] =
    "control_diag_header,seq,cycle,tick,t_s,phase,phi_raw,phi_est,omega_est,disturbance,ref_phi,"
    "angle_error,E,Eref,relative_E_error,membership,capture_gates,capture_samples,captured,preview_"
    "membership,terminal_weight,v_previous,v_planned,v_sent,u_planned,u_sent,timing_scale,"
    "decision0,sample_to_command_us,solve_us,command_interval_us,sample_read_us,command_ack_us,"
    "solver_exit,iterations,gradient,cost,derivative_evals,cost_evals,pred_phi_next,pred_omega_"
    "next,pred_phi_terminal,pred_omega_terminal,control_fault,issued,missed_samples,discarded_"
    "pulses,wheel_state,innovation,innovation_variance,converged\n";
template <class T> struct Frame {
  uint32_t sequence = 0, cycle = 0, tick = 0, phase = 0, gates = 0, captureSamples = 0,
           captured = 0;
  uint32_t delayUs = 0, solveUs = 0, intervalUs = 0, exit = 0, iterations = 0,
           derivativeEvaluations = 0, costEvaluations = 0;
  uint32_t sampleReadUs = 0, commandAckUs = 0;
  uint32_t fault = 0, issued = 0, missed = 0, pulses = 0;
  uint32_t converged = 0;
  T time = T(0), rawPhi = T(0), phi = T(0), omega = T(0), disturbance = T(0), reference = T(0),
    angleError = T(0);
  T energy = T(0), energyReference = T(0), relativeEnergy = T(0), membership = T(0), preview = T(0),
    terminal = T(0);
  T previousSpeed = T(0), plannedSpeed = T(0), sentSpeed = T(0), plannedAcceleration = T(0),
    sentAcceleration = T(0);
  T timingScale = T(1), decision = T(0), gradient = T(0), cost = T(0);
  T nextPhi = T(0), nextOmega = T(0), terminalPhi = T(0), terminalOmega = T(0);
  T wheelState = T(0), innovation = T(0), innovationVariance = T(0);
};
template <class T> bool format(char *text, size_t capacity, const Frame<T> &f, size_t &length) {
  standalone_telemetry::Writer w(text, capacity);
  w.text("control_diag,")
      .integer(f.sequence)
      .text(",")
      .integer(f.cycle)
      .text(",")
      .integer(f.tick)
      .text(",")
      .number(f.time)
      .text(",")
      .integer(f.phase)
      .text(",")
      .number(f.rawPhi)
      .text(",")
      .number(f.phi)
      .text(",")
      .number(f.omega)
      .text(",")
      .number(f.disturbance)
      .text(",")
      .number(f.reference)
      .text(",")
      .number(f.angleError)
      .text(",")
      .number(f.energy)
      .text(",")
      .number(f.energyReference)
      .text(",")
      .number(f.relativeEnergy)
      .text(",")
      .number(f.membership)
      .text(",")
      .integer(f.gates)
      .text(",")
      .integer(f.captureSamples)
      .text(",")
      .integer(f.captured)
      .text(",")
      .number(f.preview)
      .text(",")
      .number(f.terminal)
      .text(",")
      .number(f.previousSpeed)
      .text(",")
      .number(f.plannedSpeed)
      .text(",")
      .number(f.sentSpeed)
      .text(",")
      .number(f.plannedAcceleration)
      .text(",")
      .number(f.sentAcceleration)
      .text(",")
      .number(f.timingScale)
      .text(",")
      .number(f.decision)
      .text(",")
      .integer(f.delayUs)
      .text(",")
      .integer(f.solveUs)
      .text(",")
      .integer(f.intervalUs)
      .text(",")
      .integer(f.sampleReadUs)
      .text(",")
      .integer(f.commandAckUs)
      .text(",")
      .integer(f.exit)
      .text(",")
      .integer(f.iterations)
      .text(",")
      .number(f.gradient)
      .text(",")
      .number(f.cost)
      .text(",")
      .integer(f.derivativeEvaluations)
      .text(",")
      .integer(f.costEvaluations)
      .text(",")
      .number(f.nextPhi)
      .text(",")
      .number(f.nextOmega)
      .text(",")
      .number(f.terminalPhi)
      .text(",")
      .number(f.terminalOmega)
      .text(",")
      .integer(f.fault)
      .text(",")
      .integer(f.issued)
      .text(",")
      .integer(f.missed)
      .text(",")
      .integer(f.pulses)
      .text(",")
      .number(f.wheelState)
      .text(",")
      .number(f.innovation)
      .text(",")
      .number(f.innovationVariance)
      .text(",")
      .integer(f.converged)
      .text("\n");
  return w.finish(length);
}
// Foreground-only, fixed-memory ring. An output frame is copied into the existing
// transmit queue before this ring slot is consumed; partial UART frames cannot
// be overwritten. Failure/stop does not erase recent history.
template <class T, size_t Capacity = 16> class Recorder {
  static_assert(Capacity > 0, "Diagnostic history cannot be empty.");

public:
  bool push(Frame<T> frame) {
    const bool lost = count_ == Capacity;
    if (lost) {
      begin_ = (begin_ + 1) % Capacity;
      --count_;
    }
    frame.sequence = nextSequence_++;
    records_[write_] = frame;
    write_ = (write_ + 1) % Capacity;
    ++count_;
    if (history_ < Capacity) {
      ++history_;
    }
    return lost;
  }
  const Frame<T> *front() const { return count_ ? &records_[begin_] : nullptr; }
  void consume() {
    if (count_) {
      begin_ = (begin_ + 1) % Capacity;
      --count_;
    }
  }
  void replayHistory() {
    begin_ = (write_ + Capacity - history_) % Capacity;
    count_ = history_;
  }
  size_t pending() const { return count_; }

private:
  Frame<T> records_[Capacity]{};
  size_t begin_ = 0, write_ = 0, count_ = 0, history_ = 0;
  uint32_t nextSequence_ = 0;
};
} // namespace control_diagnostics
#endif
