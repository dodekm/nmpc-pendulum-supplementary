#ifndef ADAPTIVE_SOLVERWORKBUDGET_H
#define ADAPTIVE_SOLVERWORKBUDGET_H
namespace pendulum {
// Optional injected clock/budget check. No hardware dependency, heap or virtual
// dispatch. The caller owns context for the duration of the synchronous solve.
struct SolverWorkBudget {
  bool (*expired)(const void *) = nullptr;
  const void *context = nullptr;
  bool exhausted() const { return expired && expired(context); }
};
} // namespace pendulum
#endif
