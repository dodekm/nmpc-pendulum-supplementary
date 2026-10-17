#ifndef PENDULUM_NUMERICS_COMPENSATED_SUM_H
#define PENDULUM_NUMERICS_COMPENSATED_SUM_H
#include <cmath>
#include <type_traits>
namespace pendulum::numerics {
// Neumaier summation in the SELECTED scalar precision, not hidden binary64.
// Both identification and MPC use the same ordered accumulation. Nonfinite
// inputs/overflow propagate as nonfinite; callers must reject invalid costs.
template <class T> class CompensatedSum {
  static_assert(std::is_floating_point<T>::value, "Floating-point scalar required");

public:
  void clear() { sum_ = correction_ = T(0); }
  void add(T value) {
    const T next = sum_ + value;
    correction_ +=
        std::abs(sum_) >= std::abs(value) ? (sum_ - next) + value : (value - next) + sum_;
    sum_ = next;
  }
  T value() const { return sum_ + correction_; }

private:
  T sum_ = T(0), correction_ = T(0);
};
} // namespace pendulum::numerics
#endif
