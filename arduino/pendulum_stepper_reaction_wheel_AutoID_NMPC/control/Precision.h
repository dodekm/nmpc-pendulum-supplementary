#ifndef ADAPTIVE_PRECISION_H
#define ADAPTIVE_PRECISION_H
#pragma once
#ifndef PENDULUM_CONTROL_PRECISION_H
#define PENDULUM_CONTROL_PRECISION_H

#include <limits>
#include <type_traits>

// The embedded application uses the STM32's binary32 FPU throughout. Numerical
// kernels remain scalar templates for independent binary64 host verification.
#ifndef NMPC_USE_SINGLE_PRECISION
#define NMPC_USE_SINGLE_PRECISION 1
#endif
#if NMPC_USE_SINGLE_PRECISION != 1
#error "The deployed firmware requires NMPC_USE_SINGLE_PRECISION=1 (binary32)."
#endif
#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ > 0)
#error "NMPC requires IEEE finite/NaN checks; fast-math and finite-math-only are unsupported."
#endif

namespace pendulum {
using RuntimeScalar = float;

// Precision-specific linkage makes a sketch-local macro disagreement fail at
// link time instead of silently mixing modes across translation units.
void verifyBuildPrecision(RuntimeScalar *);

template <class Scalar> constexpr Scalar kPi = Scalar(3.141592653589793238462643383279502884);
template <class Scalar> constexpr Scalar kTwoPi = Scalar(2) * kPi<Scalar>;

static_assert(std::numeric_limits<RuntimeScalar>::is_iec559,
              "The numerical core requires IEEE-754 floating point.");
} // namespace pendulum
#endif // PENDULUM_CONTROL_PRECISION_H (Windows mixed path separators)
#endif // ADAPTIVE_PRECISION_H
