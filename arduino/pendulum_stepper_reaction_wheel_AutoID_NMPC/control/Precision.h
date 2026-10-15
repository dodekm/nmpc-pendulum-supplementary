#ifndef ADAPTIVE_PRECISION_H
#define ADAPTIVE_PRECISION_H
#pragma once
#ifndef PENDULUM_CONTROL_PRECISION_H
#define PENDULUM_CONTROL_PRECISION_H

#include <limits>
#include <type_traits>

// One switch for all application numerical translation units. Arduino's driver
// and encoder ABIs stay float; those explicit boundaries are not a second core.
#ifndef NMPC_USE_SINGLE_PRECISION
#define NMPC_USE_SINGLE_PRECISION 1
#endif
#if NMPC_USE_SINGLE_PRECISION != 0 && NMPC_USE_SINGLE_PRECISION != 1
#error "NMPC_USE_SINGLE_PRECISION must be 0 (binary64) or 1 (binary32)."
#endif
#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ > 0)
#error "NMPC requires IEEE finite/NaN checks; fast-math and finite-math-only are unsupported."
#endif

namespace pendulum {
#if NMPC_USE_SINGLE_PRECISION
using RuntimeScalar = float;
#else
using RuntimeScalar = double;
#endif

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
