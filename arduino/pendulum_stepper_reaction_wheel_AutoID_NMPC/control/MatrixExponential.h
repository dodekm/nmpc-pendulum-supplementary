#ifndef ADAPTIVE_MATRIXEXPONENTIAL_H
#define ADAPTIVE_MATRIXEXPONENTIAL_H
#pragma once
#include "NumericTypes.h"

namespace pendulum {
// General [13/13] Pade scaling/squaring, fixed 4x4 LU. NOT a closed-form
// pendulum-block exponential. Correct matrix 1-norm, bounded scaling.
template <class Scalar>
bool matrixExponential(const Matrix<Scalar, 4, 4> &input, Matrix<Scalar, 4, 4> &result) {
  if (!input.allFinite()) {
    return false;
  }
  Scalar norm = input.cwiseAbs().colwise().sum().maxCoeff();
  if (!std::isfinite(norm)) {
    return false;
  }
  int squarings = 0;
  if (norm > Scalar(5.371920351148152)) {
    squarings = int(std::ceil(std::log2(norm / Scalar(5.371920351148152))));
  }
  if (squarings > 30) {
    return false;
  }
  Matrix<Scalar, 4, 4> argument = input / std::ldexp(Scalar(1), squarings),
                       identity = Matrix<Scalar, 4, 4>::Identity();
  Matrix<Scalar, 4, 4> secondPower = argument * argument, fourthPower = secondPower * secondPower,
                       sixthPower = fourthPower * secondPower;
  // Normalize at compile time, then round ONCE to the selected scalar.
  // No binary64 arithmetic or coefficient-conversion loop at run time.
  static constexpr Scalar coefficients[] = {Scalar(1),
                                            Scalar(0.5),
                                            Scalar(7771770303897600. / 64764752532480000.),
                                            Scalar(1187353796428800. / 64764752532480000.),
                                            Scalar(129060195264000. / 64764752532480000.),
                                            Scalar(10559470521600. / 64764752532480000.),
                                            Scalar(670442572800. / 64764752532480000.),
                                            Scalar(33522128640. / 64764752532480000.),
                                            Scalar(1323241920. / 64764752532480000.),
                                            Scalar(40840800. / 64764752532480000.),
                                            Scalar(960960. / 64764752532480000.),
                                            Scalar(16380. / 64764752532480000.),
                                            Scalar(182. / 64764752532480000.),
                                            Scalar(1. / 64764752532480000.)};
  Matrix<Scalar, 4, 4> oddPolynomial =
      argument * (sixthPower * (coefficients[13] * sixthPower + coefficients[11] * fourthPower +
                                coefficients[9] * secondPower) +
                  coefficients[7] * sixthPower + coefficients[5] * fourthPower +
                  coefficients[3] * secondPower + coefficients[1] * identity);
  Matrix<Scalar, 4, 4> evenPolynomial =
      sixthPower * (coefficients[12] * sixthPower + coefficients[10] * fourthPower +
                    coefficients[8] * secondPower) +
      coefficients[6] * sixthPower + coefficients[4] * fourthPower + coefficients[2] * secondPower +
      coefficients[0] * identity;
  Matrix<Scalar, 4, 4> denominator = evenPolynomial - oddPolynomial;
  if (!denominator.allFinite()) {
    return false;
  }
  Eigen::PartialPivLU<Matrix<Scalar, 4, 4>> factorization(denominator);
  Scalar reciprocalCondition = factorization.rcond();
  if (!std::isfinite(reciprocalCondition) ||
      reciprocalCondition <= std::numeric_limits<Scalar>::epsilon()) {
    return false;
  }
  result = factorization.solve(Matrix<Scalar, 4, 4>(evenPolynomial + oddPolynomial));
  for (int k = 0; k < squarings; ++k) {
    result = (result * result).eval();
    if (!result.allFinite()) {
      return false;
    }
  }
  return result.allFinite();
}
} // namespace pendulum
#endif // ADAPTIVE_MATRIXEXPONENTIAL_H
