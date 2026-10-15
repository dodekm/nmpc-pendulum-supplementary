#ifndef ADAPTIVE_MPCPROBLEM_H
#define ADAPTIVE_MPCPROBLEM_H
#pragma once
#include "Model.h"
#include "MatrixValidation.h"
#include <array>

namespace pendulum {
template <class Scalar> struct MPCProblem {
  int activeHorizon = kHorizon;
  ModelParameters<Scalar> model;
  Matrix<Scalar, 3> initialState = Matrix<Scalar, 3>::Zero();
  Scalar disturbanceAcceleration = Scalar(0), accelerationScale = Scalar(25),
         wheelSpeedScale = Scalar(20);
  std::array<Matrix<Scalar, 3>, kHorizon> referenceState;
  std::array<Matrix<Scalar, 3, 3>, kHorizon> stateWeight;
  InputPlan<Scalar> inputWeight, energyReference, energyWeight, wheelSpeedWeight;
  MPCProblem() {
    for (int j = 0; j < kHorizon; ++j) {
      referenceState[j].setZero();
      stateWeight[j].setZero();
    }
    inputWeight.setConstant(Scalar(.001));
    energyReference.setZero();
    energyWeight.setZero();
    wheelSpeedWeight.setZero();
  }
  bool isValid() const {
    if (activeHorizon < 1 || activeHorizon > kHorizon) {
      return false;
    }
    if (!initialState.allFinite() || !std::isfinite(disturbanceAcceleration) ||
        !(accelerationScale > Scalar(0)) || !std::isfinite(accelerationScale) ||
        !(wheelSpeedScale > Scalar(0)) || !std::isfinite(wheelSpeedScale) || !model.isValid() ||
        !inputWeight.allFinite() || (inputWeight.array() <= Scalar(0)).any() ||
        !energyReference.allFinite() || !energyWeight.allFinite() ||
        (energyWeight.array() < Scalar(0)).any() || !wheelSpeedWeight.allFinite() ||
        (wheelSpeedWeight.array() < Scalar(0)).any()) {
      return false;
    }
    for (int j = 0; j < kHorizon; ++j) {
      if (!referenceState[j].allFinite() || !stateWeight[j].allFinite()) {
        return false;
      }
      // Equal consecutive matrices have exactly the same symmetry/PSD status.
      // This is a validation cache, not an assumption that weights are valid.
      if (j > 0 && (stateWeight[j].array() == stateWeight[j - 1].array()).all()) {
        continue;
      }
      // The supplied weights must be exactly symmetric: the objective gradient
      // uses Q*error, not (Q+Q')/2*error. Generated weights meet this contract.
      if (!(stateWeight[j].array() == stateWeight[j].transpose().array()).all() ||
          !isSymmetricPositiveSemidefinite(stateWeight[j])) {
        return false;
      }
    }
    return true;
  }
};
} // namespace pendulum
#endif // ADAPTIVE_MPCPROBLEM_H
