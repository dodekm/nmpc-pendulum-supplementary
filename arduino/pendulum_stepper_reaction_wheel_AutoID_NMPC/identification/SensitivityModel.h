#ifndef ADAPTIVE_SENSITIVITYMODEL_H
#define ADAPTIVE_SENSITIVITYMODEL_H
#pragma once
#include "Config.h"
namespace identification {
// Local integration state, independent of the controller/EKF storage. sw[j]
// and sq[j] are derivatives of angular velocity and angle with respect to
// parameter j. Slots [0,kShared) follow Config.h; slot kShared is this window's
// initial angular velocity, not another physical coefficient.
template <class T> struct State {
  T omega = T(0), angle = T(0);
  T sw[kSensitivity]{}, sq[kSensitivity]{}; // alpha,beta,gamma,bias,Coulomb,window initial omega
};
template <class T> State<T> derivative(const State<T> &x, const T *p) {
  State<T> d;
  const T sine = std::sin(x.angle), stiffness = p[0] * std::cos(x.angle);
  const T friction = std::tanh(x.omega / Policy<T>::frictionVelocityScale);
  const T damping = p[1] + p[4] / Policy<T>::frictionVelocityScale * (T(1) - friction * friction);
  d.omega = -p[0] * sine - p[1] * x.omega + p[3] - p[4] * friction;
  d.angle = x.omega;
  for (int j = 0; j < kSensitivity; ++j) {
    d.sw[j] = -stiffness * x.sq[j] - damping * x.sw[j];
    d.sq[j] = x.sw[j];
  }
  d.sw[0] -= sine;
  d.sw[1] -= x.omega;
  d.sw[3] += T(1);
  d.sw[4] -= friction;
  return d;
}
template <class T> State<T> shifted(const State<T> &x, const State<T> &d, T h) {
  State<T> y;
  y.omega = x.omega + h * d.omega;
  y.angle = x.angle + h * d.angle;
  for (int j = 0; j < kSensitivity; ++j) {
    y.sw[j] = x.sw[j] + h * d.sw[j];
    y.sq[j] = x.sq[j] + h * d.sq[j];
  }
  return y;
}
// Apply the acknowledged held-speed increment, then differentiate the same
// RK4 free-flow stages as the predicted trajectory. p is read-only and has
// kShared physical entries; h is in seconds and speedIncrement is in rad/s.
template <class T>
bool interval(State<T> &state, const T *p, T speedIncrement, T h = Policy<T>::period) {
  if (!p || !std::isfinite(speedIncrement) || !std::isfinite(h) || !(h > T(0)) ||
      h > Policy<T>::maxStep * T(100)) {
    return false;
  }
  state.omega -= p[2] * speedIncrement;
  state.sw[2] -= speedIncrement;
  const int pieces = int(std::ceil(h / Policy<T>::maxStep));
  if (pieces < 1 || pieces > 100) {
    return false;
  }
  const T dt = h / T(pieces), half = dt / T(2), sixth = dt / T(6);
  for (int i = 0; i < pieces; ++i) {
    const auto a = derivative(state, p), b = derivative(shifted(state, a, half), p),
               c = derivative(shifted(state, b, half), p), d = derivative(shifted(state, c, dt), p);
    state.omega += sixth * (a.omega + T(2) * b.omega + T(2) * c.omega + d.omega);
    state.angle += sixth * (a.angle + T(2) * b.angle + T(2) * c.angle + d.angle);
    for (int j = 0; j < kSensitivity; ++j) {
      state.sw[j] += sixth * (a.sw[j] + T(2) * b.sw[j] + T(2) * c.sw[j] + d.sw[j]);
      state.sq[j] += sixth * (a.sq[j] + T(2) * b.sq[j] + T(2) * c.sq[j] + d.sq[j]);
      if (!std::isfinite(state.sw[j]) || !std::isfinite(state.sq[j])) {
        return false;
      }
    }
    if (!std::isfinite(state.omega) || !std::isfinite(state.angle) ||
        std::abs(state.omega) > T(1000)) {
      return false;
    }
  }
  return true;
}
} // namespace identification
#endif // ADAPTIVE_SENSITIVITYMODEL_H
