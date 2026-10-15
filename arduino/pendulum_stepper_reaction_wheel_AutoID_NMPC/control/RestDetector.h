#ifndef ADAPTIVE_RESTDETECTOR_H
#define ADAPTIVE_RESTDETECTOR_H
#include <cmath>
#include <cstdint>
namespace pendulum {
template <class T> struct RestPolicy {
  static constexpr uint32_t intervals = 20, freshnessMs = 500;
  static T angle() { return T(2 * 3.14159265358979323846 / 180); }
  static T angularSpeed() { return T(.05); }
};
// Original-branch rest evidence only. Application owns lifecycle, button and
// recovery state; do not duplicate that supervisor here.
template <class T> class RestDetector final {
public:
  void discardRest() {
    count_ = 0;
    haveSample_ = false;
  }
  void observeRest(T angle, uint32_t intervals, uint32_t now) {
    const bool near = std::isfinite(angle) && std::abs(angle) <= RestPolicy<T>::angle();
    const bool quiet = near && intervals == 1 && haveSample_ &&
                       std::abs(angle - previous_) <= RestPolicy<T>::angularSpeed() * T(.1);
    count_ = quiet ? (count_ < RestPolicy<T>::intervals + 1 ? count_ + 1 : count_)
                   : (near && intervals == 1 ? 1 : 0);
    previous_ = angle;
    haveSample_ = near && intervals == 1;
    sampleMs_ = now;
  }
  bool restReady(uint32_t now) const {
    return haveSample_ && count_ > RestPolicy<T>::intervals &&
           uint32_t(now - sampleMs_) <= RestPolicy<T>::freshnessMs;
  }
  uint32_t consecutiveRestSamples() const { return count_; }

private:
  uint32_t count_ = 0, sampleMs_ = 0;
  T previous_ = T(0);
  bool haveSample_ = false;
};
} // namespace pendulum
#endif
