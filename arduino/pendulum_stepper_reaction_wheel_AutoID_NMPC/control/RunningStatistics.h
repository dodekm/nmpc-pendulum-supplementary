#ifndef ADAPTIVE_RUNNINGSTATISTICS_H
#define ADAPTIVE_RUNNINGSTATISTICS_H
#pragma once
#include <cmath>
#include <cstdint>

namespace pendulum {
// Welford's recurrence avoids cancellation in a nonzero encoder-origin mean.
template <class Scalar> class RunningStatistics final {
public:
  bool add(Scalar value) {
    if (!std::isfinite(value) || count_ == UINT32_MAX) {
      return false;
    }
    const Scalar difference = value - mean_;
    const Scalar nextMean = mean_ + difference / Scalar(count_ + 1);
    const Scalar nextSum = sumSquaredDeviations_ + difference * (value - nextMean);
    if (!std::isfinite(nextMean) || !std::isfinite(nextSum)) {
      return false;
    }
    mean_ = nextMean;
    sumSquaredDeviations_ = nextSum;
    ++count_;
    return true;
  }
  Scalar mean() const { return mean_; }
  uint32_t count() const { return count_; }
  Scalar sampleVariance() const {
    return count_ > 1 ? std::fmax(Scalar(0), sumSquaredDeviations_ / Scalar(count_ - 1))
                      : Scalar(0);
  }

private:
  Scalar mean_ = Scalar(0), sumSquaredDeviations_ = Scalar(0);
  uint32_t count_ = 0;
};
} // namespace pendulum
#endif // ADAPTIVE_RUNNINGSTATISTICS_H
