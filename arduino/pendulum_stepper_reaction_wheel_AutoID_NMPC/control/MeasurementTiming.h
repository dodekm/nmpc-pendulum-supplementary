#ifndef ADAPTIVE_MEASUREMENTTIMING_H
#define ADAPTIVE_MEASUREMENTTIMING_H
#include <cmath>
#include <cstdint>
namespace pendulum {
// Chronology of ONE observed interval. The known speed increment is applied
// at commandDelay, not at the previous encoder read. Durations are seconds.
template <class Scalar> struct MeasurementTiming {
  Scalar interval = Scalar(0), commandDelay = Scalar(0);
  bool commandOccurred = false;
  bool isValid(Scalar nominalPeriod) const {
    return std::isfinite(nominalPeriod) && nominalPeriod > Scalar(0) && std::isfinite(interval) &&
           interval >= Scalar(.5) * nominalPeriod && interval <= Scalar(1.5) * nominalPeriod &&
           std::isfinite(commandDelay) && commandDelay >= Scalar(0) && commandDelay <= interval &&
           (commandOccurred || commandDelay == Scalar(0));
  }
  // Unsigned subtraction handles clock wrap. An older acknowledgement is not
  // a new command and must not replay a previously assimilated speed increment.
  static bool fromTimestamps(uint32_t previousReadUs, uint32_t readUs, uint32_t acknowledgedUs,
                             Scalar nominalPeriod, MeasurementTiming &result) {
    const uint32_t intervalUs = readUs - previousReadUs, offsetUs = acknowledgedUs - previousReadUs;
    // On this short (<half-wrap) interval, a small positive offset beyond the
    // new read is a future acknowledgement, not a stale past command.
    if (offsetUs > intervalUs && offsetUs < 0x80000000U)
      return false;
    MeasurementTiming pending;
    pending.interval = Scalar(intervalUs) * Scalar(1e-6);
    pending.commandOccurred = offsetUs <= intervalUs;
    pending.commandDelay = pending.commandOccurred ? Scalar(offsetUs) * Scalar(1e-6) : Scalar(0);
    if (!pending.isValid(nominalPeriod))
      return false;
    result = pending;
    return true;
  }
};
} // namespace pendulum
#endif
