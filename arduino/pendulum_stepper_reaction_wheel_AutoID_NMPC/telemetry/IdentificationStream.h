#ifndef ADAPTIVE_IDENTIFICATIONSTREAM_H
#define ADAPTIVE_IDENTIFICATIONSTREAM_H
#include "telemetry/StandaloneTelemetry.h"
#include <cstring>
#include <limits>
namespace identification_stream {
class Crc32 final {
public:
  void word(uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) {
      byte(uint8_t(value));
      value >>= 8;
    }
  }
  template <class T> void scalar(T value) {
    static_assert(std::numeric_limits<T>::is_iec559 && (sizeof(T) == 4 || sizeof(T) == 8),
                  "Streaming requires IEEE binary32/binary64.");
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof value);
    for (unsigned i = 0; i < sizeof value; ++i) {
      byte(uint8_t(bits));
      bits >>= 8;
    }
  }
  uint32_t value() const { return state_ ^ UINT32_MAX; }

private:
  void byte(uint8_t value) {
    state_ ^= value;
    for (unsigned bit = 0; bit < 8; ++bit) {
      state_ = (state_ >> 1) ^ ((state_ & 1U) ? 0xEDB88320U : 0U);
    }
  }
  uint32_t state_ = UINT32_MAX;
};
// Live acquisition only: eight short packets, no trajectory/timestamp arrays.
// CRC includes EVERY acquired sample, even if UART congestion drops its packet.
// A missing header/sample/trailer therefore cannot silently pass PC verification.
template <class T, unsigned Capacity = 8> class Stream final {
public:
  void begin(uint32_t generation, uint32_t attempt, uint32_t kind, uint32_t expected,
             uint32_t readUs, T angle) {
    end();
    generation_ = generation;
    attempt_ = attempt;
    kind_ = kind;
    expected_ = expected;
    points_ = 0;
    crc_ = Crc32();
    crc_.word(generation);
    crc_.word(attempt);
    crc_.word(kind);
    active_ = true;
    Packet p{};
    p.tag = Tag::Begin;
    p.intervals = expected;
    push(p);
    sample(0, readUs, angle, T(0), 0);
  }
  void sample(uint32_t index, uint32_t readUs, T angle, T heldSpeed, uint32_t ackUs) {
    if (!active_ || index != points_ || index > expected_) {
      return;
    }
    crc_.word(index);
    crc_.word(readUs);
    crc_.scalar(angle);
    crc_.scalar(heldSpeed);
    crc_.word(ackUs);
    ++points_;
    Packet p{};
    p.tag = Tag::Sample;
    p.index = index;
    p.readUs = readUs;
    p.angle = angle;
    p.speed = heldSpeed;
    p.ackUs = ackUs;
    push(p);
  }
  void end() {
    if (!active_) {
      return;
    }
    Crc32 final = crc_;
    final.word(points_ ? points_ - 1 : 0);
    Packet p{};
    p.tag = Tag::End;
    p.intervals = points_ ? points_ - 1 : 0;
    p.crc = final.value();
    push(p);
    active_ = false;
  }
  uint32_t checksum() const {
    Crc32 final = crc_;
    final.word(points_ ? points_ - 1 : 0);
    return final.value();
  }
  uint32_t dropped() const { return dropped_; }
  unsigned pending() const { return count_; }
  bool format(char *text, size_t capacity) const {
    if (!count_) {
      return false;
    }
    const Packet &p = packets_[head_];
    standalone_telemetry::Writer w(text, capacity);
    size_t n = 0;
    if (p.tag == Tag::Begin) {
      w.text("id_archive_begin,protocol=2,generation=")
          .integer(p.generation)
          .text(",attempt=")
          .integer(p.attempt)
          .text(",kind=")
          .integer(p.kind)
          .text(",intervals=")
          .integer(p.intervals)
          .text(",scalar_bytes=")
          .integer(uint32_t(sizeof(T)))
          .text("\n");
    } else if (p.tag == Tag::Sample) {
      w.text("id_archive,")
          .integer(p.generation)
          .text(",")
          .integer(p.index)
          .text(",")
          .integer(p.readUs)
          .text(",")
          .number(p.angle)
          .text(",")
          .number(p.speed)
          .text(",")
          .integer(p.ackUs)
          .text("\n");
    } else {
      w.text("id_archive_end,generation=")
          .integer(p.generation)
          .text(",intervals=")
          .integer(p.intervals)
          .text(",crc32=")
          .integer(p.crc)
          .text("\n");
    }
    return w.finish(n);
  }
  void consume() {
    if (count_) {
      head_ = (head_ + 1) % Capacity;
      --count_;
    }
  }

private:
  enum class Tag { Begin, Sample, End };
  struct Packet {
    Tag tag = Tag::Sample;
    uint32_t generation = 0, attempt = 0, kind = 0, index = 0, readUs = 0, ackUs = 0, intervals = 0,
             crc = 0;
    T angle = T(0), speed = T(0);
  };
  void push(Packet p) {
    if (count_ == Capacity) {
      if (dropped_ < UINT32_MAX) {
        ++dropped_;
      }
      return;
    }
    p.generation = generation_;
    p.attempt = attempt_;
    p.kind = kind_;
    packets_[(head_ + count_) % Capacity] = p;
    ++count_;
  }
  static_assert(Capacity > 1, "Streaming queue needs at least a header and initial sample.");
  Packet packets_[Capacity]{};
  unsigned head_ = 0, count_ = 0;
  uint32_t generation_ = 0, attempt_ = 0, kind_ = 0, expected_ = 0, points_ = 0, dropped_ = 0;
  Crc32 crc_;
  bool active_ = false;
};
static_assert(sizeof(Stream<float>) < 512, "Identification streaming RAM budget exceeded.");
} // namespace identification_stream
#endif
