#ifndef ADAPTIVE_STANDALONETELEMETRY_H
#define ADAPTIVE_STANDALONETELEMETRY_H
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <system_error>
#include <type_traits>
namespace standalone_telemetry {
// Bounded ASCII only: no printf float support, heap, locale or Arduino runtime.
// Use the selected scalar's shortest round-trip representation. Conversion
// stays in that precision; no software-double scaling in a float build.
class Writer final {
public:
  Writer(char *buffer, size_t capacity) : buffer_(buffer), capacity_(capacity) {
    valid_ = buffer_ != nullptr && capacity_ > 0;
    if (valid_) {
      buffer_[0] = '\0';
    }
  }

  Writer &text(const char *value) {
    if (value == nullptr) {
      valid_ = false;
      return *this;
    }
    while (valid_ && *value != '\0') {
      character(*value++);
    }
    return *this;
  }

  Writer &integer(uint32_t value) {
    char digits[10];
    size_t count = 0;
    do {
      digits[count++] = char('0' + value % 10);
      value /= 10;
    } while (value);
    while (count) {
      character(digits[--count]);
    }
    return *this;
  }

  template <class Scalar> Writer &number(Scalar value) {
    static_assert(std::is_same<Scalar, float>::value || std::is_same<Scalar, double>::value,
                  "Telemetry supports binary32/binary64; use integer() for counters.");
    if (!valid_) {
      return *this;
    }
    if (std::isnan(value)) {
      return text("nan");
    }
    if (std::signbit(value)) {
      character('-');
      value = -value;
    }
    if (std::isinf(value)) {
      return text("inf");
    }
    if (value == Scalar(0)) {
      return character('0');
    }

    char scientific[32];
    const auto converted = std::to_chars(scientific, scientific + sizeof scientific, value,
                                         std::chars_format::scientific);
    if (converted.ec != std::errc{}) {
      valid_ = false;
      return *this;
    }
    char digits[17];
    int count = 0;
    const char *cursor = scientific;
    while (cursor < converted.ptr && *cursor != 'e') {
      if (*cursor != '.') {
        digits[count++] = *cursor;
      }
      ++cursor;
    }
    ++cursor; // Scientific conversion always includes an exponent.
    const bool negativeExponent = *cursor++ == '-';
    int exponent = 0;
    while (cursor < converted.ptr) {
      exponent = 10 * exponent + (*cursor++ - '0');
    }
    if (negativeExponent) {
      exponent = -exponent;
    }

    if (exponent >= 0 && exponent < 9) {
      const int point = exponent + 1;
      for (int i = 0; i < point; ++i) {
        character(i < count ? digits[i] : '0');
      }
      if (count > point) {
        character('.');
        for (int i = point; i < count; ++i) {
          character(digits[i]);
        }
      }
    } else if (exponent < 0 && exponent >= -4) {
      text("0.");
      for (int i = 0; i < -exponent - 1; ++i) {
        character('0');
      }
      for (int i = 0; i < count; ++i) {
        character(digits[i]);
      }
    } else {
      character(digits[0]);
      if (count > 1) {
        character('.');
        for (int i = 1; i < count; ++i) {
          character(digits[i]);
        }
      }
      character('e');
      character(exponent < 0 ? '-' : '+');
      integer(static_cast<uint32_t>(exponent < 0 ? -exponent : exponent));
    }
    return *this;
  }

  bool finish(size_t &length) {
    length = valid_ ? size_ : 0;
    if (!valid_ && buffer_ != nullptr && capacity_ > 0) {
      buffer_[0] = '\0';
    }
    return valid_;
  }

private:
  Writer &character(char value) {
    if (!valid_) {
      return *this;
    }
    if (size_ + 1 >= capacity_) {
      valid_ = false;
      return *this;
    }
    buffer_[size_++] = value;
    buffer_[size_] = '\0';
    return *this;
  }
  char *buffer_;
  size_t capacity_, size_ = 0;
  bool valid_ = false;
};

struct Chunk {
  const uint8_t *data;
  size_t size;
};
// Foreground-only, single-frame TX queue. Complete a partially transmitted
// record before accepting another; diagnostics must never block control.
template <size_t Capacity = 1024> class TextQueue final {
  static_assert(Capacity > 0, "TX buffer needs a terminator.");

public:
  bool textQueueAvailable() const { return sent_ == size_; }
  bool queueText(const char *text) {
    if (!textQueueAvailable()) {
      return false;
    }
    sent_ = 0;
    Writer writer(buffer_, Capacity);
    writer.text(text);
    return writer.finish(size_);
  }
  Chunk nextChunk(size_t available) const {
    const size_t count = std::min(size_ - sent_, kMaxChunkBytes);
    return {reinterpret_cast<const uint8_t *>(buffer_ + sent_), std::min(count, available)};
  }
  bool consume(size_t written) {
    if (written > size_ - sent_) {
      return false;
    }
    sent_ += written;
    return true;
  }

private:
  static constexpr size_t kMaxChunkBytes = 32;
  char buffer_[Capacity]{};
  size_t size_ = 0, sent_ = 0;
};
} // namespace standalone_telemetry
#endif
