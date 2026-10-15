#ifndef ADAPTIVE_RECORDKIND_H
#define ADAPTIVE_RECORDKIND_H
#include <cstdint>
namespace identification {
// Serial record IDs are stable and independent of application lifecycle IDs.
enum class RecordKind : uint8_t { Coarse = 0, Validation = 1, Guided = 2 };
} // namespace identification
#endif
