#ifndef ADAPTIVE_BUILDINFO_H
#define ADAPTIVE_BUILDINFO_H
#include <cstdint>
#ifndef ADAPTIVE_BUILD_FINGERPRINT
#define ADAPTIVE_BUILD_FINGERPRINT unfingerprinted
#endif
#define ADAPTIVE_STRING_IMPL(value) #value
#define ADAPTIVE_STRING(value) ADAPTIVE_STRING_IMPL(value)
namespace adaptive_build {
inline constexpr uint32_t diagnosticVersion = 7;
inline constexpr char candidate[] = "publication_rc4_rig_acceleration";
inline constexpr char fingerprint[] = ADAPTIVE_STRING(ADAPTIVE_BUILD_FINGERPRINT);
} // namespace adaptive_build
#undef ADAPTIVE_STRING
#undef ADAPTIVE_STRING_IMPL
#endif
