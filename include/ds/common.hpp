#pragma once

// Project-wide common types, aliases, and helpers.
// Every subsystem header includes this.

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace ds {

// Fixed-width types — use these, never `int`/`unsigned` for hardware state.
using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using i8  = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;

// Master clock type — ARM9 cycles since power-on.
using Cycle = u64;

// Bit helpers.
template <typename T>
constexpr T bit(unsigned n) {
    return static_cast<T>(T{1} << n);
}

template <typename T>
constexpr T bits(T value, unsigned hi, unsigned lo) {
    // Extract bits [hi:lo] inclusive.
    const unsigned width = hi - lo + 1;
    const T mask = (width >= sizeof(T) * 8)
                       ? static_cast<T>(~T{0})
                       : static_cast<T>((T{1} << width) - 1);
    return static_cast<T>((value >> lo) & mask);
}

template <typename T>
constexpr bool bit_set(T value, unsigned n) {
    return ((value >> n) & T{1}) != 0;
}

// Logging — cheap stderr macros. Phase 0 keeps it minimal; full logging in Phase 1.
#define DS_LOG(level, fmt, ...)                                                          \
    std::fprintf(stderr, "[" level "] " fmt "\n", ##__VA_ARGS__)

#define DS_LOG_DEBUG(fmt, ...) DS_LOG("DEBUG", fmt, ##__VA_ARGS__)
#define DS_LOG_INFO(fmt, ...)  DS_LOG("INFO",  fmt, ##__VA_ARGS__)
#define DS_LOG_WARN(fmt, ...)  DS_LOG("WARN",  fmt, ##__VA_ARGS__)
#define DS_LOG_ERROR(fmt, ...) DS_LOG("ERROR", fmt, ##__VA_ARGS__)

// Compile-time assertion helper with a readable name.
#define DS_STATIC_ASSERT(cond, msg) static_assert((cond), msg)

}  // namespace ds
