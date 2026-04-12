#pragma once

// Templated fixed-point type for the 3D pipeline.
// Fixed<I, F> stores a value with I integer bits and F fractional bits,
// backed by a 32-bit signed integer. Total width I + F must be <= 32.
//
// Use `from_int(n)` to construct from an integer, `from_raw(n)` from a raw
// scaled integer, and `raw()` / `to_int()` to read back.
//
// Arithmetic between same-format Fixed values works; mixing formats requires
// explicit conversion via `as<I2, F2>()`.

#include "ds/common.hpp"

namespace ds {

template <unsigned I, unsigned F>
struct Fixed {
    static_assert(I + F <= 32, "Fixed<I, F>: total bits must fit in int32_t");

    using storage_t = i32;
    static constexpr unsigned kFrac = F;
    static constexpr storage_t kOne = static_cast<storage_t>(1) << F;

    storage_t value = 0;

    // Factories
    static constexpr Fixed from_int(i32 n) {
        return Fixed{static_cast<storage_t>(n) << F};
    }
    static constexpr Fixed from_raw(storage_t raw) {
        return Fixed{raw};
    }

    // Accessors
    constexpr storage_t raw() const { return value; }
    constexpr i32       to_int() const { return static_cast<i32>(value >> F); }

    // Unary
    constexpr Fixed operator-() const { return Fixed{-value}; }

    // Binary — same format only.
    constexpr Fixed operator+(Fixed rhs) const { return Fixed{value + rhs.value}; }
    constexpr Fixed operator-(Fixed rhs) const { return Fixed{value - rhs.value}; }
    constexpr Fixed operator*(Fixed rhs) const {
        // (a * 2^F) * (b * 2^F) = (a * b) * 2^(2F)  =>  shift down by F.
        const i64 wide = static_cast<i64>(value) * static_cast<i64>(rhs.value);
        return Fixed{static_cast<storage_t>(wide >> F)};
    }

    constexpr bool operator==(Fixed rhs) const { return value == rhs.value; }
    constexpr bool operator!=(Fixed rhs) const { return value != rhs.value; }

    // Explicit format conversion — deliberately verbose so mixing formats is visible.
    template <unsigned I2, unsigned F2>
    constexpr Fixed<I2, F2> as() const {
        if constexpr (F2 >= F) {
            const i64 wide = static_cast<i64>(value) << (F2 - F);
            return Fixed<I2, F2>::from_raw(static_cast<i32>(wide));
        } else {
            return Fixed<I2, F2>::from_raw(static_cast<i32>(value >> (F - F2)));
        }
    }
};

}  // namespace ds
