#pragma once

// Fixed-capacity single-producer single-consumer ring buffer.
// Not thread-safe on its own; used where a scheduler event produces and a
// consumer (CPU, audio callback) consumes in a single-threaded context.

#include "ds/common.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <type_traits>

namespace ds {

template <typename T, std::size_t N>
class CircularBuffer {
    static_assert(N > 0, "CircularBuffer capacity must be > 0");
    static_assert(std::is_trivially_copyable_v<T>,
                  "CircularBuffer<T, N> requires trivially-copyable T");

public:
    constexpr std::size_t capacity() const { return N; }
    constexpr std::size_t size() const { return count_; }
    constexpr bool        empty() const { return count_ == 0; }
    constexpr bool        full() const { return count_ == N; }

    void push(const T& value) {
        buf_[tail_] = value;
        tail_ = (tail_ + 1) % N;
        if (count_ < N) {
            ++count_;
        } else {
            head_ = (head_ + 1) % N;  // overwrite oldest
        }
    }

    T pop() {
        // Caller must ensure !empty(); popping an empty buffer is undefined.
        assert(!empty());
        T value = buf_[head_];
        head_ = (head_ + 1) % N;
        --count_;
        return value;
    }

    void clear() {
        head_ = tail_ = 0;
        count_ = 0;
    }

private:
    std::array<T, N> buf_{};
    std::size_t      head_ = 0;
    std::size_t      tail_ = 0;
    std::size_t      count_ = 0;
};

}  // namespace ds
