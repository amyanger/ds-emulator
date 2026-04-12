# Phase 0: Scaffolding — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Establish the project foundation — CMake build with three targets, directory skeleton, header-only utility types, skeleton classes for major subsystems, and an SDL2 frontend that opens a two-screen window showing a test pattern.

**Architecture:** Three CMake targets (`ds_core` platform-free static lib, `ds_frontend` SDL2-owning static lib, `ds_emulator` final binary), plus `ds_tests` for unit tests. The NDS core has zero SDL dependency; only `src/frontend/` includes SDL headers. Class skeletons (`NDS`, `Scheduler`, `Arm9`, `Arm7`) exist as empty/no-op stubs so Phase 1 can drop in real behavior without touching the build system.

**Tech Stack:** C++20, CMake 3.20+, SDL2, clang 15+ (primary target clang on Apple Silicon macOS).

**Note on commits:** Git commit steps are included as milestones. The project CLAUDE.md says "only commit when explicitly asked" — so either (a) confirm with the user before the first commit and then proceed through the rest, or (b) skip the commit steps entirely and let the user commit in one batch at the end. Either is fine; the task verification does not depend on commits.

---

## File structure (what Phase 0 creates)

```
ds-emulator/
├── .clang-format                        # C++ style config
├── .gitignore                           # build/ roms/ saves/
├── CLAUDE.md                            # already exists
├── CMakeLists.txt                       # top-level build
├── README.md                            # brief project summary + build steps
├── cmake/
│   ├── CompilerWarnings.cmake           # -Wall -Wextra -Wpedantic -Werror
│   ├── AppleSilicon.cmake               # flag-gated arm64 specifics
│   └── Sanitizers.cmake                 # optional ASan/UBSan
├── docs/
│   └── superpowers/
│       ├── specs/2026-04-12-nds-emulator-design.md  # already exists
│       └── plans/2026-04-12-phase-0-scaffolding.md  # this file
├── include/
│   └── ds/
│       ├── common.hpp                   # fixed-width types, Cycle, LOG, bit helpers
│       ├── fixed.hpp                    # Fixed<I, F> templated fixed-point
│       └── ring_buffer.hpp              # CircularBuffer<T, N>
├── src/
│   ├── main.cpp                         # CLI parse, Frontend+NDS wiring, event loop
│   ├── nds.hpp                          # class NDS skeleton
│   ├── nds.cpp                          # class NDS skeleton impl (no-ops)
│   ├── CMakeLists.txt                   # ds_core + ds_frontend targets
│   ├── scheduler/
│   │   ├── scheduler.hpp                # Event, EventKind, Scheduler (stub)
│   │   └── scheduler.cpp                # stub impl
│   ├── cpu/
│   │   ├── cpu_core.hpp                 # abstract CpuCore
│   │   ├── dispatcher.hpp               # abstract Dispatcher
│   │   ├── arm9/
│   │   │   ├── arm9.hpp                 # class Arm9 skeleton
│   │   │   └── arm9.cpp                 # no-op run_until
│   │   └── arm7/
│   │       ├── arm7.hpp                 # class Arm7 skeleton
│   │       └── arm7.cpp                 # no-op run_until
│   └── frontend/
│       ├── frontend.hpp                 # class Frontend
│       ├── frontend.cpp                 # SDL2 window + test pattern + Esc quit
│       └── CMakeLists.txt               # defines ds_frontend target
└── tests/
    ├── CMakeLists.txt                   # defines ds_tests target
    └── unit/
        ├── fixed_test.cpp               # Fixed<I, F> tests (TDD drives fixed.hpp)
        └── ring_buffer_test.cpp         # CircularBuffer<T, N> tests (TDD)
```

**No `build/`, `roms/`, or `saves/` files in git** — only the `.gitignore`
that excludes them.

**No test for `common.hpp`** — it's just type aliases and preprocessor macros,
nothing testable beyond "does it compile." The fact that every other file
includes it is sufficient coverage.

**No tests for Scheduler / CPU skeletons** — they're no-op stubs in Phase 0.
They get tests in Phase 1 when they gain behavior.

---

## Task 0: Initialize git and create top-level files

**Files:**
- Create: `.gitignore`
- Create: `README.md`

- [ ] **Step 0a: Initialize git repo (if not already)**

```bash
cd /Users/arjunmyanger/Documents/Dev/ds-emulator
git init
git status
```

Expected: `Initialized empty Git repository in .../ds-emulator/.git/` and `git status` shows `CLAUDE.md` and `docs/` as untracked.

- [ ] **Step 0b: Create `.gitignore`**

Create `.gitignore` with exactly:

```
# Build artifacts
build/
*.o
*.a
*.so
*.dylib
cmake-build-*/
CMakeCache.txt
CMakeFiles/
CMakeScripts/
Makefile
cmake_install.cmake

# IDE
.vscode/
.idea/
.cache/
compile_commands.json
.clangd/

# macOS
.DS_Store

# Project-specific
roms/
saves/
dump_mainram.bin
dump_vram.bin
savestate_*.bin
stderr.log
```

- [ ] **Step 0c: Create `README.md`**

Create `README.md` with exactly:

```markdown
# ds-emulator

A Nintendo DS emulator written in C++20, targeting Pokemon HeartGold / SoulSilver
primarily with full compatibility goals for all Gen 4 and Gen 5 Pokemon DS titles.

First-class build support for Apple Silicon macOS. Portable to Linux and Windows.

## Build & Run

```bash
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make
./ds_emulator <rom.nds> --scale 3
```

Apple Silicon optimized release build:

```bash
mkdir -p build && cd build && cmake .. -DDS_TARGET_APPLE_SILICON=ON -DCMAKE_BUILD_TYPE=Release && make
```

## Documentation

- **Design spec:** `docs/superpowers/specs/2026-04-12-nds-emulator-design.md`
- **Implementation plans:** `docs/superpowers/plans/`
- **Project conventions:** `CLAUDE.md`

## Status

Phase 0 — Scaffolding.
```

- [ ] **Step 0d: Verify `.gitignore` works**

```bash
mkdir -p build roms saves
touch build/test.o roms/test.nds saves/test.sav
git status --short
```

Expected: `??` entries for `.gitignore`, `README.md`, `CLAUDE.md`, `docs/`, but **no** entries for `build/`, `roms/`, or `saves/`.

- [ ] **Step 0e: (Optional) Commit**

```bash
git add .gitignore README.md CLAUDE.md docs/
git commit -m "Initial scaffold: .gitignore, README, spec, CLAUDE.md"
```

---

## Task 1: Root `CMakeLists.txt` and cmake modules

**Files:**
- Create: `CMakeLists.txt`
- Create: `cmake/CompilerWarnings.cmake`
- Create: `cmake/AppleSilicon.cmake`
- Create: `cmake/Sanitizers.cmake`

- [ ] **Step 1a: Create `cmake/CompilerWarnings.cmake`**

```cmake
# Applies strict warnings to a target.
# Usage: target_enable_warnings(my_target)
function(target_enable_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wshadow
            -Wnon-virtual-dtor
            -Wold-style-cast
            -Wcast-align
            -Wunused
            -Woverloaded-virtual
            -Wconversion
            -Wsign-conversion
            -Wnull-dereference
            -Wdouble-promotion
            -Wformat=2
        )
    endif()
endfunction()
```

- [ ] **Step 1b: Create `cmake/AppleSilicon.cmake`**

```cmake
# Flag-gated: when DS_TARGET_APPLE_SILICON=ON, applies arm64-specific tuning.
# Usage: target_apple_silicon(my_target)
function(target_apple_silicon target)
    if(NOT DS_TARGET_APPLE_SILICON)
        return()
    endif()
    if(NOT APPLE)
        message(WARNING "DS_TARGET_APPLE_SILICON=ON on non-Apple platform; ignoring")
        return()
    endif()
    target_compile_options(${target} PRIVATE -mcpu=apple-m1)
    target_compile_definitions(${target} PRIVATE DS_APPLE_SILICON=1)
endfunction()
```

- [ ] **Step 1c: Create `cmake/Sanitizers.cmake`**

```cmake
# Optional: enable ASan/UBSan on a target when DS_ENABLE_SANITIZERS=ON.
# Usage: target_enable_sanitizers(my_target)
function(target_enable_sanitizers target)
    if(NOT DS_ENABLE_SANITIZERS)
        return()
    endif()
    if(MSVC)
        return()  # unsupported
    endif()
    target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target}    PRIVATE -fsanitize=address,undefined)
endfunction()
```

- [ ] **Step 1d: Create root `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.20)
project(ds_emulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile_commands.json for clangd / other tooling.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(DS_TARGET_APPLE_SILICON "Enable Apple Silicon (arm64 + NEON) optimizations" OFF)
option(DS_ENABLE_SANITIZERS    "Enable ASan + UBSan on debug builds"                OFF)
option(DS_BUILD_TESTS          "Build unit tests"                                    ON)
option(ENABLE_XRAY             "Enable in-emulator debug overlay"                    ON)

include(cmake/CompilerWarnings.cmake)
include(cmake/AppleSilicon.cmake)
include(cmake/Sanitizers.cmake)

# SDL2
find_package(SDL2 REQUIRED)

# Subdirectories
add_subdirectory(src)

if(DS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Top-level binary target wires everything together.
add_executable(ds_emulator src/main.cpp)
target_link_libraries(ds_emulator PRIVATE ds_frontend ds_core)
target_include_directories(ds_emulator PRIVATE include src)
target_enable_warnings(ds_emulator)
target_apple_silicon(ds_emulator)
target_enable_sanitizers(ds_emulator)
```

- [ ] **Step 1e: Verify CMake can parse (but not build — src/ doesn't exist yet)**

```bash
mkdir -p build && cd build && cmake ..
```

Expected: **FAIL** with `add_subdirectory given source "src" which is not an existing directory.` This confirms CMake is finding the right file and parsing it; the failure is expected at this step.

If CMake itself errors (wrong version, missing SDL2), fix before continuing.

- [ ] **Step 1f: (Optional) Commit**

```bash
git add CMakeLists.txt cmake/
git commit -m "cmake: project root + warnings/apple-silicon/sanitizers modules"
```

---

## Task 2: `.clang-format` configuration

**Files:**
- Create: `.clang-format`

- [ ] **Step 2a: Create `.clang-format`**

```yaml
---
Language: Cpp
BasedOnStyle: LLVM
IndentWidth: 4
TabWidth: 4
UseTab: Never
ColumnLimit: 100
PointerAlignment: Left
ReferenceAlignment: Left
AccessModifierOffset: -4
AllowShortFunctionsOnASingleLine: Inline
AllowShortIfStatementsOnASingleLine: Never
AllowShortLoopsOnASingleLine: false
BinPackArguments: false
BinPackParameters: false
BreakBeforeBraces: Attach
BreakConstructorInitializers: BeforeColon
NamespaceIndentation: None
SortIncludes: CaseSensitive
IncludeBlocks: Regroup
Cpp11BracedListStyle: true
FixNamespaceComments: true
SpaceAfterCStyleCast: true
SpaceAfterTemplateKeyword: true
Standard: c++20
---
```

- [ ] **Step 2b: (Optional) Commit**

```bash
git add .clang-format
git commit -m "add .clang-format: LLVM-based C++20 style, 4-space, 100 col"
```

---

## Task 3: `include/ds/common.hpp`

**Files:**
- Create: `include/ds/common.hpp`

- [ ] **Step 3a: Create `include/ds/common.hpp`**

```cpp
#pragma once

// Project-wide common types, aliases, and helpers.
// Every subsystem header includes this.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <type_traits>

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
```

- [ ] **Step 3b: Sanity-check compile**

No standalone test yet — `common.hpp` is tested by the fact that everything includes it. We'll confirm it compiles in the first target build.

- [ ] **Step 3c: (Optional) Commit**

```bash
git add include/ds/common.hpp
git commit -m "ds/common.hpp: fixed-width types, bit helpers, log macros"
```

---

## Task 4: `include/ds/fixed.hpp` — TDD

**Files:**
- Create: `include/ds/fixed.hpp`
- Test: `tests/unit/fixed_test.cpp`

This one we test-drive: the whole point of `Fixed<I, F>` is that it catches
bugs at compile and run time, so it deserves tests first.

- [ ] **Step 4a: Write failing test `fixed_test.cpp`**

```cpp
// tests/unit/fixed_test.cpp
#include "ds/fixed.hpp"

#include <cassert>
#include <cstdio>

using namespace ds;

static void test_construct_and_to_int() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(5);
    assert(a.to_int() == 5);

    Fixed<20, 12> b = Fixed<20, 12>::from_raw(4096);  // 1.0 in 20.12
    assert(b.to_int() == 1);
}

static void test_addition() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(3);
    Fixed<20, 12> b = Fixed<20, 12>::from_int(4);
    Fixed<20, 12> c = a + b;
    assert(c.to_int() == 7);
}

static void test_subtraction() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(10);
    Fixed<20, 12> b = Fixed<20, 12>::from_int(3);
    Fixed<20, 12> c = a - b;
    assert(c.to_int() == 7);
}

static void test_multiplication() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(6);
    Fixed<20, 12> b = Fixed<20, 12>::from_int(7);
    Fixed<20, 12> c = a * b;
    assert(c.to_int() == 42);
}

static void test_multiplication_fractional() {
    // 1.5 * 2.0 = 3.0
    Fixed<20, 12> a = Fixed<20, 12>::from_raw(1 << 12 | 1 << 11);  // 1.5
    Fixed<20, 12> b = Fixed<20, 12>::from_int(2);
    Fixed<20, 12> c = a * b;
    assert(c.to_int() == 3);
}

static void test_negative() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(-5);
    assert(a.to_int() == -5);
    Fixed<20, 12> b = -a;
    assert(b.to_int() == 5);
}

static void test_raw_roundtrip() {
    const int32_t raw = 0x12345;
    Fixed<20, 12> a = Fixed<20, 12>::from_raw(raw);
    assert(a.raw() == raw);
}

int main() {
    test_construct_and_to_int();
    test_addition();
    test_subtraction();
    test_multiplication();
    test_multiplication_fractional();
    test_negative();
    test_raw_roundtrip();
    std::printf("fixed_test: all passed\n");
    return 0;
}
```

- [ ] **Step 4b: Run the test to verify it fails (file doesn't exist yet)**

Cannot run yet — `ds_tests` target not created until Task 12. The test will run for the first time in Task 12b. If `include/ds/fixed.hpp` is not implemented by then, the test binary will fail to build with "file not found", which is the expected TDD failure signal.

- [ ] **Step 4c: Create `include/ds/fixed.hpp`**

```cpp
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
            return Fixed<I2, F2>::from_raw(static_cast<i32>(value) << (F2 - F));
        } else {
            return Fixed<I2, F2>::from_raw(static_cast<i32>(value) >> (F - F2));
        }
    }
};

}  // namespace ds
```

- [ ] **Step 4d: (Optional) Commit**

```bash
git add include/ds/fixed.hpp tests/unit/fixed_test.cpp
git commit -m "ds/fixed.hpp: templated Fixed<I,F> with unit tests"
```

---

## Task 5: `include/ds/ring_buffer.hpp` — TDD

**Files:**
- Create: `include/ds/ring_buffer.hpp`
- Test: `tests/unit/ring_buffer_test.cpp`

- [ ] **Step 5a: Write failing test `ring_buffer_test.cpp`**

```cpp
// tests/unit/ring_buffer_test.cpp
#include "ds/ring_buffer.hpp"

#include <cassert>
#include <cstdio>

using namespace ds;

static void test_empty_new() {
    CircularBuffer<int, 4> rb;
    assert(rb.size() == 0);
    assert(rb.empty());
    assert(!rb.full());
}

static void test_push_and_size() {
    CircularBuffer<int, 4> rb;
    rb.push(10);
    assert(rb.size() == 1);
    assert(!rb.empty());
    rb.push(20);
    rb.push(30);
    assert(rb.size() == 3);
}

static void test_pop_fifo_order() {
    CircularBuffer<int, 4> rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    assert(rb.pop() == 1);
    assert(rb.pop() == 2);
    assert(rb.pop() == 3);
    assert(rb.empty());
}

static void test_full() {
    CircularBuffer<int, 4> rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    rb.push(4);
    assert(rb.full());
    assert(rb.size() == 4);
}

static void test_wrap_around() {
    CircularBuffer<int, 4> rb;
    rb.push(1); rb.push(2); rb.push(3); rb.push(4);
    assert(rb.pop() == 1);
    assert(rb.pop() == 2);
    rb.push(5);
    rb.push(6);
    assert(rb.pop() == 3);
    assert(rb.pop() == 4);
    assert(rb.pop() == 5);
    assert(rb.pop() == 6);
    assert(rb.empty());
}

static void test_clear() {
    CircularBuffer<int, 4> rb;
    rb.push(1); rb.push(2);
    rb.clear();
    assert(rb.empty());
    assert(rb.size() == 0);
}

int main() {
    test_empty_new();
    test_push_and_size();
    test_pop_fifo_order();
    test_full();
    test_wrap_around();
    test_clear();
    std::printf("ring_buffer_test: all passed\n");
    return 0;
}
```

- [ ] **Step 5b: Create `include/ds/ring_buffer.hpp`**

```cpp
#pragma once

// Fixed-capacity single-producer single-consumer ring buffer.
// Not thread-safe on its own; used where a scheduler event produces and a
// consumer (CPU, audio callback) consumes in a single-threaded context.

#include "ds/common.hpp"

#include <array>
#include <cstddef>

namespace ds {

template <typename T, std::size_t N>
class CircularBuffer {
    static_assert(N > 0, "CircularBuffer capacity must be > 0");

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
```

- [ ] **Step 5c: (Optional) Commit**

```bash
git add include/ds/ring_buffer.hpp tests/unit/ring_buffer_test.cpp
git commit -m "ds/ring_buffer.hpp: CircularBuffer<T,N> with unit tests"
```

---

## Task 6: Scheduler skeleton

**Files:**
- Create: `src/scheduler/scheduler.hpp`
- Create: `src/scheduler/scheduler.cpp`

Phase 0 stub only — real min-heap + event kinds come in Phase 1.

- [ ] **Step 6a: Create `src/scheduler/scheduler.hpp`**

```cpp
#pragma once

#include "ds/common.hpp"

#include <cstdint>

namespace ds {

// Placeholder event kind set for Phase 0. Real enumeration lives in Phase 1.
enum class EventKind : uint32_t {
    None = 0,
    FrameEnd,
};

// Phase 0 skeleton. Only exposes `now()` and a stubbed `advance_to` that
// updates the clock without firing events.
class Scheduler {
public:
    Scheduler() = default;

    Cycle now() const { return now_; }

    // Phase 0: just bump the clock. Phase 1 will pop the heap and fire events.
    void advance_to(Cycle target) {
        if (target > now_) {
            now_ = target;
        }
    }

    // Phase 1 will implement these. For now, stubs that do nothing so callers
    // can link.
    void reset() { now_ = 0; }

private:
    Cycle now_ = 0;
};

}  // namespace ds
```

- [ ] **Step 6b: Create `src/scheduler/scheduler.cpp`**

```cpp
#include "scheduler/scheduler.hpp"

// Phase 0: header-only-sufficient stub. This .cpp exists so the target has at
// least one TU for this component; Phase 1 will move logic here.

namespace ds {
// Intentionally empty.
}  // namespace ds
```

- [ ] **Step 6c: (Optional) Commit**

```bash
git add src/scheduler/
git commit -m "scheduler: Phase 0 skeleton with now()/advance_to stub"
```

---

## Task 7: CPU core abstract headers + skeletons

**Files:**
- Create: `src/cpu/cpu_core.hpp`
- Create: `src/cpu/dispatcher.hpp`
- Create: `src/cpu/arm9/arm9.hpp`
- Create: `src/cpu/arm9/arm9.cpp`
- Create: `src/cpu/arm7/arm7.hpp`
- Create: `src/cpu/arm7/arm7.cpp`

- [ ] **Step 7a: Create `src/cpu/cpu_core.hpp`**

```cpp
#pragma once

#include "ds/common.hpp"

namespace ds {

// Abstract CPU core. Both ARM9 and ARM7 implement this.
// In Phase 0 the implementations are empty stubs; Phase 1 fills them in.
class CpuCore {
public:
    virtual ~CpuCore() = default;

    // Advance this core up to (but not past) the given ARM9 cycle target.
    // ARM7's implementation internally converts to ARM7 cycles (half rate).
    virtual void run_until(Cycle arm9_target) = 0;

    virtual void reset() = 0;
};

}  // namespace ds
```

- [ ] **Step 7b: Create `src/cpu/dispatcher.hpp`**

```cpp
#pragma once

namespace ds {

class CpuCore;

// Swappable instruction dispatcher. Phase 0 has no implementations; Phase 1
// introduces the interpreter. Phase 6+ may add cached-block or JIT dispatchers.
class Dispatcher {
public:
    virtual ~Dispatcher() = default;
    virtual void step(CpuCore& core) = 0;
};

}  // namespace ds
```

- [ ] **Step 7c: Create `src/cpu/arm9/arm9.hpp`**

```cpp
#pragma once

#include "cpu/cpu_core.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm9 : public CpuCore {
public:
    void run_until(Cycle arm9_target) override;
    void reset() override;

private:
    Cycle cycles_ = 0;  // ARM9 cycles executed
};

}  // namespace ds
```

- [ ] **Step 7d: Create `src/cpu/arm9/arm9.cpp`**

```cpp
#include "cpu/arm9/arm9.hpp"

namespace ds {

void Arm9::run_until(Cycle arm9_target) {
    // Phase 0 no-op: pretend we executed up to the target.
    if (arm9_target > cycles_) {
        cycles_ = arm9_target;
    }
}

void Arm9::reset() {
    cycles_ = 0;
}

}  // namespace ds
```

- [ ] **Step 7e: Create `src/cpu/arm7/arm7.hpp`**

```cpp
#pragma once

#include "cpu/cpu_core.hpp"
#include "ds/common.hpp"

namespace ds {

class Arm7 : public CpuCore {
public:
    // Takes an ARM9-cycle target; internally converts to ARM7 cycles (half rate).
    void run_until(Cycle arm9_target) override;
    void reset() override;

private:
    Cycle arm7_cycles_ = 0;
};

}  // namespace ds
```

- [ ] **Step 7f: Create `src/cpu/arm7/arm7.cpp`**

```cpp
#include "cpu/arm7/arm7.hpp"

namespace ds {

void Arm7::run_until(Cycle arm9_target) {
    // ARM7 runs at half the ARM9 clock rate. Phase 0 no-op.
    const Cycle arm7_target = arm9_target / 2;
    if (arm7_target > arm7_cycles_) {
        arm7_cycles_ = arm7_target;
    }
}

void Arm7::reset() {
    arm7_cycles_ = 0;
}

}  // namespace ds
```

- [ ] **Step 7g: (Optional) Commit**

```bash
git add src/cpu/
git commit -m "cpu: Phase 0 skeletons for CpuCore, Dispatcher, Arm9, Arm7"
```

---

## Task 8: `NDS` top-level class

**Files:**
- Create: `src/nds.hpp`
- Create: `src/nds.cpp`

- [ ] **Step 8a: Create `src/nds.hpp`**

```cpp
#pragma once

#include "cpu/arm7/arm7.hpp"
#include "cpu/arm9/arm9.hpp"
#include "ds/common.hpp"
#include "scheduler/scheduler.hpp"

namespace ds {

// Top-level system. Owns every subsystem. In Phase 0 it has only the
// scheduler and two CPU stubs; subsystems land in subsequent phases.
class NDS {
public:
    NDS();

    // Advance the emulator by one frame. Phase 0 just advances the clock a
    // fixed amount and returns — no CPU execution, no rendering.
    void run_frame();

    void reset();

    Scheduler& scheduler() { return scheduler_; }
    Arm9&      cpu9()      { return cpu9_; }
    Arm7&      cpu7()      { return cpu7_; }

private:
    Scheduler scheduler_;
    Arm9      cpu9_;
    Arm7      cpu7_;
    bool      frame_done_ = false;
};

}  // namespace ds
```

- [ ] **Step 8b: Create `src/nds.cpp`**

```cpp
#include "nds.hpp"

namespace ds {

// Approximate cycles per frame at 67.03 MHz / 59.82 Hz ≈ 1,120,380.
// Phase 0 only uses this as a stand-in "how far to advance the clock."
static constexpr Cycle kFrameCycles = 1'120'380;

NDS::NDS() {
    reset();
}

void NDS::reset() {
    scheduler_.reset();
    cpu9_.reset();
    cpu7_.reset();
    frame_done_ = false;
}

void NDS::run_frame() {
    // Phase 0: drive clock forward; CPUs are no-ops; scheduler has no events.
    const Cycle target = scheduler_.now() + kFrameCycles;
    cpu9_.run_until(target);
    cpu7_.run_until(target);
    scheduler_.advance_to(target);
    frame_done_ = true;
}

}  // namespace ds
```

- [ ] **Step 8c: (Optional) Commit**

```bash
git add src/nds.hpp src/nds.cpp
git commit -m "nds: Phase 0 top-level NDS class with no-op run_frame"
```

---

## Task 9: `ds_core` CMake target

**Files:**
- Create: `src/CMakeLists.txt`

- [ ] **Step 9a: Create `src/CMakeLists.txt`**

```cmake
# ds_core — platform-free static library. Zero SDL2 dependency.
# Anything under src/frontend/ is NOT in this target; see the subdir below.
add_library(ds_core STATIC
    nds.cpp
    scheduler/scheduler.cpp
    cpu/arm9/arm9.cpp
    cpu/arm7/arm7.cpp
)
target_include_directories(ds_core
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
)
target_enable_warnings(ds_core)
target_apple_silicon(ds_core)
target_enable_sanitizers(ds_core)

# ds_frontend — SDL2-owning static library. This is the ONLY target that may
# include SDL headers.
add_subdirectory(frontend)
```

- [ ] **Step 9b: Try to build `ds_core` (will fail because frontend subdir doesn't exist yet)**

```bash
cd build && cmake .. && make ds_core 2>&1 | tail -20
```

Expected: CMake error about missing `frontend/CMakeLists.txt`. That's fine — we build the frontend in the next task.

- [ ] **Step 9c: (Optional) Commit**

```bash
git add src/CMakeLists.txt
git commit -m "cmake: ds_core static library target"
```

---

## Task 10: `ds_frontend` target + SDL2 window with test pattern

**Files:**
- Create: `src/frontend/frontend.hpp`
- Create: `src/frontend/frontend.cpp`
- Create: `src/frontend/CMakeLists.txt`

- [ ] **Step 10a: Create `src/frontend/frontend.hpp`**

```cpp
#pragma once

#include "ds/common.hpp"

// Forward-declare SDL types to keep this header SDL-free for clients.
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;

namespace ds {

class NDS;

// Owns the SDL2 window, renderer, textures for both DS screens, input pump,
// and audio output. Only this translation unit (and any file under
// src/frontend/) may include SDL headers.
class Frontend {
public:
    Frontend();
    ~Frontend();

    Frontend(const Frontend&)            = delete;
    Frontend& operator=(const Frontend&) = delete;

    // Initialize SDL subsystems, create window + renderer + textures.
    // Returns false on failure.
    bool init(int scale);

    // Poll SDL events. Returns false when the user requested quit (Esc or
    // window close).
    bool pump_events();

    // Upload the two screens' framebuffers and present.
    // Phase 0: passes null pointers and uses the internal test pattern instead.
    void present(const u32* top_rgba, const u32* bot_rgba);

    // Release all SDL resources.
    void shutdown();

private:
    SDL_Window*   window_    = nullptr;
    SDL_Renderer* renderer_  = nullptr;
    SDL_Texture*  tex_top_   = nullptr;
    SDL_Texture*  tex_bot_   = nullptr;
    int           scale_     = 1;
    bool          initialized_ = false;

    // Phase 0 test pattern buffer — a simple gradient.
    u32 test_pattern_top_[192 * 256] = {};
    u32 test_pattern_bot_[192 * 256] = {};

    void fill_test_pattern();
};

}  // namespace ds
```

- [ ] **Step 10b: Create `src/frontend/frontend.cpp`**

```cpp
#include "frontend/frontend.hpp"

#include <SDL.h>

#include <cstdio>

namespace ds {

static constexpr int kScreenWidth  = 256;
static constexpr int kScreenHeight = 192;
static constexpr int kScreenGap    = 4;  // px between top and bottom

Frontend::Frontend() {
    fill_test_pattern();
}

Frontend::~Frontend() {
    shutdown();
}

void Frontend::fill_test_pattern() {
    // Top: horizontal red-green gradient with a blue grid.
    for (int y = 0; y < kScreenHeight; ++y) {
        for (int x = 0; x < kScreenWidth; ++x) {
            const u8 r = static_cast<u8>(x);
            const u8 g = static_cast<u8>(y);
            const u8 b = ((x % 32) == 0 || (y % 32) == 0) ? 0xFF : 0x20;
            test_pattern_top_[y * kScreenWidth + x] =
                0xFF000000u | (static_cast<u32>(r) << 16) | (static_cast<u32>(g) << 8)
                | static_cast<u32>(b);
        }
    }
    // Bottom: inverse gradient (cyan/yellow with magenta grid).
    for (int y = 0; y < kScreenHeight; ++y) {
        for (int x = 0; x < kScreenWidth; ++x) {
            const u8 r = static_cast<u8>(255 - x);
            const u8 g = static_cast<u8>(255 - y);
            const u8 b = ((x % 32) == 0 || (y % 32) == 0) ? 0xFF : 0xE0;
            test_pattern_bot_[y * kScreenWidth + x] =
                0xFF000000u | (static_cast<u32>(r) << 16) | (static_cast<u32>(g) << 8)
                | static_cast<u32>(b);
        }
    }
}

bool Frontend::init(int scale) {
    scale_ = scale < 1 ? 1 : scale;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    const int win_w = kScreenWidth * scale_;
    const int win_h = (kScreenHeight * 2 + kScreenGap) * scale_;

    window_ = SDL_CreateWindow(
        "ds-emulator",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        win_w, win_h,
        SDL_WINDOW_ALLOW_HIGHDPI);
    if (!window_) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return false;
    }

    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer_) {
        std::fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        return false;
    }

    tex_top_ = SDL_CreateTexture(
        renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        kScreenWidth, kScreenHeight);
    tex_bot_ = SDL_CreateTexture(
        renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        kScreenWidth, kScreenHeight);
    if (!tex_top_ || !tex_bot_) {
        std::fprintf(stderr, "SDL_CreateTexture failed: %s\n", SDL_GetError());
        return false;
    }

    initialized_ = true;
    return true;
}

bool Frontend::pump_events() {
    SDL_Event ev;
    while (SDL_PollEvent(&ev) != 0) {
        if (ev.type == SDL_QUIT) {
            return false;
        }
        if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) {
            return false;
        }
    }
    return true;
}

void Frontend::present(const u32* top_rgba, const u32* bot_rgba) {
    if (!initialized_) return;

    const u32* src_top = top_rgba ? top_rgba : test_pattern_top_;
    const u32* src_bot = bot_rgba ? bot_rgba : test_pattern_bot_;

    SDL_UpdateTexture(tex_top_, nullptr, src_top, kScreenWidth * static_cast<int>(sizeof(u32)));
    SDL_UpdateTexture(tex_bot_, nullptr, src_bot, kScreenWidth * static_cast<int>(sizeof(u32)));

    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);

    const int w = kScreenWidth  * scale_;
    const int h = kScreenHeight * scale_;

    SDL_Rect dst_top{0, 0, w, h};
    SDL_Rect dst_bot{0, (kScreenHeight + kScreenGap) * scale_, w, h};

    SDL_RenderCopy(renderer_, tex_top_, nullptr, &dst_top);
    SDL_RenderCopy(renderer_, tex_bot_, nullptr, &dst_bot);

    SDL_RenderPresent(renderer_);
}

void Frontend::shutdown() {
    if (tex_top_)  { SDL_DestroyTexture(tex_top_);  tex_top_  = nullptr; }
    if (tex_bot_)  { SDL_DestroyTexture(tex_bot_);  tex_bot_  = nullptr; }
    if (renderer_) { SDL_DestroyRenderer(renderer_); renderer_ = nullptr; }
    if (window_)   { SDL_DestroyWindow(window_);    window_   = nullptr; }
    if (initialized_) {
        SDL_Quit();
        initialized_ = false;
    }
}

}  // namespace ds
```

- [ ] **Step 10c: Create `src/frontend/CMakeLists.txt`**

```cmake
# ds_frontend — SDL2-aware static library. Platform abstractions live here.
# This is the ONLY target allowed to include SDL headers.

add_library(ds_frontend STATIC
    frontend.cpp
)
target_include_directories(ds_frontend
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(ds_frontend
    PUBLIC  ds_core
    PRIVATE SDL2::SDL2
)
target_enable_warnings(ds_frontend)
target_apple_silicon(ds_frontend)
target_enable_sanitizers(ds_frontend)
```

- [ ] **Step 10d: Verify `ds_frontend` builds**

```bash
cd build && cmake .. && make ds_frontend 2>&1 | tail -20
```

Expected: `[100%] Built target ds_frontend`. If linking against `SDL2::SDL2` fails, check the `find_package(SDL2 REQUIRED)` in root `CMakeLists.txt` and confirm SDL2 is installed (`brew list | grep sdl2` on macOS).

- [ ] **Step 10e: (Optional) Commit**

```bash
git add src/frontend/
git commit -m "frontend: SDL2 window with 256x384 test pattern, Esc to quit"
```

---

## Task 11: `main.cpp` — wire it all together

**Files:**
- Create: `src/main.cpp`

- [ ] **Step 11a: Create `src/main.cpp`**

```cpp
#include "frontend/frontend.hpp"
#include "nds.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

void print_usage(const char* argv0) {
    std::fprintf(stderr,
        "Usage: %s [<rom.nds>] [--scale N]\n"
        "\n"
        "Phase 0: ROM argument is ignored. Window shows a test pattern.\n"
        "Press Esc or close the window to quit.\n",
        argv0);
}

int parse_scale(int argc, char** argv) {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::strcmp(argv[i], "--scale") == 0) {
            return std::atoi(argv[i + 1]);
        }
    }
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    const int scale = parse_scale(argc, argv);

    ds::NDS      nds;
    ds::Frontend frontend;

    if (!frontend.init(scale)) {
        std::fprintf(stderr, "Failed to initialize frontend.\n");
        return 1;
    }

    std::fprintf(stderr, "ds-emulator Phase 0: window open, showing test pattern.\n");
    std::fprintf(stderr, "Press Esc to quit.\n");

    while (frontend.pump_events()) {
        nds.run_frame();                 // advances clock, no-op otherwise
        frontend.present(nullptr, nullptr);  // null = use test pattern
    }

    frontend.shutdown();
    std::fprintf(stderr, "Clean exit.\n");
    return 0;
}
```

- [ ] **Step 11b: Build `ds_emulator`**

```bash
cd build && cmake .. && make ds_emulator 2>&1 | tail -20
```

Expected: `[100%] Built target ds_emulator`.

- [ ] **Step 11c: Run it**

```bash
./ds_emulator
```

Expected: A window opens showing two screens stacked vertically with gradient test patterns. Press **Esc**. The process prints `Clean exit.` and returns to the shell.

If the window does not open on macOS, check the Console app for any code-signing / entitlement errors. On Apple Silicon, SDL2 installed via Homebrew should work without signing for local dev.

- [ ] **Step 11d: (Optional) Commit**

```bash
git add src/main.cpp
git commit -m "main: Phase 0 entry point wiring NDS + Frontend"
```

---

## Task 12: `ds_tests` target

**Files:**
- Create: `tests/CMakeLists.txt`

- [ ] **Step 12a: Create `tests/CMakeLists.txt`**

```cmake
# ds_tests — one binary per test source file, each links ds_core only.
# No SDL dependency, runs in milliseconds.

function(add_ds_unit_test name)
    add_executable(${name} unit/${name}.cpp)
    target_link_libraries(${name} PRIVATE ds_core)
    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
    )
    target_enable_warnings(${name})
    target_apple_silicon(${name})
    target_enable_sanitizers(${name})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_ds_unit_test(fixed_test)
add_ds_unit_test(ring_buffer_test)
```

- [ ] **Step 12b: Build and run the tests**

```bash
cd build && cmake .. && make fixed_test ring_buffer_test 2>&1 | tail -20
./fixed_test
./ring_buffer_test
```

Expected:
```
fixed_test: all passed
ring_buffer_test: all passed
```

If either test fails, fix the corresponding header (`include/ds/fixed.hpp` or `include/ds/ring_buffer.hpp`) until it passes. The test is the source of truth.

- [ ] **Step 12c: Run via ctest**

```bash
cd build && ctest --output-on-failure
```

Expected:
```
Test project .../ds-emulator/build
    Start 1: fixed_test
1/2 Test #1: fixed_test .......................   Passed    ...
    Start 2: ring_buffer_test
2/2 Test #2: ring_buffer_test .................   Passed    ...

100% tests passed, 0 tests failed out of 2
```

- [ ] **Step 12d: (Optional) Commit**

```bash
git add tests/
git commit -m "tests: ds_tests target with fixed and ring_buffer unit tests"
```

---

## Task 13: Full rebuild verification

**Files:** none (verification only)

- [ ] **Step 13a: Clean rebuild from scratch**

```bash
cd /Users/arjunmyanger/Documents/Dev/ds-emulator
rm -rf build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug 2>&1 | tail -20
make 2>&1 | tail -30
```

Expected: Clean configuration, clean build, `[100%] Built target ds_emulator`, no compiler warnings.

- [ ] **Step 13b: Run unit tests**

```bash
ctest --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 2`.

- [ ] **Step 13c: Run the emulator binary and confirm the milestone**

```bash
./ds_emulator
```

Expected: Window opens with stacked two-screen test pattern. Close with Esc. `Clean exit.`

- [ ] **Step 13d: Verify Apple Silicon build flag works (macOS arm64 only)**

```bash
cd /Users/arjunmyanger/Documents/Dev/ds-emulator
rm -rf build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DDS_TARGET_APPLE_SILICON=ON 2>&1 | tail -20
make 2>&1 | tail -20
./ds_emulator
```

Expected: Clean build with `-mcpu=apple-m1`, window opens identically. `file ./ds_emulator` confirms `Mach-O 64-bit executable arm64`.

- [ ] **Step 13e: Verify `ds_core` has no SDL dependency**

```bash
cd build
# On macOS:
otool -L libds_core.a 2>&1 | grep -i sdl || echo "OK: ds_core has no SDL linkage"
# On Linux:
# nm libds_core.a 2>&1 | grep -i SDL_ || echo "OK: ds_core has no SDL symbols"
```

Expected: `OK: ds_core has no SDL linkage`. This is the non-negotiable rule
from the spec.

- [ ] **Step 13f: Verify headless test binaries don't link SDL**

```bash
# On macOS:
otool -L fixed_test 2>&1 | grep -i sdl || echo "OK: fixed_test is SDL-free"
otool -L ring_buffer_test 2>&1 | grep -i sdl || echo "OK: ring_buffer_test is SDL-free"
```

Expected: both show `OK`.

- [ ] **Step 13g: (Optional) Final commit for Phase 0**

```bash
cd /Users/arjunmyanger/Documents/Dev/ds-emulator
git status
git log --oneline
```

If any uncommitted changes remain:

```bash
git add -A
git commit -m "Phase 0: scaffolding complete — window opens, tests green"
```

---

## Phase 0 exit criteria (milestone)

All of the following must be true before Phase 1 begins:

1. `./ds_emulator` opens a window showing a stacked two-screen test pattern.
2. Pressing Esc closes the window cleanly with `Clean exit.` printed.
3. `ctest --output-on-failure` reports 100% tests passed (2 tests).
4. `libds_core.a` has zero SDL2 symbols / linkage.
5. Clean rebuild from scratch produces zero compiler warnings at
   `-Wall -Wextra -Wpedantic -Wshadow`.
6. `DS_TARGET_APPLE_SILICON=ON` build also succeeds (on macOS arm64).

When all six pass, Phase 0 is done. Start Phase 1 brainstorming: CPUs, bus,
memory, IRQs, IPC, direct boot.
