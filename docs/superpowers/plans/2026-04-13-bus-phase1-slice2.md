# Bus + memory (Phase 1, slice 2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the NDS real memory and two bus views — `Arm9Bus` and `Arm7Bus` — that the CPU cores will read/write against in later slices. Main RAM, shared WRAM with full `WRAMCNT` banking, and ARM7 WRAM are all testable without the CPUs executing a single instruction.

**Architecture:** `class NDS` owns three flat byte arrays (main RAM, shared WRAM, ARM7 WRAM) and a `WramControl` helper holding the `WRAMCNT` byte. `Arm9Bus` and `Arm7Bus` each own a 256-entry `PageTable` keyed on the top 8 address bits, per spec §3.4. Fast-path reads/writes look up `ptr + (addr & mask)` with a single memcpy; unmapped regions and I/O fall through to a `slow_read/write` dispatcher. The only I/O register wired up in this slice is `WRAMCNT` (`0x04000247`) on ARM9 — writing it routes through `NDS`, updates `WramControl`, and asks both buses to rebuild the entries that depend on the new banking. All cross-subsystem calls go through `NDS&`, preserving the tree invariant from CLAUDE.md rule #3.

**Tech Stack:** C++20, `std::array` for backing storage, `std::memcpy` for aligned loads/stores (portable, no UB), no new third-party deps. Unit tests link `ds_core` only via the existing `add_ds_unit_test()` CMake helper.

**Parent spec:** `docs/superpowers/specs/2026-04-12-nds-emulator-design.md` §3.4, §4.1, §4.4

---

## Context for the engineer

- **Read CLAUDE.md first.** The "Architecture Rules (non-negotiable)" and "Code Style" sections are binding. Critical rules for this slice:
  - Rule 1: the scheduler is the clock. (Not relevant today, but don't introduce ad-hoc time loops.)
  - Rule 2: two bus domains, not one. ARM9 and ARM7 each get their own `Bus` class with its own page table. Do not share a table.
  - Rule 3: no subsystem points back at another subsystem. Buses hold `NDS&` (the container), not pointers to peers like `Arm9` or `Arm7`.
  - Rule 4: I/O access is routed through the bus it was issued from. `Arm9Bus` writing `WRAMCNT` ≠ `Arm7Bus` writing `WRAMCNT`. In fact, ARM7 cannot write `WRAMCNT` at all — that's ARM9-only. The ARM7 bus must ignore writes at that address.
  - Rule 5: every subsystem implements `reset()`, `save_state()`, `load_state()` from day one. Save-state serialization is deferred to slice 5+ but `reset()` is mandatory here.
  - Rule 6: NDS core has no `<SDL.h>` in its include graph.
  - 4-space indent, pointer alignment left (`u8* p`), fixed-width types from `ds/common.hpp` (`u8`/`u16`/`u32`/`u64`), no `int`/`unsigned` for hardware state.
  - Compiler warnings treated as errors in spirit (`-Wall -Wextra -Wpedantic -Werror`).
- **Read the parent spec §3.4, §4.1, §4.4.** They describe the target architecture. This slice is a *simplification* of §3.4 — a single page table per bus (not one per access size) and no waitstate table yet. Task 7 amends the spec to match.
- **Build from `build/`**, not the repo root:
  ```bash
  cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make
  ```
- **Do not commit** `CLAUDE.md` — it is gitignored as local-only context. Check `git status` before every commit.
- **Do not add Claude (or any AI) as a co-author** on any commit. No `Co-Authored-By` lines, no `--author="Claude..."`.
- **Do not amend previous commits.** When a step fails, fix and create a *new* commit.
- **Alignment:** loads/stores use `std::memcpy` into a local `u16`/`u32` rather than `reinterpret_cast<u32*>`, which is UB for misaligned pointers per the C++ standard. This compiles to a single load/store on ARM64 and x86-64. Do not "optimize" it back to a cast.
- **CPU cores stay Phase-0 stubs for this slice.** `Arm9::run_until` and `Arm7::run_until` are still no-ops that just advance cycle counters. Do not wire the buses into the CPUs yet — that is slice 3 (ARM7 core) or slice 4 (ARM9 core). Tests call `nds.arm9_bus().read32(...)` directly.

---

## File structure

### Files created

| Path | Responsibility |
|---|---|
| `src/bus/page_table.hpp` | `PageEntry` struct and `PageTable` alias (256-entry `std::array<PageEntry, 256>`). Pure data, no behavior. |
| `src/bus/wram_control.hpp` | `class WramControl` — tiny wrapper around the `WRAMCNT` byte. `reset()`, `value()`, `write()`. |
| `src/bus/arm9_bus.hpp` | `class Arm9Bus` — ARM9's view of memory. |
| `src/bus/arm9_bus.cpp` | Page table build, fast-path read/write, slow-path I/O dispatch, WRAMCNT rebuild. |
| `src/bus/arm7_bus.hpp` | `class Arm7Bus` — ARM7's view of memory. |
| `src/bus/arm7_bus.cpp` | Same shape as ARM9's but with ARM7 WRAM and different shared WRAM mapping. |
| `tests/unit/page_table_test.cpp` | Unit tests for `PageEntry`/`PageTable` fast-path lookup and masking. |
| `tests/unit/arm9_bus_test.cpp` | Unit tests for `Arm9Bus` main RAM, I/O stub, open-bus fallback. |
| `tests/unit/arm7_bus_test.cpp` | Unit tests for `Arm7Bus` ARM7 WRAM and shared WRAM region slow path. |
| `tests/unit/wramcnt_test.cpp` | Unit tests for all four `WRAMCNT` modes, both bus views, rebuild round-trip. |

### Files modified

| Path | Reason |
|---|---|
| `src/CMakeLists.txt` | Add `bus/arm9_bus.cpp`, `bus/arm7_bus.cpp` to `ds_core`. |
| `tests/CMakeLists.txt` | Register `page_table_test`, `arm9_bus_test`, `arm7_bus_test`, `wramcnt_test`. |
| `src/nds.hpp` | Add `main_ram_`, `shared_wram_`, `arm7_wram_` arrays, `wram_ctl_`, `arm9_bus_`, `arm7_bus_`. Add accessors and I/O routing methods. |
| `src/nds.cpp` | Construct/reset the new members. Implement `arm9_io_write8/16/32`, `arm9_io_read8/16/32` stubs that only handle `WRAMCNT`. |
| `docs/superpowers/specs/2026-04-12-nds-emulator-design.md` | Amend §3.4 and §4.1 to match the implemented shape (single page table per bus, slow-path fallback, `WRAMCNT` routed through `NDS`). |

---

## Task 1: Raw memory storage on NDS

Before we have a bus, we need memory to route to. Add three `std::array`s to `NDS` — main RAM (4 MB), shared WRAM (32 KB), ARM7 WRAM (64 KB) — and zero them on reset. No bus yet; this task is plumbing only.

**Files:**
- Modify: `src/nds.hpp`
- Modify: `src/nds.cpp`

- [ ] **Step 1: Add memory arrays to `NDS`.**

Edit `src/nds.hpp`. Inside `class NDS`, below the existing private members, add:

```cpp
private:
    // Central dispatch for scheduler events — the only place EventKind values
    // map to behavior. NDS owns this switch (scheduler is a pure data structure).
    void on_scheduler_event(const Event& ev);

    Scheduler scheduler_;
    Arm9      cpu9_;
    Arm7      cpu7_;
    bool      frame_done_ = false;

    // Physical memory owned by NDS. Sizes match real hardware; see design
    // spec §4.1. Bus classes (slice 2) hold raw pointers into these arrays.
    static constexpr std::size_t kMainRamBytes    = 4 * 1024 * 1024;  // 4 MB
    static constexpr std::size_t kSharedWramBytes = 32 * 1024;        // 32 KB
    static constexpr std::size_t kArm7WramBytes   = 64 * 1024;        // 64 KB

    std::array<u8, kMainRamBytes>    main_ram_{};
    std::array<u8, kSharedWramBytes> shared_wram_{};
    std::array<u8, kArm7WramBytes>   arm7_wram_{};
```

Also add `#include <array>` and `#include <cstddef>` at the top of the file if they are not already pulled in transitively.

- [ ] **Step 2: Zero the memory on reset.**

Edit `src/nds.cpp`. Update `NDS::reset()`:

```cpp
void NDS::reset() {
    scheduler_.reset();
    cpu9_.reset();
    cpu7_.reset();
    frame_done_ = false;

    main_ram_.fill(0);
    shared_wram_.fill(0);
    arm7_wram_.fill(0);
}
```

Real DS power-on leaves RAM in an undefined state; direct boot writes over the interesting parts before the game runs. Zero-fill is the deterministic choice for tests and save states.

- [ ] **Step 3: Build to verify the struct compiles.**

Run:
```bash
cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make
```
Expected: clean build, no warnings. `ds_core` and `ds_emulator` both re-link.

- [ ] **Step 4: Run existing tests to confirm no regressions.**

Run:
```bash
cd build && ctest --output-on-failure
```
Expected: `3/3 tests passed` (the scheduler slice's `fixed_test`, `ring_buffer_test`, `scheduler_test`).

- [ ] **Step 5: Commit.**

```bash
git add src/nds.hpp src/nds.cpp
git commit -m "nds: add main RAM, shared WRAM, ARM7 WRAM backing arrays

Phase 1 slice 2 step 1: flat byte storage owned by NDS, zeroed on
reset. No bus wiring yet; this is the substrate slice 2's Arm9Bus
and Arm7Bus will hold references into."
```

---

## Task 2: `PageTable` and `WramControl` types

Introduce the two tiny value types that both buses depend on. No bus class yet — this task lands the headers and a unit test for `PageTable` lookup/masking. `WramControl` is a plain wrapper around the `WRAMCNT` byte with no behavior beyond `reset()`/`value()`/`write()`.

**Files:**
- Create: `src/bus/page_table.hpp`
- Create: `src/bus/wram_control.hpp`
- Create: `tests/unit/page_table_test.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write `src/bus/page_table.hpp`.**

```cpp
#pragma once

#include "ds/common.hpp"

#include <array>

namespace ds {

// A single 16-MB-region entry in a 256-entry page table. When `ptr` is
// non-null, `addr & mask` is the byte offset into the region's backing
// store. When `ptr` is null, the access falls through to the owning bus's
// slow-path dispatcher (I/O, open bus, sub-region split).
struct PageEntry {
    u8* ptr  = nullptr;
    u32 mask = 0;
};

// 256 entries keyed on the top 8 bits of a 32-bit physical address. One
// entry covers 16 MB of address space. Regions smaller than 16 MB are
// handled by `mask` (e.g., 4 MB main RAM mirrored four times across 16 MB
// uses mask = 0x003F'FFFF).
//
// Per design spec §3.4 the final implementation will split the table by
// access size (read8/16/32, write8/16/32) to support 8-bit-write promotion
// on palette / OAM / VRAM. This slice uses a single shared table because
// those regions don't exist yet. See task 7 for the spec amendment.
struct PageTable {
    std::array<PageEntry, 256> read{};
    std::array<PageEntry, 256> write{};

    void clear() {
        read.fill({});
        write.fill({});
    }
};

}  // namespace ds
```

- [ ] **Step 2: Write `src/bus/wram_control.hpp`.**

```cpp
#pragma once

#include "ds/common.hpp"

namespace ds {

// Wrapper around the WRAMCNT I/O byte at 0x0400'0247. Only the bottom two
// bits are meaningful; the upper six are reserved and read back as zero.
// Writes come from `Arm9Bus::slow_write` via `NDS::arm9_io_write8` — the
// ARM7 bus cannot modify this register (rule 4: I/O routed through the
// bus it was issued from; WRAMCNT is ARM9-only).
class WramControl {
public:
    void reset() { value_ = 0; }

    u8 value() const { return value_; }

    void write(u8 raw) { value_ = raw & 0x3; }

private:
    u8 value_ = 0;
};

}  // namespace ds
```

- [ ] **Step 3: Write `tests/unit/page_table_test.cpp`.**

```cpp
// Unit tests for PageTable / PageEntry. Exercises the fast-path shape
// without any bus class wrapping it, so failures localize here before
// Arm9Bus / Arm7Bus add their own behavior on top.

#include "bus/page_table.hpp"
#include "require.hpp"

#include <array>

using namespace ds;

static void default_constructed_is_all_unmapped() {
    PageTable table;
    for (const auto& e : table.read) {
        REQUIRE(e.ptr == nullptr);
        REQUIRE(e.mask == 0);
    }
    for (const auto& e : table.write) {
        REQUIRE(e.ptr == nullptr);
        REQUIRE(e.mask == 0);
    }
}

static void clear_resets_all_entries() {
    PageTable table;
    std::array<u8, 16> storage{};
    table.read [0x02] = PageEntry{storage.data(), 0x0000'000F};
    table.write[0x02] = PageEntry{storage.data(), 0x0000'000F};

    table.clear();

    REQUIRE(table.read [0x02].ptr == nullptr);
    REQUIRE(table.write[0x02].ptr == nullptr);
}

static void mask_mirrors_short_storage_across_the_region() {
    // 4-byte backing store, 16 MB page. Mask = 0x3 gives a 4-byte mirror.
    std::array<u8, 4> storage{0xAA, 0xBB, 0xCC, 0xDD};
    PageTable table;
    table.read[0x02] = PageEntry{storage.data(), 0x0000'0003};

    const PageEntry& e = table.read[0x02];

    // Mirror 0: 0x02000000 -> storage[0]
    REQUIRE(e.ptr[(0x0200'0000u) & e.mask] == 0xAA);
    // Mirror 1: 0x02000004 -> storage[0]
    REQUIRE(e.ptr[(0x0200'0004u) & e.mask] == 0xAA);
    // Mirror offset: 0x0200_0002 -> storage[2]
    REQUIRE(e.ptr[(0x0200'0002u) & e.mask] == 0xCC);
    // Far end: 0x02FF_FFFF -> storage[3]
    REQUIRE(e.ptr[(0x02FF'FFFFu) & e.mask] == 0xDD);
}

int main() {
    default_constructed_is_all_unmapped();
    clear_resets_all_entries();
    mask_mirrors_short_storage_across_the_region();
    return 0;
}
```

Note: `require.hpp` is the Release-safe `REQUIRE` macro from slice 1, found in `tests/support/require.hpp`. The existing `tests/CMakeLists.txt` already adds `tests/support` to every test binary's include path, so a bare `#include "require.hpp"` works.

- [ ] **Step 4: Register the test in `tests/CMakeLists.txt`.**

Add to the bottom:
```cmake
add_ds_unit_test(page_table_test)
```

- [ ] **Step 5: Build and run the new test.**

```bash
cd build && cmake .. && make && ctest --output-on-failure
```
Expected: `4/4 tests passed`. If `page_table_test` fails, re-read the expected values — the mask is `0x3` meaning only the bottom 2 bits of the address index the storage.

- [ ] **Step 6: Commit.**

```bash
git add src/bus/page_table.hpp src/bus/wram_control.hpp \
        tests/unit/page_table_test.cpp tests/CMakeLists.txt
git commit -m "bus: PageTable and WramControl value types

Phase 1 slice 2 step 2: introduce the 256-entry page-table shape
that Arm9Bus and Arm7Bus will share, plus a tiny WramControl wrapper
around the WRAMCNT byte. Unit test covers default-constructed and
mirrored-mask lookups. No bus class yet."
```

---

## Task 3: `Arm9Bus` — main RAM fast path + slow-path stub

First real bus class. This task lands `Arm9Bus` with:
- A constructor taking `NDS&` (for I/O routing) and raw pointers to `main_ram_` and `shared_wram_`.
- A `reset()` that rebuilds the initial page table (WRAMCNT=0 means ARM9 sees all 32 KB of shared WRAM).
- `read8/16/32` + `write8/16/32` using fast path via the page table, with `std::memcpy` loads/stores.
- A `slow_read`/`slow_write` fallback for unmapped addresses and I/O. In this task the fallback routes I/O to `NDS` (stubs return 0 / ignore) and returns 0 for anything else.

No WRAMCNT rebuild logic yet — that lands in task 5. For this task the shared-WRAM entry is built once at `reset()` time in mode 0.

**Files:**
- Create: `src/bus/arm9_bus.hpp`
- Create: `src/bus/arm9_bus.cpp`
- Create: `tests/unit/arm9_bus_test.cpp`
- Modify: `src/CMakeLists.txt`
- Modify: `src/nds.hpp`
- Modify: `src/nds.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write `src/bus/arm9_bus.hpp`.**

```cpp
#pragma once

#include "bus/page_table.hpp"
#include "ds/common.hpp"

namespace ds {

class NDS;
class WramControl;

// ARM9's view of memory. Owns a 256-entry page table keyed on the top 8
// bits of the address. Fast-path reads/writes do a single lookup + memcpy.
// Unmapped addresses and I/O regions fall through to the owning NDS's I/O
// dispatcher (which in this slice only handles WRAMCNT at 0x0400'0247).
class Arm9Bus {
public:
    Arm9Bus(NDS& nds, u8* main_ram, u8* shared_wram, const WramControl& wram_ctl);

    void reset();

    u32 read32(u32 addr);
    u16 read16(u32 addr);
    u8  read8 (u32 addr);

    void write32(u32 addr, u32 value);
    void write16(u32 addr, u16 value);
    void write8 (u32 addr, u8  value);

    // Called by NDS after a WRAMCNT write to rebuild only the 0x03 entry.
    void rebuild_shared_wram();

private:
    u32  slow_read32 (u32 addr);
    u16  slow_read16 (u32 addr);
    u8   slow_read8  (u32 addr);
    void slow_write32(u32 addr, u32 value);
    void slow_write16(u32 addr, u16 value);
    void slow_write8 (u32 addr, u8  value);

    NDS*               nds_         = nullptr;
    u8*                main_ram_    = nullptr;
    u8*                shared_wram_ = nullptr;
    const WramControl* wram_ctl_    = nullptr;
    PageTable          table_{};
};

}  // namespace ds
```

- [ ] **Step 2: Write `src/bus/arm9_bus.cpp`.**

```cpp
#include "bus/arm9_bus.hpp"

#include "bus/wram_control.hpp"
#include "nds.hpp"

#include <cstring>

namespace ds {

namespace {
// Region constants (top 8 bits of the address).
constexpr u8 kRegionMainRam   = 0x02;
constexpr u8 kRegionSharedWram = 0x03;

// Main RAM: 4 MB mirrored across its 16 MB region.
constexpr u32 kMainRamMask = 0x003F'FFFFu;
}  // namespace

Arm9Bus::Arm9Bus(NDS& nds, u8* main_ram, u8* shared_wram, const WramControl& wram_ctl)
    : nds_(&nds), main_ram_(main_ram), shared_wram_(shared_wram), wram_ctl_(&wram_ctl) {}

void Arm9Bus::reset() {
    table_.clear();

    // Main RAM is fast-path readable and writable across 0x02xx'xxxx.
    table_.read [kRegionMainRam] = PageEntry{main_ram_, kMainRamMask};
    table_.write[kRegionMainRam] = PageEntry{main_ram_, kMainRamMask};

    // Shared WRAM entry depends on WRAMCNT; rebuild once here.
    rebuild_shared_wram();
}

void Arm9Bus::rebuild_shared_wram() {
    // WRAMCNT from ARM9's point of view:
    //   0: 32 KB mapped, mirrored across 0x03xx'xxxx
    //   1: low  16 KB mapped, mirrored across 0x03xx'xxxx
    //   2: high 16 KB mapped, mirrored across 0x03xx'xxxx
    //   3: unmapped — reads return 0 (open bus), writes ignored
    const u8 mode = wram_ctl_->value();
    PageEntry entry{};
    switch (mode) {
        case 0: entry = PageEntry{shared_wram_,              0x0000'7FFFu}; break;
        case 1: entry = PageEntry{shared_wram_,              0x0000'3FFFu}; break;
        case 2: entry = PageEntry{shared_wram_ + 0x4000u,    0x0000'3FFFu}; break;
        case 3: entry = PageEntry{nullptr,                   0};            break;
    }
    table_.read [kRegionSharedWram] = entry;
    table_.write[kRegionSharedWram] = entry;
}

u32 Arm9Bus::read32(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        u32 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~3u), sizeof(u32));
        return value;
    }
    return slow_read32(addr);
}

u16 Arm9Bus::read16(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        u16 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~1u), sizeof(u16));
        return value;
    }
    return slow_read16(addr);
}

u8 Arm9Bus::read8(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        return e.ptr[addr & e.mask];
    }
    return slow_read8(addr);
}

void Arm9Bus::write32(u32 addr, u32 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        std::memcpy(e.ptr + ((addr & e.mask) & ~3u), &value, sizeof(u32));
        return;
    }
    slow_write32(addr, value);
}

void Arm9Bus::write16(u32 addr, u16 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        std::memcpy(e.ptr + ((addr & e.mask) & ~1u), &value, sizeof(u16));
        return;
    }
    slow_write16(addr, value);
}

void Arm9Bus::write8(u32 addr, u8 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        e.ptr[addr & e.mask] = value;
        return;
    }
    slow_write8(addr, value);
}

// --- slow path ------------------------------------------------------------

u32 Arm9Bus::slow_read32(u32 addr) {
    if ((addr >> 24) == 0x04) return nds_->arm9_io_read32(addr);
    return 0;  // open bus
}

u16 Arm9Bus::slow_read16(u32 addr) {
    if ((addr >> 24) == 0x04) return nds_->arm9_io_read16(addr);
    return 0;
}

u8 Arm9Bus::slow_read8(u32 addr) {
    if ((addr >> 24) == 0x04) return nds_->arm9_io_read8(addr);
    return 0;
}

void Arm9Bus::slow_write32(u32 addr, u32 value) {
    if ((addr >> 24) == 0x04) nds_->arm9_io_write32(addr, value);
    // else ignore (open bus write)
}

void Arm9Bus::slow_write16(u32 addr, u16 value) {
    if ((addr >> 24) == 0x04) nds_->arm9_io_write16(addr, value);
}

void Arm9Bus::slow_write8(u32 addr, u8 value) {
    if ((addr >> 24) == 0x04) nds_->arm9_io_write8(addr, value);
}

}  // namespace ds
```

- [ ] **Step 3: Add `Arm9Bus` member + I/O stubs to `NDS`.**

Edit `src/nds.hpp`. Add includes near the top:
```cpp
#include "bus/arm9_bus.hpp"
#include "bus/wram_control.hpp"
```

Add accessor + I/O methods in the public section, below `cpu7()`:
```cpp
    Arm9Bus& arm9_bus() { return arm9_bus_; }

    // ARM9 I/O dispatch. This slice only handles WRAMCNT (0x0400'0247);
    // everything else reads as 0 and ignores writes. Task 5 wires WRAMCNT.
    u32  arm9_io_read32 (u32 addr);
    u16  arm9_io_read16 (u32 addr);
    u8   arm9_io_read8  (u32 addr);
    void arm9_io_write32(u32 addr, u32 value);
    void arm9_io_write16(u32 addr, u16 value);
    void arm9_io_write8 (u32 addr, u8  value);
```

Add private members, below `arm7_wram_`:
```cpp
    WramControl wram_ctl_{};
    Arm9Bus     arm9_bus_;
```

- [ ] **Step 4: Initialize and reset `arm9_bus_` in `NDS`.**

Edit `src/nds.cpp`. Update the constructor (member-init list can't call `reset()` in the middle, so construct with raw pointers then call `reset()`):

```cpp
NDS::NDS()
    : arm9_bus_(*this, main_ram_.data(), shared_wram_.data(), wram_ctl_) {
    reset();
}
```

Update `reset()`:
```cpp
void NDS::reset() {
    scheduler_.reset();
    cpu9_.reset();
    cpu7_.reset();
    frame_done_ = false;

    main_ram_.fill(0);
    shared_wram_.fill(0);
    arm7_wram_.fill(0);

    wram_ctl_.reset();
    arm9_bus_.reset();
}
```

Add I/O stub definitions at the bottom of `nds.cpp`, above the closing namespace brace:
```cpp
u32 NDS::arm9_io_read32 (u32 /*addr*/) { return 0; }
u16 NDS::arm9_io_read16 (u32 /*addr*/) { return 0; }
u8  NDS::arm9_io_read8  (u32 /*addr*/) { return 0; }
void NDS::arm9_io_write32(u32 /*addr*/, u32 /*value*/) {}
void NDS::arm9_io_write16(u32 /*addr*/, u16 /*value*/) {}
void NDS::arm9_io_write8 (u32 /*addr*/, u8  /*value*/) {}
```

Task 5 will replace the `write8` stub with the real WRAMCNT handler.

- [ ] **Step 5: Add `arm9_bus.cpp` to `src/CMakeLists.txt`.**

Edit `src/CMakeLists.txt`. In the `add_library(ds_core STATIC ...)` list, add:
```cmake
    bus/arm9_bus.cpp
```

- [ ] **Step 6: Write `tests/unit/arm9_bus_test.cpp`.**

```cpp
// Unit tests for Arm9Bus fast-path + slow-path behavior. Uses a live NDS
// because Arm9Bus holds a back-reference for I/O routing — there's no
// clean way to mock NDS in this slice, so we just construct one and hit
// its bus.

#include "nds.hpp"
#include "require.hpp"

using namespace ds;

static void main_ram_round_trip_all_widths() {
    NDS nds;

    nds.arm9_bus().write32(0x0200'0000, 0xDEAD'BEEFu);
    REQUIRE(nds.arm9_bus().read32(0x0200'0000) == 0xDEAD'BEEFu);
    REQUIRE(nds.arm9_bus().read16(0x0200'0000) == 0xBEEFu);
    REQUIRE(nds.arm9_bus().read16(0x0200'0002) == 0xDEADu);
    REQUIRE(nds.arm9_bus().read8 (0x0200'0000) == 0xEFu);
    REQUIRE(nds.arm9_bus().read8 (0x0200'0003) == 0xDEu);

    nds.arm9_bus().write16(0x0200'0010, 0xCAFEu);
    REQUIRE(nds.arm9_bus().read16(0x0200'0010) == 0xCAFEu);

    nds.arm9_bus().write8(0x0200'0020, 0x5Au);
    REQUIRE(nds.arm9_bus().read8(0x0200'0020) == 0x5Au);
}

static void main_ram_mirrors_four_times_across_the_region() {
    NDS nds;
    nds.arm9_bus().write32(0x0200'0000, 0x1234'5678u);

    // 4 MB mirrors at offsets 0, 0x0040'0000, 0x0080'0000, 0x00C0'0000.
    REQUIRE(nds.arm9_bus().read32(0x0240'0000) == 0x1234'5678u);
    REQUIRE(nds.arm9_bus().read32(0x0280'0000) == 0x1234'5678u);
    REQUIRE(nds.arm9_bus().read32(0x02C0'0000) == 0x1234'5678u);

    // Write near the far end of the 4 MB backing array, read near the far
    // end of the 16 MB region — should see the same byte.
    nds.arm9_bus().write32(0x023F'FFFC, 0xFACE'FEEDu);
    REQUIRE(nds.arm9_bus().read32(0x02FF'FFFC) == 0xFACE'FEEDu);
}

static void shared_wram_default_mode_is_arm9_32k() {
    NDS nds;
    // WRAMCNT = 0 at reset → ARM9 sees 32 KB at 0x0300'0000, mirrored.
    nds.arm9_bus().write32(0x0300'0000, 0xAABB'CCDDu);
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0xAABB'CCDDu);
    // 32 KB mirror: +0x0000'8000 is the same location.
    REQUIRE(nds.arm9_bus().read32(0x0300'8000) == 0xAABB'CCDDu);
}

static void unmapped_reads_return_zero_and_writes_are_noops() {
    NDS nds;
    // 0x0A region: unmapped in this slice. Must not crash.
    REQUIRE(nds.arm9_bus().read32(0x0A00'0000) == 0);
    nds.arm9_bus().write32(0x0A00'0000, 0xFFFF'FFFFu);
    REQUIRE(nds.arm9_bus().read32(0x0A00'0000) == 0);
}

static void io_region_routes_to_nds_stub() {
    NDS nds;
    // ARM9 I/O at 0x0400'xxxx. In this slice NDS::arm9_io_readN returns 0
    // and writes are no-ops. Just verify we don't crash and we see 0.
    REQUIRE(nds.arm9_bus().read32(0x0400'0000) == 0);
    REQUIRE(nds.arm9_bus().read16(0x0400'0000) == 0);
    REQUIRE(nds.arm9_bus().read8 (0x0400'0000) == 0);
    nds.arm9_bus().write32(0x0400'0000, 0x1234'5678u);
    nds.arm9_bus().write16(0x0400'0000, 0x1234u);
    nds.arm9_bus().write8 (0x0400'0000, 0x12u);
}

int main() {
    main_ram_round_trip_all_widths();
    main_ram_mirrors_four_times_across_the_region();
    shared_wram_default_mode_is_arm9_32k();
    unmapped_reads_return_zero_and_writes_are_noops();
    io_region_routes_to_nds_stub();
    return 0;
}
```

- [ ] **Step 7: Register the test.**

Edit `tests/CMakeLists.txt`, add:
```cmake
add_ds_unit_test(arm9_bus_test)
```

- [ ] **Step 8: Build and run tests.**

```bash
cd build && cmake .. && make && ctest --output-on-failure
```
Expected: `5/5 tests passed`. If `arm9_bus_test` fails, re-check the mask constants in `Arm9Bus::reset()` and `rebuild_shared_wram()`. The mirror-region test is the usual suspect — `kMainRamMask` must be `0x003F'FFFF`, not `0x00FF'FFFF`.

- [ ] **Step 9: Commit.**

```bash
git add src/bus/arm9_bus.hpp src/bus/arm9_bus.cpp src/CMakeLists.txt \
        src/nds.hpp src/nds.cpp tests/unit/arm9_bus_test.cpp tests/CMakeLists.txt
git commit -m "bus: Arm9Bus with main RAM + shared WRAM + I/O stub

Phase 1 slice 2 step 3: first real bus class. Fast-path read/write
via 256-entry page table, main RAM mirrored across its 16 MB region,
shared WRAM in default WRAMCNT=0 mode (32 KB all to ARM9), I/O slow
path routes to NDS stubs that return 0. Unmapped regions read as 0
and ignore writes. Unit tests cover all widths, mirroring, open bus,
and I/O routing."
```

---

## Task 4: `Arm7Bus` — ARM7 WRAM + shared WRAM slow path

Second bus. ARM7's memory map is trickier because the 16 MB region `0x03xx'xxxx` contains *two* sub-regions (shared WRAM at `0x0300'0000` and ARM7 WRAM at `0x0380'0000`) that a single top-8-bit page entry cannot distinguish. We solve this by leaving entry `0x03` as `nullptr` and handling the whole region in `slow_read`/`slow_write`. Entry `0x02` (main RAM) still goes through the fast path.

**Files:**
- Create: `src/bus/arm7_bus.hpp`
- Create: `src/bus/arm7_bus.cpp`
- Create: `tests/unit/arm7_bus_test.cpp`
- Modify: `src/CMakeLists.txt`
- Modify: `src/nds.hpp`
- Modify: `src/nds.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write `src/bus/arm7_bus.hpp`.**

```cpp
#pragma once

#include "bus/page_table.hpp"
#include "ds/common.hpp"

namespace ds {

class NDS;
class WramControl;

// ARM7's view of memory. Shape mirrors Arm9Bus except:
//   - No write access to ARM9-only I/O (WRAMCNT is ignored here).
//   - Region 0x03 is handled entirely in the slow path because it holds
//     two distinct sub-regions: 0x0300'0000 (shared WRAM, WRAMCNT-gated)
//     and 0x0380'0000 (ARM7 WRAM, always). A single 16 MB page entry
//     can't distinguish them.
class Arm7Bus {
public:
    Arm7Bus(NDS& nds, u8* main_ram, u8* shared_wram, u8* arm7_wram,
            const WramControl& wram_ctl);

    void reset();

    u32 read32(u32 addr);
    u16 read16(u32 addr);
    u8  read8 (u32 addr);

    void write32(u32 addr, u32 value);
    void write16(u32 addr, u16 value);
    void write8 (u32 addr, u8  value);

    // Called by NDS after a WRAMCNT write. ARM7's page table itself does
    // not change (entry 0x03 is always nullptr), but the slow-path lookup
    // reads from WramControl so there is nothing to rebuild. This method
    // exists to keep the bus interface symmetric with Arm9Bus.
    void rebuild_shared_wram() {}

private:
    u32  slow_read32 (u32 addr);
    u16  slow_read16 (u32 addr);
    u8   slow_read8  (u32 addr);
    void slow_write32(u32 addr, u32 value);
    void slow_write16(u32 addr, u16 value);
    void slow_write8 (u32 addr, u8  value);

    // Resolve a 0x03xx'xxxx address to the backing pointer + mask for the
    // current WRAMCNT state. Returns {nullptr, 0} for ARM9-only WRAMCNT
    // configurations (mode 0 at 0x0300'0000 is *not* unmapped — it mirrors
    // ARM7 WRAM).
    PageEntry resolve_region3(u32 addr) const;

    NDS*               nds_         = nullptr;
    u8*                main_ram_    = nullptr;
    u8*                shared_wram_ = nullptr;
    u8*                arm7_wram_   = nullptr;
    const WramControl* wram_ctl_    = nullptr;
    PageTable          table_{};
};

}  // namespace ds
```

- [ ] **Step 2: Write `src/bus/arm7_bus.cpp`.**

```cpp
#include "bus/arm7_bus.hpp"

#include "bus/wram_control.hpp"
#include "nds.hpp"

#include <cstring>

namespace ds {

namespace {
constexpr u8  kRegionMainRam = 0x02;
constexpr u32 kMainRamMask   = 0x003F'FFFFu;
constexpr u32 kArm7WramMask  = 0x0000'FFFFu;  // 64 KB mirror
}  // namespace

Arm7Bus::Arm7Bus(NDS& nds, u8* main_ram, u8* shared_wram, u8* arm7_wram,
                 const WramControl& wram_ctl)
    : nds_(&nds),
      main_ram_(main_ram),
      shared_wram_(shared_wram),
      arm7_wram_(arm7_wram),
      wram_ctl_(&wram_ctl) {}

void Arm7Bus::reset() {
    table_.clear();
    table_.read [kRegionMainRam] = PageEntry{main_ram_, kMainRamMask};
    table_.write[kRegionMainRam] = PageEntry{main_ram_, kMainRamMask};
    // Entry 0x03 stays nullptr — slow path handles shared/ARM7 WRAM split.
}

// Fast-path readers/writers are identical in shape to Arm9Bus. Duplicated
// on purpose so each bus is readable top-to-bottom without chasing a
// template.
u32 Arm7Bus::read32(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        u32 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~3u), sizeof(u32));
        return value;
    }
    return slow_read32(addr);
}

u16 Arm7Bus::read16(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        u16 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~1u), sizeof(u16));
        return value;
    }
    return slow_read16(addr);
}

u8 Arm7Bus::read8(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        return e.ptr[addr & e.mask];
    }
    return slow_read8(addr);
}

void Arm7Bus::write32(u32 addr, u32 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        std::memcpy(e.ptr + ((addr & e.mask) & ~3u), &value, sizeof(u32));
        return;
    }
    slow_write32(addr, value);
}

void Arm7Bus::write16(u32 addr, u16 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        std::memcpy(e.ptr + ((addr & e.mask) & ~1u), &value, sizeof(u16));
        return;
    }
    slow_write16(addr, value);
}

void Arm7Bus::write8(u32 addr, u8 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        e.ptr[addr & e.mask] = value;
        return;
    }
    slow_write8(addr, value);
}

// --- slow path ------------------------------------------------------------

PageEntry Arm7Bus::resolve_region3(u32 addr) const {
    // 0x0380'0000-0x03FF'FFFF is always ARM7 WRAM, 64 KB mirrored.
    if (addr & 0x0080'0000u) {
        return PageEntry{arm7_wram_, kArm7WramMask};
    }
    // 0x0300'0000-0x037F'FFFF depends on WRAMCNT.
    //   0: ARM7 sees ARM7 WRAM mirror (no shared-WRAM access)
    //   1: ARM7 sees shared WRAM high 16 KB (mirror)
    //   2: ARM7 sees shared WRAM low  16 KB (mirror)
    //   3: ARM7 sees full 32 KB shared WRAM (mirror)
    const u8 mode = wram_ctl_->value();
    switch (mode) {
        case 0: return PageEntry{arm7_wram_,              kArm7WramMask};
        case 1: return PageEntry{shared_wram_ + 0x4000u,  0x0000'3FFFu};
        case 2: return PageEntry{shared_wram_,            0x0000'3FFFu};
        case 3: return PageEntry{shared_wram_,            0x0000'7FFFu};
    }
    return PageEntry{};  // unreachable
}

u32 Arm7Bus::slow_read32(u32 addr) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        u32 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~3u), sizeof(u32));
        return value;
    }
    if ((addr >> 24) == 0x04) return nds_->arm7_io_read32(addr);
    return 0;
}

u16 Arm7Bus::slow_read16(u32 addr) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        u16 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~1u), sizeof(u16));
        return value;
    }
    if ((addr >> 24) == 0x04) return nds_->arm7_io_read16(addr);
    return 0;
}

u8 Arm7Bus::slow_read8(u32 addr) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        return e.ptr[addr & e.mask];
    }
    if ((addr >> 24) == 0x04) return nds_->arm7_io_read8(addr);
    return 0;
}

void Arm7Bus::slow_write32(u32 addr, u32 value) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        std::memcpy(e.ptr + ((addr & e.mask) & ~3u), &value, sizeof(u32));
        return;
    }
    if ((addr >> 24) == 0x04) nds_->arm7_io_write32(addr, value);
}

void Arm7Bus::slow_write16(u32 addr, u16 value) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        std::memcpy(e.ptr + ((addr & e.mask) & ~1u), &value, sizeof(u16));
        return;
    }
    if ((addr >> 24) == 0x04) nds_->arm7_io_write16(addr, value);
}

void Arm7Bus::slow_write8(u32 addr, u8 value) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        e.ptr[addr & e.mask] = value;
        return;
    }
    if ((addr >> 24) == 0x04) nds_->arm7_io_write8(addr, value);
}

}  // namespace ds
```

- [ ] **Step 3: Add `Arm7Bus` member + I/O stubs to `NDS`.**

Edit `src/nds.hpp`. Add include:
```cpp
#include "bus/arm7_bus.hpp"
```

Add accessor and ARM7 I/O methods in the public section below `arm9_bus()`:
```cpp
    Arm7Bus& arm7_bus() { return arm7_bus_; }

    u32  arm7_io_read32 (u32 addr);
    u16  arm7_io_read16 (u32 addr);
    u8   arm7_io_read8  (u32 addr);
    void arm7_io_write32(u32 addr, u32 value);
    void arm7_io_write16(u32 addr, u16 value);
    void arm7_io_write8 (u32 addr, u8  value);
```

Add the private member below `arm9_bus_`:
```cpp
    Arm7Bus arm7_bus_;
```

- [ ] **Step 4: Construct and reset `arm7_bus_` in `NDS`.**

Edit `src/nds.cpp`. Update the constructor's member-init list:
```cpp
NDS::NDS()
    : arm9_bus_(*this, main_ram_.data(), shared_wram_.data(), wram_ctl_),
      arm7_bus_(*this, main_ram_.data(), shared_wram_.data(), arm7_wram_.data(), wram_ctl_) {
    reset();
}
```

Update `reset()` to call the ARM7 bus reset:
```cpp
void NDS::reset() {
    scheduler_.reset();
    cpu9_.reset();
    cpu7_.reset();
    frame_done_ = false;

    main_ram_.fill(0);
    shared_wram_.fill(0);
    arm7_wram_.fill(0);

    wram_ctl_.reset();
    arm9_bus_.reset();
    arm7_bus_.reset();
}
```

Add ARM7 I/O stub definitions next to the ARM9 ones:
```cpp
u32 NDS::arm7_io_read32 (u32 /*addr*/) { return 0; }
u16 NDS::arm7_io_read16 (u32 /*addr*/) { return 0; }
u8  NDS::arm7_io_read8  (u32 /*addr*/) { return 0; }
void NDS::arm7_io_write32(u32 /*addr*/, u32 /*value*/) {}
void NDS::arm7_io_write16(u32 /*addr*/, u16 /*value*/) {}
void NDS::arm7_io_write8 (u32 /*addr*/, u8  /*value*/) {}
```

- [ ] **Step 5: Add `arm7_bus.cpp` to `src/CMakeLists.txt`.**

```cmake
    bus/arm7_bus.cpp
```

- [ ] **Step 6: Write `tests/unit/arm7_bus_test.cpp`.**

```cpp
// Unit tests for Arm7Bus. Covers main RAM fast path (shared with Arm9Bus)
// plus the region-3 slow path which is ARM7-specific.

#include "nds.hpp"
#include "require.hpp"

using namespace ds;

static void main_ram_shared_with_arm9() {
    NDS nds;
    nds.arm9_bus().write32(0x0200'0000, 0xDEAD'BEEFu);
    // Both buses see the same main RAM.
    REQUIRE(nds.arm7_bus().read32(0x0200'0000) == 0xDEAD'BEEFu);

    nds.arm7_bus().write32(0x0200'1000, 0xFEED'FACEu);
    REQUIRE(nds.arm9_bus().read32(0x0200'1000) == 0xFEED'FACEu);
}

static void arm7_wram_round_trips_at_0x03800000() {
    NDS nds;
    nds.arm7_bus().write32(0x0380'0000, 0xA1B2'C3D4u);
    REQUIRE(nds.arm7_bus().read32(0x0380'0000) == 0xA1B2'C3D4u);
    // 64 KB mirror across the 8 MB upper half of region 3.
    REQUIRE(nds.arm7_bus().read32(0x0381'0000) == 0xA1B2'C3D4u);
    REQUIRE(nds.arm7_bus().read32(0x03FF'0000) == 0xA1B2'C3D4u);
    // ARM9 does not have a mapping at 0x0380'0000 → reads zero.
    REQUIRE(nds.arm9_bus().read32(0x0380'0000) == 0);
}

static void wramcnt_0_arm7_sees_arm7_wram_mirror_at_0x03000000() {
    NDS nds;
    // Default WRAMCNT = 0 at reset.
    // ARM7 write at 0x0380'0000 should be visible at 0x0300'0000 on ARM7
    // because 0x0300'0000 maps to ARM7 WRAM in this mode.
    nds.arm7_bus().write32(0x0380'0000, 0x1122'3344u);
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x1122'3344u);
    // ARM9 at 0x0300'0000 sees shared WRAM (different backing), not ARM7 WRAM.
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0);
}

static void io_region_routes_to_arm7_stub() {
    NDS nds;
    REQUIRE(nds.arm7_bus().read32(0x0400'0000) == 0);
    nds.arm7_bus().write32(0x0400'0000, 0xFFFF'FFFFu);
    // Should not crash; write ignored.
    REQUIRE(nds.arm7_bus().read32(0x0400'0000) == 0);
}

int main() {
    main_ram_shared_with_arm9();
    arm7_wram_round_trips_at_0x03800000();
    wramcnt_0_arm7_sees_arm7_wram_mirror_at_0x03000000();
    io_region_routes_to_arm7_stub();
    return 0;
}
```

- [ ] **Step 7: Register the test.**

Edit `tests/CMakeLists.txt`, add:
```cmake
add_ds_unit_test(arm7_bus_test)
```

- [ ] **Step 8: Build and run tests.**

```bash
cd build && cmake .. && make && ctest --output-on-failure
```
Expected: `6/6 tests passed`.

- [ ] **Step 9: Commit.**

```bash
git add src/bus/arm7_bus.hpp src/bus/arm7_bus.cpp src/CMakeLists.txt \
        src/nds.hpp src/nds.cpp tests/unit/arm7_bus_test.cpp tests/CMakeLists.txt
git commit -m "bus: Arm7Bus with ARM7 WRAM + region-3 slow path

Phase 1 slice 2 step 4: ARM7's view of memory. Main RAM shares the
Arm9Bus fast path; region 0x03 is resolved in the slow path because
shared WRAM (0x0300'0000) and ARM7 WRAM (0x0380'0000) cannot coexist
in a single 16 MB page entry. Default WRAMCNT=0 mirrors ARM7 WRAM
into the lower sub-region as real hardware does. Unit tests cover
cross-core main RAM visibility, ARM7 WRAM mirroring, and the
WRAMCNT=0 ARM7-private view."
```

---

## Task 5: Full `WRAMCNT` banking with rebuild on write

Everything so far has hard-coded `WRAMCNT = 0` at reset. This task:
1. Replaces `NDS::arm9_io_write8` with a real handler that intercepts `0x0400'0247`, updates `WramControl`, and calls `rebuild_shared_wram()` on both buses.
2. Adds a `wramcnt_test.cpp` that exercises all four modes and confirms the rebuild is observed by both buses.

**Files:**
- Modify: `src/nds.cpp`
- Create: `tests/unit/wramcnt_test.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Replace `NDS::arm9_io_write8` with the WRAMCNT handler.**

Edit `src/nds.cpp`. Replace the stub:

```cpp
void NDS::arm9_io_write8(u32 addr, u8 value) {
    // WRAMCNT is the only I/O register handled in slice 2. Everything else
    // is an accepted no-op until later slices wire the remaining I/O.
    if (addr == 0x0400'0247) {
        wram_ctl_.write(value);
        arm9_bus_.rebuild_shared_wram();
        arm7_bus_.rebuild_shared_wram();
    }
}
```

Leave `arm9_io_write16` / `arm9_io_write32` as-is (still stubs). The WRAMCNT register is byte-sized; writes through wider accesses are uncommon and will be handled in a later slice when the full I/O dispatcher lands.

- [ ] **Step 2: Write `tests/unit/wramcnt_test.cpp`.**

```cpp
// Unit tests for WRAMCNT banking. Each mode is exercised from both buses,
// and we verify that a WRAMCNT write through ARM9 bus rebuilds both views.

#include "nds.hpp"
#include "require.hpp"

using namespace ds;

namespace {
constexpr u32 kWramcntAddr = 0x0400'0247u;

void set_wramcnt(NDS& nds, u8 mode) {
    nds.arm9_bus().write8(kWramcntAddr, mode);
}
}  // namespace

static void mode_0_arm9_has_full_32k_arm7_sees_private() {
    NDS nds;
    set_wramcnt(nds, 0);

    nds.arm9_bus().write32(0x0300'0000, 0xAABB'CCDDu);
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0xAABB'CCDDu);

    // 32 KB mirror: +0x0000'8000 is the same location, +0x0000'4000 is
    // the upper half (unwritten in this test, still zero).
    REQUIRE(nds.arm9_bus().read32(0x0300'4000) == 0);
    REQUIRE(nds.arm9_bus().read32(0x0300'8000) == 0xAABB'CCDDu);

    // ARM7 at 0x0300'0000 sees its own WRAM mirror, not shared WRAM.
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0);
}

static void mode_1_splits_16k_16k_low_to_arm9_high_to_arm7() {
    NDS nds;
    set_wramcnt(nds, 1);

    // ARM9 writes land in the low 16 KB of shared WRAM.
    nds.arm9_bus().write32(0x0300'0000, 0x1111'1111u);
    // ARM7 writes land in the high 16 KB (offset 0x4000).
    nds.arm7_bus().write32(0x0300'0000, 0x2222'2222u);

    // ARM9 reads back its own 16 KB slice, mirrored across region 3.
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0x1111'1111u);
    REQUIRE(nds.arm9_bus().read32(0x0300'4000) == 0x1111'1111u);  // 16 KB mirror
    // ARM7 reads back its own 16 KB slice.
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x2222'2222u);
    REQUIRE(nds.arm7_bus().read32(0x0300'4000) == 0x2222'2222u);  // 16 KB mirror
}

static void mode_2_splits_16k_16k_high_to_arm9_low_to_arm7() {
    NDS nds;
    set_wramcnt(nds, 2);

    nds.arm9_bus().write32(0x0300'0000, 0x3333'3333u);
    nds.arm7_bus().write32(0x0300'0000, 0x4444'4444u);

    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0x3333'3333u);
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x4444'4444u);

    // Cross-check: ARM9's high 16 KB is the same physical bytes as ARM7's
    // view under mode 1. Switch to mode 1 and ARM7 should see 0x33333333.
    set_wramcnt(nds, 1);
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x3333'3333u);
}

static void mode_3_arm7_has_full_32k_arm9_is_open_bus() {
    NDS nds;
    set_wramcnt(nds, 3);

    nds.arm7_bus().write32(0x0300'0000, 0x5555'5555u);
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x5555'5555u);
    REQUIRE(nds.arm7_bus().read32(0x0300'8000) == 0x5555'5555u);  // 32 KB mirror

    // ARM9 at 0x0300'0000 is now unmapped — reads return 0, writes ignored.
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0);
    nds.arm9_bus().write32(0x0300'0000, 0xFFFF'FFFFu);
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0);
    // The ARM7 data is still there.
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0x5555'5555u);
}

static void arm7_cannot_modify_wramcnt() {
    NDS nds;
    set_wramcnt(nds, 1);
    // ARM7 write to WRAMCNT: ARM7 I/O dispatch is a stub, so nothing
    // happens. Mode stays at 1.
    nds.arm7_bus().write8(kWramcntAddr, 3);

    // Prove: under mode 1 ARM9 still sees the low 16 KB slice.
    nds.arm9_bus().write32(0x0300'0000, 0x7777'7777u);
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0x7777'7777u);
    // ARM7 at 0x0300'0000 still sees the high slice, not the low one.
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) != 0x7777'7777u);
}

int main() {
    mode_0_arm9_has_full_32k_arm7_sees_private();
    mode_1_splits_16k_16k_low_to_arm9_high_to_arm7();
    mode_2_splits_16k_16k_high_to_arm9_low_to_arm7();
    mode_3_arm7_has_full_32k_arm9_is_open_bus();
    arm7_cannot_modify_wramcnt();
    return 0;
}
```

- [ ] **Step 3: Register the test.**

Edit `tests/CMakeLists.txt`, add:
```cmake
add_ds_unit_test(wramcnt_test)
```

- [ ] **Step 4: Build and run tests.**

```bash
cd build && cmake .. && make && ctest --output-on-failure
```
Expected: `7/7 tests passed`. If `mode_2` fails, verify the mode-2 branch in `Arm9Bus::rebuild_shared_wram()` uses `shared_wram_ + 0x4000u`, not `shared_wram_`. The high 16 KB is at offset 0x4000 into the 32 KB backing array.

- [ ] **Step 5: Also run a Release build to catch any `-DNDEBUG` fallout.**

```bash
cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make && ctest --output-on-failure
```
Expected: `7/7 tests passed`. The `REQUIRE` macro from slice 1 survives `-DNDEBUG`; this step just confirms no optimizer-specific breakage.

- [ ] **Step 6: Commit.**

```bash
git add src/nds.cpp tests/unit/wramcnt_test.cpp tests/CMakeLists.txt
git commit -m "bus: full WRAMCNT banking with rebuild on write

Phase 1 slice 2 step 5: writing 0x0400'0247 via ARM9 bus updates
WramControl and rebuilds the shared-WRAM page-table entry on both
buses. Unit tests cover all four modes (ARM9 32K / 16+16 split /
16+16 split / ARM7 32K) and confirm the ARM7 bus cannot modify
WRAMCNT — rule 4 (I/O routed through the bus it was issued from)."
```

---

## Task 6: `Arm9Bus` / `Arm7Bus` integration test via `NDS`

All prior tasks tested individual buses. This task adds one integration test that constructs an `NDS`, runs a frame (the existing scheduler drain, which is a no-op for the buses), and confirms nothing regressed — memory stays zero, the buses still work, and the frame loop doesn't crash.

This is deliberately small. The value is in the verification it provides once the full slice is in, not in any new logic.

**Files:**
- Create: `tests/unit/nds_integration_test.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write `tests/unit/nds_integration_test.cpp`.**

```cpp
// Slice-2 integration test. Proves the bus + WRAMCNT wiring survives a
// full NDS construction + reset + run_frame cycle, and that reset()
// zeroes memory as expected.

#include "nds.hpp"
#include "require.hpp"

using namespace ds;

static void run_frame_does_not_touch_bus_state() {
    NDS nds;
    nds.arm9_bus().write32(0x0200'0000, 0xCAFE'BABEu);
    nds.run_frame();
    REQUIRE(nds.arm9_bus().read32(0x0200'0000) == 0xCAFE'BABEu);
}

static void reset_zeroes_all_memory_and_resets_wramcnt() {
    NDS nds;

    // Dirty all three backing arrays + WRAMCNT.
    nds.arm9_bus().write32(0x0200'0000, 0x1111'1111u);
    nds.arm9_bus().write32(0x0300'0000, 0x2222'2222u);  // shared WRAM (mode 0)
    nds.arm7_bus().write32(0x0380'0000, 0x3333'3333u);  // ARM7 WRAM
    nds.arm9_bus().write8(0x0400'0247u, 2);             // WRAMCNT = 2

    // Under mode 2, ARM7 sees the low 16 KB.
    nds.arm7_bus().write32(0x0300'0000, 0x4444'4444u);

    nds.reset();

    REQUIRE(nds.arm9_bus().read32(0x0200'0000) == 0);
    REQUIRE(nds.arm9_bus().read32(0x0300'0000) == 0);  // back to mode 0, cleared
    REQUIRE(nds.arm7_bus().read32(0x0380'0000) == 0);
    // Mode is back to 0 — ARM7 at 0x0300'0000 mirrors its own cleared WRAM.
    REQUIRE(nds.arm7_bus().read32(0x0300'0000) == 0);
}

int main() {
    run_frame_does_not_touch_bus_state();
    reset_zeroes_all_memory_and_resets_wramcnt();
    return 0;
}
```

- [ ] **Step 2: Register the test.**

Edit `tests/CMakeLists.txt`, add:
```cmake
add_ds_unit_test(nds_integration_test)
```

- [ ] **Step 3: Build and run tests.**

```bash
cd build && cmake .. && make && ctest --output-on-failure
```
Expected: `8/8 tests passed`.

- [ ] **Step 4: Verify the emulator binary still runs.**

```bash
cd build && ./ds_emulator
```
Expected: the existing Phase 0 window opens with the test pattern; press Esc to close. No runtime errors in stderr.

- [ ] **Step 5: Commit.**

```bash
git add tests/unit/nds_integration_test.cpp tests/CMakeLists.txt
git commit -m "bus: NDS integration smoke test for slice 2

Phase 1 slice 2 step 6: prove NDS construction, run_frame, and reset
play nicely with the new bus members. One test confirms run_frame
does not disturb bus state, another confirms reset zeros all three
memory arrays and returns WRAMCNT to mode 0."
```

---

## Task 7: Amend the master design spec

The spec in §3.4 still describes "a 256-entry page table per access size (read8/16/32, write8/16/32)". The implemented shape is a single shared table with one read array and one write array. Also §4.1 should note that `WramControl` is the concrete owner of the `WRAMCNT` byte, and §4.4 should call out the slow-path fallback for ARM7 region 3.

This task is a doc-only amendment that keeps the spec and the code honest.

**Files:**
- Modify: `docs/superpowers/specs/2026-04-12-nds-emulator-design.md`

- [ ] **Step 1: Amend §3.4 to describe the implemented single-table shape.**

Open `docs/superpowers/specs/2026-04-12-nds-emulator-design.md`. Find §3.4 ("Bus domains"). Replace the paragraph starting "A 256-entry page table per access size..." with:

```markdown
- A **256-entry page table** keyed on the top 8 bits of the address. Each
  entry is a `{u8* ptr, u32 mask}` pair: when `ptr` is non-null, the fast
  path resolves the access as `ptr[addr & mask]` with a `std::memcpy` for
  load/store. Entries for palette / OAM / VRAM (which have 8-bit-write
  promotion rules) will be split by access size when those regions land in
  Phase 2; until then one shared read/write table is sufficient.
- A **slow-path dispatcher** invoked whenever `ptr` is null. Handles I/O
  regions (`0x04xx'xxxx`, routed through `NDS::armN_io_*`) and sub-region
  splits that a single 16-MB page entry cannot represent — notably ARM7's
  `0x03xx'xxxx`, which holds both shared WRAM (`0x0300'0000`) and private
  ARM7 WRAM (`0x0380'0000`).
- **References to shared memory** living on `NDS` — no duplication.
- **Waitstate tables** for timing — deferred until the CPUs need them.
```

- [ ] **Step 2: Amend §4.1 to name `WramControl`.**

Find the row in §4.1's table for "Shared WRAM" and update the Notes column if needed so the row reads:

```markdown
| Shared WRAM   | 32 KB        | `NDS::shared_wram_`   | Bankable per `WRAMCNT` (4 modes); state owned by `WramControl` |
```

Also add a short paragraph below the table, before §4.2:

```markdown
`WRAMCNT` lives in a dedicated `class WramControl` held by `NDS`. Writes to
`0x0400'0247` are routed through `NDS::arm9_io_write8` (ARM7 cannot modify
this register — rule 4), which updates `WramControl` and then calls
`rebuild_shared_wram()` on both buses. ARM9's entry 0x03 rebuilds to a
concrete `{ptr, mask}` pair; ARM7's entry stays null and the slow-path
dispatcher reads `WramControl` directly.
```

- [ ] **Step 3: Verify the doc still renders cleanly.**

```bash
grep -n "per access size" docs/superpowers/specs/2026-04-12-nds-emulator-design.md
```
Expected: no matches (the old phrasing is gone). If any remain, sweep them.

```bash
grep -n "WramControl" docs/superpowers/specs/2026-04-12-nds-emulator-design.md
```
Expected: at least one match in §4.1.

- [ ] **Step 4: Commit.**

```bash
git add docs/superpowers/specs/2026-04-12-nds-emulator-design.md
git commit -m "spec: amend §3.4/§4.1 to match slice-2 bus implementation

The design sketch said 'one page table per access size' and left
WRAMCNT ownership implicit. The implemented shape is a single
read/write table plus slow-path dispatch, and WRAMCNT lives in a
WramControl helper routed through NDS::arm9_io_write8. Sync the
master spec so the two docs don't drift."
```

---

## Verification checklist (run at the end)

- [ ] `cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make` is clean, no warnings.
- [ ] `cd build && ctest --output-on-failure` shows `8/8 tests passed`.
- [ ] `cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make && ctest --output-on-failure` also shows `8/8 tests passed`.
- [ ] `git log --oneline origin/main..HEAD` shows seven new commits matching Tasks 1–7, none with `Co-Authored-By` lines.
- [ ] `git status` is clean.
- [ ] `CLAUDE.md` does not appear in any commit in the branch (`git log -p origin/main..HEAD -- CLAUDE.md` produces no output).
- [ ] Running `./ds_emulator` in the build dir still opens the test-pattern window and exits on Esc.
- [ ] `grep -rn "reinterpret_cast<u32\*>\|reinterpret_cast<u16\*>" src/bus/` returns no matches (all loads/stores go through `std::memcpy`).
- [ ] `grep -n "Arm7\|Arm9" src/bus/arm9_bus.hpp src/bus/arm7_bus.hpp` shows that neither bus header pulls in the CPU headers — rule 3 (no subsystem pointing at another subsystem).
