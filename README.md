# 🎮 ds-emulator

> A from-scratch **Nintendo DS** emulator written in **C++20**, built for the love of the craft and the dream of replaying *Pokémon HeartGold* on a machine I built myself.

[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/20)
[![CMake](https://img.shields.io/badge/CMake-3.20%2B-064F8C?logo=cmake&logoColor=white)](https://cmake.org/)
[![SDL2](https://img.shields.io/badge/SDL-2-blue)](https://www.libsdl.org/)
[![Apple Silicon](https://img.shields.io/badge/Apple%20Silicon-arm64-black?logo=apple&logoColor=white)](#-apple-silicon)
[![Status](https://img.shields.io/badge/status-Phase%200%20%E2%9C%85-brightgreen)](#-current-status)
[![License](https://img.shields.io/badge/license-TBD-lightgrey)](#)

---

## ✨ What is this?

A solo project to build a Nintendo DS emulator from scratch in modern C++. The primary target is **Pokémon HeartGold / SoulSilver**, with full compatibility goals for the rest of the Gen 4 and Gen 5 Pokémon DS lineup:

- 💎 Pokémon **Diamond / Pearl**
- 🌟 Pokémon **Platinum**
- ❤️ Pokémon **HeartGold / SoulSilver** *(primary target)*
- ⚫ Pokémon **Black / White**
- ⚫ Pokémon **Black 2 / White 2**

This is the follow-up to my [GBA emulator](https://github.com/amyanger) — but the DS is a *fundamentally* more complex machine, so this isn't a port. It's been re-architected from the hardware up.

---

## 🚧 Current Status

**Phase 0 — Scaffolding ✅ Complete**

The foundation is in place. The build system works, the project skeleton is laid out, the foundational utility types (`Fixed<I, F>`, `CircularBuffer<T, N>`) are implemented and unit-tested, and an SDL2 window opens with the two-screen DS layout showing a test pattern. The hardware emulation itself starts in Phase 1.

```text
Phase 0 — Scaffolding                          ✅ DONE
Phase 1 — CPUs + bus + memory + direct boot    🔜 NEXT
Phase 2 — 2D PPU + first pixels                📋 planned
Phase 3 — Input + audio + saves                📋 planned
Phase 4 — 3D engine (geometry + rasterizer)    📋 planned
Phase 5 — Polish + accuracy + full compat      📋 planned
Phase 6 — Performance + tooling                📋 planned
```

Each phase has a concrete ROM-boot milestone (e.g. *Phase 2 ships when HG/SS title screen renders correctly*). Estimated timeline to all six Pokémon games playable: **4.5–7.5 months** of focused solo work.

---

## 🏗️ Architecture at a Glance

The DS isn't "GBA + more" — it's a fundamentally different machine, so the architecture starts from scratch:

- 🧠 **Two CPUs** on **two bus domains** — ARM9 (ARMv5TE @ 67 MHz, with caches + TCM) and ARM7 (ARMv4T @ 33 MHz)
- ⏱️ **Single central event scheduler** (min-heap of timed events) drives everything — no scanline-chunk loop
- 🎨 **Two 2D PPU engines** (main + sub) sharing one class via a flag
- 📐 **Two-stage 3D pipeline** — geometry engine → `Frame3D` struct → software rasterizer (with the door open for a hardware backend later)
- 🧩 **Runtime-reconfigurable VRAM** modeled as 9 banks + per-consumer page tables rebuilt on `VRAMCNT` writes
- 🔌 **Direct boot only** — no BIOS, no firmware dumps required; the cart's KEY1 decryption is handled by a hardcoded seed table + minimal HLE
- 🪟 **Three CMake targets**: `ds_core` (platform-free), `ds_frontend` (the only SDL-aware code), and `ds_emulator` (the binary)
- 🩻 **Staged in-emulator debug overlay** (`DsXray`) — page-cycled real-time view of CPU state, VRAM bank map, geometry FIFO, scheduler events, etc.

> 📐 **Full architecture spec:** [`docs/superpowers/specs/2026-04-12-nds-emulator-design.md`](docs/superpowers/specs/2026-04-12-nds-emulator-design.md) (16 sections, ~1000 lines)

---

## 🚀 Build & Run

### 📦 Dependencies

- **SDL2** — `brew install sdl2` (macOS) / `apt install libsdl2-dev` (Linux)
- **CMake 3.20+** — `brew install cmake` / `apt install cmake`
- **C++20 compiler** — clang 15+ or gcc 11+

That's it. SDL2 is the only runtime dependency.

### 🔨 Build

```bash
git clone https://github.com/amyanger/ds-emulator.git
cd ds-emulator
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make
```

### ▶️ Run

```bash
./ds_emulator                # opens the test pattern window
./ds_emulator --scale 3      # 3x window scale
./ds_emulator --frames 30    # auto-quit after 30 frames (for testing)
```

> ℹ️ Phase 0 only opens a window with a test pattern. The cart loader and game boot path land in Phase 1+.

### 🧪 Tests

```bash
cd build
ctest --output-on-failure
```

```text
    Start 1: fixed_test
1/2 Test #1: fixed_test .......................   Passed
    Start 2: ring_buffer_test
2/2 Test #2: ring_buffer_test .................   Passed

100% tests passed, 0 tests failed out of 2
```

Unit tests link against `ds_core` only — no SDL, no window, runs in milliseconds.

---

## 🍎 Apple Silicon

First-class build target. Optional optimized build flag enables `arm64`-specific tuning:

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DDS_TARGET_APPLE_SILICON=ON
make
```

When enabled, this turns on:

- 🛠️ `-mcpu=apple-m1` clang tuning
- 🎯 NEON SIMD rasterizer paths *(Phase 6 — placeholder for now)*
- 🧵 `QOS_CLASS_USER_INTERACTIVE` thread attribute (keeps the emulator off efficiency cores)
- 🔐 Codesign / JIT entitlement scaffolding *(unused until a JIT backend lands in Phase 6+)*

The default build stays portable across macOS arm64, macOS x86_64, Linux, and Windows. Apple-specific code is confined to `src/frontend/platform/macos.cpp` and `src/gpu3d/simd_neon.cpp` — nothing else uses `#ifdef __APPLE__`.

---

## 🎮 Controls *(planned, lands in Phase 3)*

| NDS Button         | Default Key      |
|--------------------|------------------|
| A                  | Z                |
| B                  | X                |
| X                  | S                |
| Y                  | A                |
| Start              | Enter            |
| Select             | Right Shift      |
| L                  | Q                |
| R                  | W                |
| D-Pad              | Arrow Keys       |
| 👆 Touch           | Left mouse drag  |
| 💼 Lid open/close  | L                |
| 🎤 Mic (blow)      | M (hold)         |

Rebindable via `keybinds.cfg`. MFi / Xbox / DualSense controllers auto-detected via SDL2.

### 🐛 Debug keys *(DEBUG builds)*

| Key | Action                                       |
|-----|----------------------------------------------|
| F1  | Dump both CPUs' state to stderr              |
| F2  | Toggle 🩻 X-Ray debug overlay                 |
| F3  | Step one instruction                         |
| F4  | Step one scanline                            |
| F5  | Step one frame                               |
| F6  | Toggle ARM9 instruction trace                |
| F7  | Toggle ARM7 instruction trace                |
| F8  | Dump main RAM                                |
| F9  | Dump VRAM (all banks)                        |
| F10 | Save state                                   |
| F11 | Load state                                   |
| Tab | Cycle X-Ray overlay pages                    |
| Esc | Quit                                         |

---

## 📁 Project Layout

```
ds-emulator/
├── 📐 docs/superpowers/
│   ├── specs/2026-04-12-nds-emulator-design.md     ← THE design spec (16 sections)
│   └── plans/2026-04-12-phase-0-scaffolding.md     ← Phase 0 task plan
├── 🛠️ cmake/                                       ← Build helpers
│   ├── CompilerWarnings.cmake
│   ├── AppleSilicon.cmake
│   └── Sanitizers.cmake
├── 📦 include/ds/                                   ← Cross-cutting types
│   ├── common.hpp
│   ├── fixed.hpp                                   ← templated Fixed<I, F>
│   └── ring_buffer.hpp                             ← CircularBuffer<T, N>
├── ⚙️ src/
│   ├── main.cpp                                    ← Entry point
│   ├── nds.hpp / nds.cpp                           ← Top-level system
│   ├── scheduler/                                  ← Min-heap event scheduler
│   ├── cpu/
│   │   ├── arm9/                                   ← ARMv5TE core (Phase 1)
│   │   └── arm7/                                   ← ARMv4T core (Phase 1)
│   └── frontend/                                   ← SDL2 — only place SDL is allowed
├── 🧪 tests/unit/                                  ← Unit tests (link ds_core only)
└── 📜 CLAUDE.md                                    ← Project conventions
```

---

## 📖 Documentation

| Document | What it is |
|----------|-----------|
| 🏛️ [Architecture Spec](docs/superpowers/specs/2026-04-12-nds-emulator-design.md) | Full design — 16 sections covering every subsystem |
| 📋 [Phase 0 Plan](docs/superpowers/plans/2026-04-12-phase-0-scaffolding.md) | Step-by-step implementation plan for the scaffolding phase |
| 📜 [CLAUDE.md](CLAUDE.md) | Project conventions, code style, hardware quirks, and the non-negotiable architecture rules |

---

## 🛣️ Roadmap

### ✅ Phase 0 — Scaffolding *(complete)*
Window opens, test pattern displays, build system and project skeleton in place, foundational utility types implemented and tested.

### 🔜 Phase 1 — CPUs, bus, memory, direct boot *(next)*
Full ARM9 (ARMv5TE) and ARM7 (ARMv4T) decoders, CP15 + TCM, dual bus domains with page tables, raw memory + `VramController` skeleton, 8-channel DMA, 8 timers, 2 IRQ controllers, IPC sync + FIFO, HLE BIOS SWIs, direct-boot path with KEY1 cart decryption, real scheduler.
> 🎯 **Milestone:** Pokémon HeartGold executes past cart boot to its ARM9 entry point and runs several frames without crashing. *No pixels yet — verified via instruction trace.*

### 📋 Phase 2 — 2D PPU
Both 2D engines, all 4 BG modes, sprite engine, compositor, real VRAM bank routing.
> 🎯 **Milestone:** HG/SS title screen renders correctly with the Lugia animation.

### 📋 Phase 3 — Input + audio + saves
SPU (16 channels), keypad, touchscreen, audio output, EEPROM + FLASH saves.
> 🎯 **Milestone:** HG/SS playable from title to walking around New Bark Town with music and SFX. Save and reload work.

### 📋 Phase 4 — 3D engine *(the big one)*
Geometry engine, software rasterizer, edge marking, fog, toon shading.
> 🎯 **Milestone:** HG/SS first wild Pokémon battle renders correctly. *(Both Pokémon as 3D models.)*

### 📋 Phase 5 — Polish + Gen 5 compatibility
Save states, rewind, RTC, cache timing, BW / BW2 fixes.
> 🎯 **Milestone:** All six target Pokémon games playable to first gym, 30+ minutes without crashes.

### 📋 Phase 6 — Performance + tooling
NEON SIMD rasterizer, GDB stub, headless regression tests, optional cached interpreter / JIT, optional hardware 3D backend.
> 🎯 **Milestone:** HG/SS at full speed on M-series Macs, full main story playable end to end.

---

## 🏛️ Non-Negotiable Architecture Rules

These come straight from the design spec and are enforced in code review:

1. ⏱️ **The scheduler is the clock.** Nothing advances time except `scheduler.run_until(t)`.
2. 🧠 **Two bus domains, not one.** ARM9 and ARM7 each model their own view of memory.
3. 🌳 **No subsystem holds a pointer to another subsystem.** All cross-subsystem access goes through `NDS&`.
4. 🔌 **I/O is routed per-bus.** ARM9 I/O and ARM7 I/O are different tables.
5. 💾 **Every subsystem implements `reset()`, `save_state()`, `load_state()`** from day one.
6. 🪟 **`class NDS` has no `<SDL.h>` in its transitive include graph.** Core is platform-free; this is enforced at link time.
7. 📁 **One hardware component per file.** When a file exceeds ~800 lines, split.
8. 🧮 **`Fixed<I, F>` for all 3D math.** Raw `int32_t` in the geometry pipeline is a code review block.
9. 🚀 **Direct boot only.** No BIOS / firmware dumps. Add missing SWIs to the HLE layer instead.

---

## 📚 References

The references that made this project possible:

- 📕 **[GBATEK](https://problemkaputt.de/gbatek.htm)** — the canonical DS hardware reference
- 📗 **[GBATEK Markdown fork](https://mgba-emu.github.io/gbatek/)** — same content, prettier
- 🌐 **[Copetti's NDS Architecture write-up](https://www.copetti.org/writings/consoles/nintendo-ds/)** — accessible deep dive
- 🦊 **[melonDS](https://github.com/melonDS-emu/melonDS)** — reference accurate DS emulator
- 🐉 **[DeSmuME](https://github.com/TASEmulators/desmume)** — older but still invaluable
- 📚 **[awesome-nds-dev](https://github.com/pret/awesome-nds-dev)** — curated resource list
- 🔐 **[KEY1/KEY2 cartridge encryption docs](https://problemkaputt.de/gbatek.htm#dscartridgeencryption)**

---

## ⚖️ Legal

This project does **not** distribute any Nintendo intellectual property:
- ❌ No ROMs
- ❌ No BIOS files
- ❌ No firmware dumps
- ❌ No proprietary game data

Direct boot is implemented from publicly documented hardware behavior. Users supply their own legally obtained ROMs of games they own. Pokémon, Nintendo DS, and all related trademarks belong to Nintendo / The Pokémon Company / Game Freak.

---

## 🙏 Acknowledgments

Built standing on the shoulders of giants: the GBATEK authors who reverse-engineered the DS hardware, the melonDS and DeSmuME teams for showing what's possible, and the gbadev / nds-dev homebrew communities for keeping the knowledge alive.

---

## 📬 Status

🚧 **Active development.** This is a personal hobby project — slow, methodical, and built phase-by-phase against verifiable Pokémon ROM milestones. Watch / star to follow along.

> 💡 *"You don't really understand a machine until you've tried to emulate it."*
