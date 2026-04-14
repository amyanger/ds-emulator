# 🎮 ds-emulator

> A from-scratch **Nintendo DS** emulator written in **C++20**, built for the love of the craft and the dream of replaying *Pokémon HeartGold* on a machine I built myself.

[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/20)
[![CMake](https://img.shields.io/badge/CMake-3.20%2B-064F8C?logo=cmake&logoColor=white)](https://cmake.org/)
[![SDL2](https://img.shields.io/badge/SDL-2-blue)](https://www.libsdl.org/)
[![Apple Silicon](https://img.shields.io/badge/Apple%20Silicon-arm64-black?logo=apple&logoColor=white)](#-apple-silicon)
[![License](https://img.shields.io/badge/license-TBD-lightgrey)](#)

---

## ✨ What is this?

A solo project to build a Nintendo DS emulator from scratch in modern C++. The primary target is **Pokémon HeartGold / SoulSilver**, with full compatibility goals for the rest of the Gen 4 and Gen 5 Pokémon DS lineup:

- 💎 Pokémon **Diamond / Pearl**
- 🌟 Pokémon **Platinum**
- ❤️ Pokémon **HeartGold / SoulSilver** *(primary target)*
- ⚫ Pokémon **Black / White**
- ⚫ Pokémon **Black 2 / White 2**

---

## 🏗️ Architecture at a Glance

The DS isn't "GBA + more" — it's a fundamentally different machine, so the architecture starts from scratch:

- 🧠 **Two CPUs on two bus domains** — ARM9 (ARMv5TE @ 67 MHz, with caches + TCM) and ARM7 (ARMv4T @ 33 MHz). Each CPU has its own `Bus` object modeling its view of memory.
- ⏱️ **Single central event scheduler** (min-heap of timed events) drives everything. Nothing advances time except `scheduler.run_until(t)` — no scanline-chunk loop, no per-subsystem clocks.
- 🎨 **Two 2D PPU engines** (main + sub) sharing one class via an `is_main` flag — four BG modes, sprite engine, windowing, blending, master brightness.
- 📐 **Two-stage 3D pipeline** — geometry engine → `Frame3D` hand-off struct → software rasterizer, with post-processing passes for edge marking, fog, and toon/highlight. The door is deliberately left open for a hardware backend later.
- 🧩 **Runtime-reconfigurable VRAM** modeled as 9 independently-mapped banks (A–I) plus per-consumer page tables that are rebuilt on `VRAMCNT` writes.
- 🔊 **SPU on the ARM7 side** — 16 channels (PCM8 / PCM16 / IMA-ADPCM / PSG / noise) plus sound capture units.
- 🔌 **Direct boot only** — no BIOS, no firmware dumps required. The cart's KEY1 decryption is handled by a hardcoded seed table, and BIOS SWIs are provided by a minimal HLE layer.
- 🪟 **Three CMake targets**: `ds_core` (platform-free), `ds_frontend` (the only SDL-aware code), and `ds_emulator` (the binary). Unit tests link `ds_core` only — no SDL, no window.
- 🩻 **In-emulator X-Ray debug overlay** — page-cycled real-time view of CPU state, bus page tables, VRAM bank map, OAM, palette, geometry FIFO, polygon list, scheduler events, IPC state, and SPU channels.

---

## 🏛️ Non-Negotiable Architecture Rules

These come straight from the design spec and are enforced in code review:

1. ⏱️ **The scheduler is the clock.** Nothing advances time except `scheduler.run_until(t)`. Subsystems never sleep, wait, or run their own time loops.
2. 🧠 **Two bus domains, not one.** ARM9 and ARM7 each model their own view of memory. The same physical address can mean different things to each CPU.
3. 🌳 **No subsystem holds a pointer to another subsystem.** All cross-subsystem access goes through `NDS&`. The dependency graph is a tree.
4. 🔌 **I/O is routed per-bus.** ARM9 I/O and ARM7 I/O are different tables.
5. 💾 **Every subsystem implements `reset()`, `save_state()`, `load_state()`** from day one. Save states are not a bolt-on.
6. 🪟 **`class NDS` has no `<SDL.h>` in its transitive include graph.** Core is platform-free; this is enforced at link time.
7. 📁 **One hardware component per file.** When a file exceeds ~800 lines, split it by concern.
8. 🧮 **`Fixed<I, F>` for all 3D math.** Raw `int32_t` in the geometry pipeline is a code review block.
9. 🚀 **Direct boot only.** No BIOS / firmware dumps. If you want to load `bios9.bin`, you're solving the wrong problem — add the missing SWI to the HLE layer instead.

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
./ds_emulator <rom.nds>             # boot a ROM
./ds_emulator <rom.nds> --scale 3   # 3x window scale
```

ROMs go in `roms/`. Saves are written to `saves/<game_code>.sav` automatically. No BIOS or firmware required.

### 🧪 Tests

```bash
cd build
ctest --output-on-failure
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
- 🎯 NEON SIMD rasterizer paths
- 🧵 `QOS_CLASS_USER_INTERACTIVE` thread attribute (keeps the emulator off efficiency cores)
- 🔐 Codesign / JIT entitlement scaffolding

The default build stays portable across macOS arm64, macOS x86_64, Linux, and Windows. Apple-specific code is confined to `src/frontend/platform/macos.cpp` and `src/gpu3d/simd_neon.cpp` — nothing else uses `#ifdef __APPLE__`.

---

## 🎮 Controls

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
├── 📐 docs/specs/        ← Design spec (source of truth)
├── 🛠️ cmake/                         ← Build helpers
│   ├── CompilerWarnings.cmake
│   ├── AppleSilicon.cmake
│   └── Sanitizers.cmake
├── 📦 include/ds/                    ← Cross-cutting types
│   ├── common.hpp
│   ├── fixed.hpp                     ← templated Fixed<I, F>
│   └── ring_buffer.hpp               ← CircularBuffer<T, N>
├── ⚙️ src/
│   ├── main.cpp                      ← Entry point
│   ├── nds.hpp / nds.cpp             ← Top-level system
│   ├── scheduler/                    ← Min-heap event scheduler
│   ├── cpu/
│   │   ├── arm9/                     ← ARMv5TE core
│   │   ├── arm7/                     ← ARMv4T core
│   │   └── bios/                     ← HLE SWI handlers
│   ├── bus/                          ← ARM9 + ARM7 bus page tables
│   ├── memory/vram/                  ← 9-bank VRAM controller
│   ├── ppu/                          ← 2D engines (main + sub)
│   ├── gpu3d/                        ← 3D geometry + rasterizer
│   ├── spu/                          ← 16-channel sound processor
│   ├── dma/ timer/ ipc/ interrupt/   ← Supporting hardware
│   ├── cartridge/                    ← Slot-1 bus + KEY1 + saves
│   └── frontend/                     ← SDL2 — only place SDL is allowed
│       └── xray/                     ← In-emulator debug overlay
└── 🧪 tests/                         ← Unit tests (link ds_core only)
```

---

## 📖 Documentation

The full architecture spec lives at [`docs/specs/2026-04-12-nds-emulator-design.md`](docs/specs/2026-04-12-nds-emulator-design.md) — 16 sections covering every subsystem.

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
