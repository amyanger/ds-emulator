# Nintendo DS Emulator

A Nintendo DS emulator written in C++20, targeting Pokemon HeartGold / SoulSilver
primarily and full compatibility with Gen 4 and Gen 5 Pokemon DS titles
(Diamond, Pearl, Platinum, Black, White, Black 2, White 2). First-class build
support for Apple Silicon macOS, portable by default.

**Design spec:** `docs/superpowers/specs/2026-04-12-nds-emulator-design.md` —
this is the source of truth for the architecture. Read it before making any
non-trivial change.

---

## Build & Run

```bash
# Default portable build (macOS, Linux, Windows)
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make

# Apple Silicon optimized build (M1/M2/M3/M4)
mkdir -p build && cd build && cmake .. -DDS_TARGET_APPLE_SILICON=ON -DCMAKE_BUILD_TYPE=Release && make

# Run
./ds_emulator <rom.nds> --scale 3

# Run with X-Ray debug overlay enabled (default)
./ds_emulator <rom.nds> --scale 3
# ...then press F2 in the window to toggle

# Build without X-Ray overlay (smaller binary, no debug UI)
mkdir -p build && cd build && cmake .. -DENABLE_XRAY=OFF && make

# Run headless (for regression tests)
./ds_emulator <rom.nds> --headless --frames 600 --dump-hash out.txt

# Unit tests (no SDL, runs in milliseconds)
cd build && make ds_tests && ./ds_tests

# Clean rebuild
rm -rf build && mkdir build && cd build && cmake .. && make
```

**No BIOS, no firmware required.** Direct boot reads the cart header, copies
the ARM9/ARM7 binaries to RAM, initializes I/O registers to post-firmware
values, generates the KEY1 cart decryption schedule, and jumps to the entry
points. Place ROMs in `roms/` — saves are written to `saves/<game_code>.sav`
automatically.

---

## Controls

| NDS Button        | Default Key      |
|-------------------|------------------|
| A                 | Z                |
| B                 | X                |
| X                 | S                |
| Y                 | A                |
| Start             | Enter            |
| Select            | Right Shift      |
| L                 | Q                |
| R                 | W                |
| D-Pad             | Arrow Keys       |
| Touch             | Left mouse drag  |
| Lid open/close    | L                |
| Mic (blow)        | M (hold)         |

Remap via `keybinds.cfg`. MFi / Xbox / DualSense controllers auto-detected via
SDL2.

### Debug keys (DEBUG builds only)

| Key | Action                                         |
|-----|------------------------------------------------|
| F1  | Dump both CPUs' state to stderr                |
| F2  | Toggle X-Ray debug overlay                      |
| F3  | Step one instruction (ARM9 priority)            |
| F4  | Step one scanline                              |
| F5  | Step one frame                                 |
| F6  | Toggle ARM9 instruction trace                  |
| F7  | Toggle ARM7 instruction trace                  |
| F8  | Dump main RAM to `dump_mainram.bin`             |
| F9  | Dump VRAM (all banks) to `dump_vram.bin`        |
| F10 | Save state to `savestate_0.bin`                |
| F11 | Load state from `savestate_0.bin`               |
| Tab | Cycle X-Ray overlay pages                       |
| Esc | Quit                                            |

---

## Dependencies

- **SDL2**: `brew install sdl2` (macOS) / `apt install libsdl2-dev` (Linux)
- **CMake 3.20+**: `brew install cmake` / `apt install cmake`
- **C++20 compiler**: clang 15+ or gcc 11+

No other external libraries. SDL2 is the only runtime dependency.

---

## Project Structure

```
src/
  main.cpp              Entry point, CLI parsing, Frontend/NDS wiring
  nds.hpp / nds.cpp     class NDS — top-level, owns every subsystem

  scheduler/
    scheduler.hpp/.cpp  Min-heap event scheduler. THE CLOCK of the emulator.

  cpu/
    cpu_core.hpp        Abstract CpuCore interface
    dispatcher.hpp      Swappable instruction dispatcher (interpreter today)
    arm9/               ARM9 (ARMv5TE, 67 MHz, caches + TCM)
      arm9.hpp/.cpp     CPU state, IRQ check, run_until
      arm9_decode.cpp   ARMv5TE ARM instruction decoder/executor
      arm9_thumb.cpp    Thumb instruction decoder/executor
      cp15.hpp/.cpp     System control coprocessor
      tcm.cpp           ITCM / DTCM remapping
    arm7/               ARM7 (ARMv4T, 33 MHz, fresh C++ rewrite)
      arm7.hpp/.cpp
      arm7_decode.cpp
      arm7_thumb.cpp
    bios/
      bios9_hle.cpp     HLE SWI handlers for ARM9
      bios7_hle.cpp     HLE SWI handlers for ARM7

  bus/
    arm9_bus.hpp/.cpp   256-entry page tables for ARM9 memory view
    arm7_bus.hpp/.cpp   256-entry page tables for ARM7 memory view
    io_regs.hpp         I/O register address constants
    wram_control.cpp    Shared WRAM banking (WRAMCNT)

  memory/
    main_ram.cpp        4 MB main RAM wrapper
    vram/
      vram_controller.hpp/.cpp   9-bank VRAM + per-consumer page tables
      vram_page_tables.cpp       "Cheat sheet" rebuild on VRAMCNT writes

  dma/
    dma_controller.cpp  4 channels per CPU (8 total)

  timer/
    timer_bank.cpp      4 timers per CPU (8 total) with cascade

  ipc/
    ipc_sync.cpp        IPC sync register
    ipc_fifo.cpp        64-byte bidirectional FIFOs between CPUs

  interrupt/
    irq_controller.cpp  IE/IF/IME per CPU

  ppu/                  2D Picture Processing Unit (two engines)
    ppu_engine.hpp/.cpp class PpuEngine — one class, two instances (is_main flag)
    render_bg_text.cpp  Text BG (tiled)
    render_bg_affine.cpp
    render_bg_extended.cpp
    render_bg_bitmap.cpp
    render_sprites.cpp  OAM sprites (all sizes, 1D/2D, affine)
    compositor.cpp      Windows, blending, master brightness
    palette.cpp         Palette RAM + extended palettes

  gpu3d/                3D engine (geometry + software rasterizer)
    geometry_engine.hpp/.cpp    Command FIFO, matrix stacks, vertex pipeline
    matrix_stack.cpp            4 matrix stacks (projection/modelview/dir/tex)
    vertex_pipeline.cpp         Transform + light + project
    clipper.cpp                 6-plane frustum clipping
    gx_fifo.cpp                 GXFIFO + GXCMD packed command ingestion
    rendering_engine.hpp/.cpp   Scanline software rasterizer
    rasterizer.cpp              Edge-function triangle raster
    interpolator.cpp            Perspective-correct attribute interp
    texture_unit.cpp            Texel fetch via VramController
    post_edge.cpp               Edge marking pass
    post_fog.cpp                Fog pass
    post_toon.cpp               Toon / highlight pass
    frame3d.hpp                 Hand-off struct (color + depth + attr)
    simd_neon.cpp               Apple Silicon NEON fast path

  spu/                  Sound Processing Unit (16 channels, owned by NDS)
    spu.hpp/.cpp
    channel_pcm.cpp     PCM8 / PCM16
    channel_adpcm.cpp   IMA ADPCM
    channel_psg.cpp     PSG square waves (channels 8-13)
    channel_noise.cpp   LFSR noise (channels 14-15)
    capture.cpp         Sound capture units

  cartridge/
    cartridge.hpp/.cpp  class Cartridge — ROM + save + protocol
    nds_header.cpp      Header parsing
    cart_protocol.cpp   Slot-1 bus state machine
    key1.cpp            KEY1 seed table + per-game schedule (direct boot)
    save_chip.hpp
    save_eeprom.cpp     EEPROM saves (HG/SS, Platinum)
    save_flash.cpp      FLASH saves (D/P, B/W, B/W2)
    save_detect.cpp     Auto-detect save type from game code
    card_dma.cpp        Card-DMA transfers scheduled as events

  rtc/
    rtc.cpp             Real-time clock backed by host time

  input/
    keypad.cpp          Button state + KEYINPUT/KEYCNT
    touchscreen.cpp     TSC2046 SPI touch controller model
    mic.cpp             Microphone stub

  frontend/             ONLY place SDL2 is allowed. NDS core is platform-free.
    frontend.hpp/.cpp   SDL window, renderer, audio device, input pump
    audio_ring.cpp      Lock-free SPSC ring buffer for audio
    input_config.cpp    keybinds.cfg loader
    platform/
      macos.cpp         QoS class, HiDPI, Apple Silicon specifics
      generic.cpp       Default stubs
    xray/               In-emulator debug overlay (F2 to toggle)
      xray.hpp/.cpp     Page manager
      xray_canvas.cpp   Overlay drawing primitives
      xray_font.hpp     Embedded constexpr bitmap font
      page_cpu.cpp      ARM9+ARM7 state, disasm of current instruction
      page_bus.cpp      Bus page tables + TCM state
      page_vram.cpp     Live 9-bank VRAM map color-coded by purpose
      page_ppu.cpp      DISPCNT, BG state, window state per engine
      page_oam.cpp      Sprite list
      page_palette.cpp  Color grid
      page_geometry.cpp Command FIFO, matrix stacks, polygon RAM usage
      page_poly.cpp     Finished polygon list with attributes
      page_scheduler.cpp Upcoming events + last N fired
      page_ipc.cpp      IPC FIFO fill, sync register state
      page_audio.cpp    SPU channel state + mixing visualization

  savestate/
    savestate.hpp       Uniform serialize/deserialize interface
    savestate.cpp

include/
  ds/
    common.hpp          Fixed-width types, Cycle alias, LOG macros, bit helpers
    fixed.hpp           templated Fixed<I, F> fixed-point type (REQUIRED in 3D code)
    ring_buffer.hpp     CircularBuffer<T, N> template
    span_ext.hpp        Small std::span helpers

cmake/
  CompilerWarnings.cmake
  AppleSilicon.cmake    arm64 + NEON + QoS + codesign, flag-gated
  Sanitizers.cmake

tests/                  Unit + regression tests (links ds_core only)
roms/                   ROMs (gitignored)
saves/                  Save files (gitignored)
build/                  Build artifacts (gitignored)
```

### Build targets

```
libds_core.a       Everything under src/ except frontend/ and main.cpp.
                   Zero SDL2 dependency. Platform-free.
libds_frontend.a   src/frontend/. SDL2 lives here and nowhere else.
ds_emulator        main.cpp + ds_frontend + ds_core.
ds_tests           tests/ + ds_core only. No SDL.
```

---

## Architecture Rules (non-negotiable)

1. **The scheduler is the clock.** Nothing advances time except
   `scheduler.run_until(t)`. Subsystems never sleep, wait, or run their own
   time loops.
2. **Two bus domains, not one.** ARM9 and ARM7 each own a `class Bus` that
   models that CPU's view of memory. Shared backing storage lives on `NDS`;
   bus objects hold references.
3. **No subsystem holds a pointer to another subsystem.** All cross-subsystem
   interaction is a method call on `NDS&`. The dependency graph is a tree.
4. **I/O access is routed through the bus it was issued from.** ARM9 I/O and
   ARM7 I/O are different tables.
5. **Every subsystem implements `reset()`, `save_state()`, `load_state()`**
   from day one. Save states are not a bolt-on.
6. **`class NDS` has no `<SDL.h>` in its transitive include graph.** NDS core
   is a platform-free static library; the frontend is a separate target.
   Violating this is a build-break.
7. **One hardware component per file.** When a file exceeds ~800 lines, split
   it by concern. Do not merge unrelated functionality.
8. **No cross-subsystem includes.** `ppu/` never includes from `cpu/`.
9. **`Fixed<I, F>` for all 3D math.** Raw `int32_t` in the geometry engine or
   rasterizer is a code review block.
10. **Direct boot only.** No BIOS / firmware dumps. If you find yourself
    wanting to load `bios9.bin`, you're solving the wrong problem — add the
    missing SWI to the HLE layer instead.

---

## Code Style

- **C++20 standard.** Enforced by CMake (`CMAKE_CXX_STANDARD 20`).
- **"C with containers and strong types"** — use `std::array`, `std::span`,
  `std::variant`, templated `Fixed`/`CircularBuffer`; avoid virtual
  inheritance in hardware components, avoid exceptions in the hot path, avoid
  smart pointers in the CPU loop.
- **4-space indentation**, no tabs. See `.clang-format`.
- **100-column line limit.**
- **Pointer alignment left**: `uint8_t* ptr` not `uint8_t *ptr`.
- **Fixed-width types everywhere**: `uint8_t`, `uint16_t`, `uint32_t`,
  `int8_t`, `int16_t`, `int32_t`. Never use `int` or `unsigned` for hardware
  state.
- **Struct naming**: `struct Foo { ... };` — class when it has non-trivial
  invariants or lifecycle, struct when it's plain data.
- **Function naming**: `module_action` for free functions (`bus_read32`),
  `methodName` on class methods (`arm9.run_until`). Consistent within a file.
- **Constants**: `#define` with `UPPER_SNAKE_CASE` only for hardware register
  addresses. `constexpr` everywhere else.
- **Enum class** over plain enum. Hardware-mirrored bitfields get a
  `static_cast`-friendly underlying type.
- **Include guards** via `#pragma once` — consistent across the repo.
- **Forward declarations** over circular includes.
- **Compiler warnings are errors-in-spirit**. Code compiles clean with
  `-Wall -Wextra -Wpedantic -Werror`.

---

## Critical Hardware Details

These are non-obvious DS behaviors that MUST be correct. Verify against
GBATEK when implementing.

### CPU / bus
- **Two cores, two bus views.** The same physical address can mean different
  things to ARM9 vs ARM7 (IPC regs, SPU regs, cart bus).
- **ARM9 PC is ahead by 8 (ARM) / 4 (Thumb)** due to the 3-stage pipeline.
  ARM7 is the same (ARMv4T pipeline). Getting this wrong produces
  off-by-one branches everywhere.
- **CP15 TCM remaps.** When the game moves ITCM/DTCM via CP15, rebuild the
  ARM9 bus page table for only the affected address range.
- **ARMv5TE additions:** `CLZ`, `BLX`, `QADD`/`QSUB`, signed saturating math,
  extended `SMLA*`/`SMUL*`, `BKPT`, `PLD` (hint, nop). Roughly 15-20 new
  encodings on top of ARMv4T.

### Memory / VRAM banks
- **VRAM is 9 independently mapped banks** (A-I). Every `VRAMCNT_x` write
  potentially changes which bank serves which address in which consumer's
  address space. The `VramController` rebuilds only the affected
  per-consumer page tables, not all of them.
- **Bank overlap policy:** when two banks are mapped to the same purpose at
  the same offset, **later-written wins**. Real hardware behavior is
  undefined; this is the accepted convention.
- **Writing 1 to IF clears that bit** (acknowledge interrupt). Opposite of
  most registers.
- **KEYINPUT is active-LOW**: bit = 0 means pressed. Initialize to `0x03FF`.
- **8-bit writes to palette / OAM / VRAM obj-BG regions are ignored or
  promoted to halfword duplicate** — per-region rules, documented in GBATEK.

### Scheduler
- **`now` is ARM9 cycles**, 64-bit. ARM7 runs at exactly half rate:
  `Arm7::run_until(arm9_target)` advances until `arm7_cycles == arm9_target / 2`.
- **Both cores take an ARM9-cycle target.** Never pass ARM7 cycles to ARM7's
  `run_until`. The halving is internal.
- **Cancelled events stay in the heap as tombstones** until they're popped.
  Never try to remove mid-heap.
- **Event latency matters.** DMA start, IRQ delivery, cart DMA completion —
  these all take N cycles to propagate on real hardware. Schedule the event
  N cycles in the future, don't fire it immediately.

### 3D engine
- **Matrix stacks use right-multiplication**: `MTX_MULT_*` computes
  `current = current × argument`, not `argument × current`.
- **`VTX_XY`/`VTX_XZ`/`VTX_YZ`/`VTX_DIFF`** reuse coordinates from the
  previous vertex. Track "last vertex" in the pipeline state.
- **Winding and culling are decided post-projection in screen space**, not
  in world space.
- **`SWAP_BUFFERS` is the frame boundary.** Until it fires, polygon RAM
  accumulates; when it fires, the rasterizer runs and vertex/polygon RAM
  double-buffers swap.
- **Matrix stack overflow sets a dirty bit**, does not raise an exception.
- **GXFIFO back-pressure**: >128 entries stalls ARM9 on writes. Modeled as a
  scheduler event that releases the stall when drained.
- **Use `Fixed<I, F>` everywhere.** Raw int shift-width bugs are the worst
  3D bugs to track down.

### SPU
- **SPU is on ARM7 side of the bus.** ARM9 has no I/O mapping for SPU
  registers; reads return open-bus. `Arm9Bus` must not contain a handler for
  `0x0400_0400..0x0400_04FF`.
- **IMA ADPCM uses a specific 4-step prediction table.** Off-by-one in the
  predictor index ruins every sound.
- **Channel sample rates are arbitrary** per-channel via `timer_reload`.
  Cannot precompute a fixed rate per channel.
- **Pokemon games don't feed raw samples** — they run a sequencer on ARM7
  that generates SPU register writes. The emulator just has to respond
  correctly to writes; it never needs to parse SDAT/SSEQ.

### Cart
- **KEY1 decryption** uses a precomputed 0x1048-byte seed table (same for
  every DS, documented on GBATEK) plus per-game mixing from the game code in
  the header. Lives in `cart_keys.cpp`.
- **Direct boot** parses the header, copies ARM9/ARM7 binaries to their
  specified RAM addresses, initializes ~30 I/O registers to post-firmware
  values, builds the KEY1 schedule, and jumps. No firmware menu.
- **Save type auto-detect** from `header.game_code` via a lookup table. No
  runtime heuristics.

---

## Testing

### Unit tests (`ds_tests`)
- `fixed_test.cpp` — `Fixed<I, F>` arithmetic.
- `scheduler_test.cpp` — min-heap, cancellation, same-cycle ordering.
- `clipper_test.cpp` — frustum clip of degenerate cases.
- `vram_controller_test.cpp` — bank remapping on every `VRAMCNT` variant.

Linked against `ds_core` only, runs in milliseconds, no SDL, no window. Add
tests here for anything purely computational with a clean interface.

### Regression tests (Phase 6+)
Headless harness: `./ds_emulator --headless --frames N --dump-hash out.txt`
runs the emulator with a recorded input stream and hashes the final
framebuffer. Compare against stored hashes to catch regressions.

### Test ROMs (place in `roms/tests/`)
- **armwrestler-nds** — ARM instruction correctness.
- **gbatek-armtest** — timing / edge cases.
- **melonDS test rom suite** — scheduler and hardware feature validation.
- **mollusk/ndsexamples** — hardware feature demos.

### Pokemon boot milestone checklist (per target game)
1. Cart header parsed, ARM9/ARM7 binaries loaded, direct boot jumps cleanly.
2. Nintendo logo / opening splash renders.
3. Title screen renders with correct colors and animation.
4. "New Game" advances to intro cutscene, accepts input.
5. Overworld loads, player visible, can walk.
6. Music plays correctly.
7. Wild Pokemon battle renders (first "holy crap" moment for 3D).
8. Save → close → reload → continue works.
9. Play 30+ minutes without crash.

### Debugging workflow
- **F1**: register dump of both CPUs.
- **F2**: toggle X-Ray overlay (per-page state viewers).
- **F6/F7**: instruction trace for ARM9 / ARM7. Compare against melonDS or
  DeSmuME trace output to find divergence points. This is the single most
  valuable debugging tool for correctness bugs.
- **F8/F9**: memory dumps for offline inspection.

---

## Monitor Tool — Streaming Error Detection

Use the Monitor tool to watch for errors in real-time during builds and
testing:

```bash
# Stream build errors/warnings
cd build && cmake .. && make 2>&1 | grep --line-buffered -E "error:|warning:|undefined reference|fatal"

# Watch emulator output for failures during ROM boot
./ds_emulator <rom.nds> 2>&1 | grep --line-buffered -E "ERROR|FAIL|WARN|assertion|segfault"

# Monitor instruction trace divergence (compare against melonDS)
./ds_emulator <rom.nds> 2>&1 | grep --line-buffered -E "PC9=|PC7=|SIGABRT|segfault|Bus error"

# Watch unit tests
./ds_tests 2>&1 | grep --line-buffered -E "FAIL|PASS|error"
```

- Set `persistent: true` for session-length watches.
- Use selective `grep` filters — instruction traces are extremely verbose.
- Use `TaskStop` to cancel early.

---

## Implementation Phases

Each phase has a **concrete boot milestone** against a real Pokemon ROM. Do
not start Phase N+1 until Phase N's milestone ROM runs reliably.

| Phase | Focus | Milestone |
|-------|-------|-----------|
| 0 | Scaffolding | Window opens, two-screen test pattern |
| 1 | CPUs + bus + memory + IRQs + IPC + direct boot | HG/SS executes past cart boot (no pixels yet) |
| 2 | 2D PPU, first pixels | HG/SS title screen + Lugia animation renders |
| 3 | Input + audio + saves | HG/SS from title to New Bark Town with sound |
| 4 | 3D engine | HG/SS wild battle renders (biggest phase) |
| 5 | Polish + DPPt/BW/BW2 compatibility | All 6 Pokemon games to first gym, 30 min playable |
| 6 | Performance + tooling | Full speed on M-series, full main story playable |

Phases 0–5: **18–30 weeks** of focused solo work. Phase 6 is indefinite polish.

See `docs/superpowers/specs/2026-04-12-nds-emulator-design.md` §13 for
detailed per-phase deliverables.

---

## Key References

- **GBATEK** (primary hardware ref): https://problemkaputt.de/gbatek.htm
- **GBATEK Markdown fork**: https://mgba-emu.github.io/gbatek/
- **melonDS source** (reference emulator): https://github.com/melonDS-emu/melonDS
- **DeSmuME source**: https://github.com/TASEmulators/desmume
- **Copetti NDS Architecture**: https://www.copetti.org/writings/consoles/nintendo-ds/
- **awesome-nds-dev**: https://github.com/pret/awesome-nds-dev
- **KEY1 / KEY2 cart encryption**: https://problemkaputt.de/gbatek.htm#dscartridgeencryption

---

## Agent Workflow (DS-specific)

All agent rules from the parent CLAUDE.md apply. Additional DS-specific
guidance:

- **Before implementing any subsystem:** cross-reference the target hardware
  behavior against GBATEK. Do not implement from memory. The DS has too many
  non-obvious quirks.
- **Do not base structure on the gba-emulator repo.** NDS is a fundamentally
  different machine (two CPUs, runtime VRAM banking, 3D engine, scheduler-
  driven timing). The GBA project is a source of *lessons*, not a template.
- **Quality review checklist** (in addition to standard review):
  - Incorrect bit manipulation (off-by-one in shifts, wrong mask widths)
  - Missing hardware edge cases (overflow, wraparound, saturation)
  - Wrong bus routing (ARM9 accessing ARM7-private I/O, or vice versa)
  - VRAM page table not rebuilt after a `VRAMCNT` write
  - Scheduler event scheduled in ARM7 cycles instead of ARM9 cycles
  - Raw `int32_t` in 3D pipeline code (should be `Fixed<I, F>`)
  - SDL include outside `src/frontend/`
  - Subsystem holding a pointer to another subsystem instead of `NDS&`
  - IMA ADPCM predictor off-by-one
  - Matrix multiplication order flipped (`MTX_MULT_*` is right-multiply)
- **For cross-cutting changes** (new I/O register, new scheduler event kind,
  new VRAM mapping target): use Plan mode first. These changes touch
  multiple subsystems and the blast radius warrants up-front design.
- **When debugging a CPU divergence:** capture an instruction trace and diff
  against melonDS or DeSmuME on the same ROM + same input. The first
  diverging instruction is almost always the bug.
- **When debugging a PPU glitch:** F2 → `XrayVramPage` to check bank mapping
  is what the game expects, then `XrayPpuPage` for `DISPCNT` / BG state,
  then `XrayOamPage` for sprite state. Work top-down from the mode bits.
- **When debugging a 3D glitch:** F2 → `XrayGeometryPage` to see the command
  FIFO and matrix stack state, then `XrayPolyPage` for finished polygons.
  `Fixed<I, F>` compile errors are a good sign you just avoided a
  shift-width bug.

### Agent usage (from parent CLAUDE.md)

- **senior-architect** for architectural analysis before coding a
  cross-cutting change.
- **precision-implementer** for implementing from the design spec.
- **quality-reviewer** for review of production-critical changes after
  writing code.
