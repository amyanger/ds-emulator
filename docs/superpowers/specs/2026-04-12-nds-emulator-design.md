# Nintendo DS Emulator — Design Spec

**Date:** 2026-04-12
**Author:** Arjun Myanger
**Status:** Approved (brainstorming phase complete, pending implementation plan)

---

## 0. Project summary

A from-scratch Nintendo DS emulator written in **C++20**, using **CMake** and
**SDL2**, targeted primarily at **Pokemon HeartGold / SoulSilver** with full
compatibility goals for the rest of the Gen 4 and Gen 5 Pokemon DS titles
(Diamond, Pearl, Platinum, Black, White, Black 2, White 2). First-class build
support for **Apple Silicon macOS**, portable by default.

The emulator is architected around **a single central event scheduler** (min-heap
of timed events), **two CPU cores with independent bus views**, a **software 3D
rasterizer** with a clean hand-off contract for a future hardware backend, and a
staged **in-emulator debug overlay** inspired by (but not copied from) the GBA
emulator's X-Ray mode. Boot is **direct-boot only**: no BIOS or firmware dumps
required — cart decryption is handled by a hardcoded KEY1 seed table and a
minimal HLE SWI layer.

This spec is the single source of truth for the high-level architecture. The
detailed implementation plan lives separately (see `writing-plans` output) and
describes per-phase work order, checklists, and verification steps.

---

## 1. Goals and non-goals

### Primary goals
- Run Pokemon HeartGold / SoulSilver from title screen through the main story,
  with working battles, saves, and sound.
- Run all six target Pokemon games (HG/SS, D/P/Pt, B/W, B2/W2) to at least the
  first gym with no visual regressions.
- Stay portable (C++20 + CMake + SDL2), buildable on macOS arm64, macOS x86_64,
  Linux, and Windows.
- Be readable by one developer — one hardware component per file, no
  cross-subsystem includes, no magic.
- Emit save states from day one; be deterministic given the same ROM + input
  stream.

### Stretch goals
- Full-speed gameplay on M-series Macs via NEON SIMD in the rasterizer and
  (optionally) a cached interpreter.
- GDB remote serial protocol stub on ARM9 for symbol-aware debugging of
  homebrew test ROMs.
- Hardware 3D backend (Metal or OpenGL 4.1) consuming the existing `Frame3D`
  contract without touching the geometry engine.

### Explicit non-goals
- DSi hardware, DSi-enhanced games, DSi Ware.
- Wireless / NDS-to-NDS multiplayer / Nintendo Wi-Fi Connection.
- GBA slot-2 compatibility (we are not shipping a GBA emulator inside the DS
  emulator).
- Bit-accurate cycle timing. We target "correct enough for the target games"
  and accept scanline-level granularity on the PPU.
- The firmware menu and user profile screen. Direct boot skips them entirely.

---

## 2. Top-level architecture

### 2.1 Shape

```
                 ┌─────────────────────────────────────┐
                 │              class NDS              │
                 │  (owns everything, drives frames)   │
                 └─┬───────────────────────────────────┘
                   │
     ┌─────────────┼───────────────────────────────────┐
     │             │                                   │
┌────▼────┐   ┌────▼────┐                         ┌────▼─────┐
│Scheduler│   │   CPUs  │                         │ Hardware │
│(min-heap│   │ARM9+ARM7│                         │Subsystems│
│ events) │   │         │                         │          │
└─────────┘   └────┬────┘                         └────┬─────┘
                   │                                   │
           ┌───────┴────────┐            ┌─────────────┼─────────────┐
           │                │            │             │             │
      ┌────▼────┐      ┌────▼────┐   ┌───▼──┐    ┌─────▼────┐   ┌────▼───┐
      │ARM9 Bus │      │ARM7 Bus │   │ PPU  │    │   3D     │   │ Audio  │
      │(MMU +   │      │(simple) │   │2 engs│    │Geo + Ras │   │16 chan │
      │TCM + $) │      │         │   │      │    │          │   │+capture│
      └────┬────┘      └────┬────┘   └──────┘    └──────────┘   └────────┘
           │                │
           └────┬───┬───────┘
                │   │
          ┌─────▼───▼────────────────────────────────────────┐
          │              Shared Hardware State               │
          │  Main RAM (4MB)  Shared WRAM (32KB, bankable)    │
          │  VRAM (656KB, 9 banks)  Palette  OAM  I/O regs   │
          │  Cart + KEY1/KEY2 engine   IPC (sync + FIFOs)    │
          │  RTC   Touch/Buttons/Mic                         │
          └───────────────────────────────────────────────────┘
```

### 2.2 Structural rules (non-negotiable)

1. **The scheduler is the clock.** Nothing advances time except
   `scheduler.run_until(t)`. Subsystems never sleep, wait, or run their own
   time loops.
2. **Two bus domains, not one.** ARM9 and ARM7 each own a `class Bus` that
   models that CPU's view of memory. Shared backing storage lives on `NDS`;
   the bus objects hold references.
3. **No subsystem holds a pointer to another subsystem.** All cross-subsystem
   interaction is a method call on `NDS&`. The dependency graph is a tree,
   always.
4. **I/O access is routed through the bus it was issued from.** ARM9 I/O and
   ARM7 I/O are different tables; the same address can mean different things
   on each core.
5. **Every subsystem implements `reset()`, `save_state()`, `load_state()`** as
   a uniform interface from day one. Save states are not a Phase 5 bolt-on.
6. **`class NDS` has no `<SDL.h>` anywhere in its transitive include graph.**
   NDS core is a platform-free static library; the frontend is a separate
   target.

### 2.3 The frame loop

```cpp
void NDS::run_frame() {
    // FrameEnd fires at the end of VBlank and sets `frame_done = true`
    // via its event handler in scheduler.cpp.
    scheduler.schedule_at(next_vblank_end_cycle, EventKind::FrameEnd);
    frame_done = false;
    while (!frame_done) {
        Cycle next = scheduler.peek_next();
        // Both cores take an ARM9-cycle target. Arm7::run_until internally
        // converts to ARM7 cycles (half-rate); see section 9.3.
        cpu9.run_until(next);
        cpu7.run_until(next);
        scheduler.advance_to(next);  // fires every event due by `next`
    }
}
```

The main loop is ~10 lines; everything else is subsystem behavior hanging off
the scheduler. The `frame_done` flag is a plain `bool` member of `NDS`, set
only by the `FrameEnd` event handler, and reset at the top of each
`run_frame` call.

---

## 3. CPUs and bus domains

### 3.1 ARM9 (67.03 MHz, ARMv5TE)

- 5-stage pipeline, emulated as fetch-decode-execute with the classic "PC is
  ahead by 8 (ARM) / 4 (Thumb)" trick.
- **8 KB I-cache, 4 KB D-cache**, 4-way set-associative. *Functionally stubbed
  from day one* (treat as always-hit-memory). Cache *timing* is modeled only
  in Phase 5+.
- **32 KB ITCM + 16 KB DTCM**, remappable via CP15. Modeled from day one —
  exception handlers and hot-path code live in ITCM.
- **CP15 coprocessor:** only the handful of registers actually used by target
  games are implemented. Enumerate and implement; no speculative coverage.
- **ISA additions over ARMv4T:** `CLZ`, `BLX`, `QADD`/`QSUB`, signed saturating
  math, extended `SMLA*`/`SMUL*`, `BKPT`, `PLD` (nop). ~15-20 new encodings.

### 3.2 ARM7 (33.51 MHz, ARMv4T)

- Same ISA as the GBA's ARM7TDMI. A fresh C++ rewrite, **not** a copy-paste
  from the GBA emulator repo.
- No caches, no TCM, no CP15.

### 3.3 Dispatcher seam

Each core owns a `Dispatcher` whose only job is `step(CpuCore&)`: fetch,
decode, execute one instruction, update cycles. The interpreter implementation
is `DecodeAndExecute`. In Phase 6+, this can be swapped for
`CachedBlockDispatcher` (decoded basic blocks) or `JitDispatcher` (native
dynarec) without touching the CPU core classes.

```cpp
class CpuCore {
public:
    virtual void run_until(Cycle target) = 0;
    virtual void request_irq(IrqSource) = 0;
    // ...
};

class Dispatcher {
public:
    virtual void step(CpuCore&) = 0;
};
```

One virtual call per **core** (not per instruction) — the dispatcher is
dereferenced once and the hot loop is a direct call.

### 3.4 Bus domains

`class Arm9Bus` and `class Arm7Bus` each own:

- A **256-entry page table** per access size (read8/16/32, write8/16/32),
  keyed on the top 8 bits of the address, pointing at either a direct memory
  base pointer or a function-pointer dispatch for I/O regions. Rebuilt in
  slices when VRAM banking or TCM mapping changes.
- **References to shared memory** living on `NDS` — no duplication.
- **Waitstate tables** for timing (ARM9 with TCM = 1 cycle, main RAM = 9
  cycles, VRAM = varies per bank).

Waitstates feed the CPU's cycle counter per access. Page table rebuild is
O(pages affected), not O(all 256), so a VRAM-only change costs ~5 writes.

### 3.5 Interrupt model

Each core has its own IE/IF/IME triple. Subsystems raise interrupts via
`nds.request_irq(Core::ARM9, IrqSource::VBlank)`. The core checks IRQs only
between instructions (never mid-decode) and only when IME is set. Standard ARM.

---

## 4. Memory map and VRAM banking

### 4.1 Physical storage

All real memory is owned by `class NDS` as flat byte arrays:

| Region        | Size         | Owner                 | Notes                                                       |
|---------------|--------------|-----------------------|-------------------------------------------------------------|
| Main RAM      | 4 MB         | `NDS::main_ram`       | Shared; ARM9 hits cached, ARM7 uncached                     |
| Shared WRAM   | 32 KB        | `NDS::shared_wram`    | Bankable per `WRAMCNT` (4 modes)                            |
| ARM7 WRAM     | 64 KB        | `NDS::arm7_wram`      | Private to ARM7                                             |
| VRAM banks    | 656 KB total | `NDS::vram[9]`        | 9 banks (A-I) with independent runtime mapping               |
| Palette RAM   | 2 KB         | `NDS::palette`        | 1 KB main + 1 KB sub                                        |
| OAM           | 2 KB         | `NDS::oam`            | 1 KB main + 1 KB sub                                        |
| ITCM / DTCM   | 32 + 16 KB   | `Arm9::{itcm,dtcm}`   | ARM9 private, CP15-remappable                                |
| Firmware      | 256 KB       | `NDS::firmware_bytes` | Synthesized; only header is meaningful for direct boot      |

### 4.2 VRAM banking

Bank sizes: A/B/C/D = 128 KB, E = 64 KB, F/G = 16 KB, H = 32 KB, I = 16 KB.

Each of `VRAMCNT_A..I` is an 8-bit I/O register that selects:
- Whether the bank is enabled
- MST (purpose): LCDC raw, main BG, main OBJ, main BG ext-palette, main OBJ
  ext-palette, sub BG, sub OBJ, sub BG ext-palette, sub OBJ ext-palette,
  texture, texture palette, ARM7 WRAM (C/D only)
- Offset within that purpose's address space

### 4.3 `class VramController`

Owns the 9 banks and publishes **per-consumer page tables** (cheat sheets):
```cpp
class VramController {
    std::array<uint8_t, 128 * 1024> bank_a;  // etc., 9 banks
    PageTable arm9_lcdc_view;
    PageTable arm7_wram_view;
    PageTable main_bg_view;
    PageTable main_obj_view;
    PageTable sub_bg_view;
    PageTable sub_obj_view;
    PageTable main_bg_extpal_view;   // 4 slots
    PageTable main_obj_extpal_view;  // 2 slots
    PageTable sub_bg_extpal_view;
    PageTable sub_obj_extpal_view;
    PageTable texture_view;          // 4 slots of 128 KB
    PageTable tex_palette_view;      // 6 slots of 16 KB
public:
    void write_vramcnt(int bank, uint8_t value);  // rebuilds affected tables only
    uint8_t  read_bg_byte(int engine, uint32_t addr) const;
    uint16_t read_texel(uint32_t addr) const;
    // ...
};
```

When `VRAMCNT_A` is written, only the page tables containing bank A (before or
after) are rebuilt. Consumers (PPU engines, texture unit, ARM9 bus, ARM7 bus)
read from their precomputed table — no per-access branching. This is the "cheat
sheet" model: compute once, read many.

**Overlap policy:** when two banks are mapped to the same purpose at the same
offset, the later-written bank wins. Real hardware behavior is technically
undefined; this is the accepted convention.

### 4.4 I/O regions

Three I/O windows:
- `0x0400_0000..0x0400_0FFF` — ARM9 I/O, heavily populated
- `0x0400_0000..0x0400_08FF` — ARM7 I/O (same address space, *different*
  registers depending on which core issued the access)
- `0x0410_0000..0x0410_FFFF` — IPC and cart interface

Each bus has its own I/O dispatch table keyed on `(addr & 0xFFFF) >> 2`.
There is no shared "one I/O table for both cores" — ambiguity in shared address
space is resolved at the bus layer, not in a big conditional.

---

## 5. 2D PPU (two engines)

### 5.1 One class, two instances

```cpp
class PpuEngine {
    bool is_main;           // true = main engine (engine A), false = sub engine (engine B)
    uint8_t framebuffer[192][256][4];  // RGBA
    // reads from VramController's per-consumer page tables
};

// NDS owns two:
PpuEngine ppu_main{.is_main = true};
PpuEngine ppu_sub {.is_main = false};
```

### 5.2 Main vs sub differences (controlled by the flag)

- Main engine has BG mode 6 (large bitmap) and can display the 3D engine's
  framebuffer on BG0; sub engine does not.
- Main engine has sprite bitmap extended modes; sub has a reduced set.
- Main reads from "main" VRAM views; sub reads from "sub" views.
- Main engine reads palette RAM lower half; sub reads upper half.

~80% of the engine code is shared. Feature differences are small `if
(is_main)` branches at the top of a handful of paths.

### 5.3 Rendering model

**Scanline-based.** At the start of each scanline:
1. HBlank-start event fires
2. `PpuEngine::render_scanline(y)` is called for main and sub
3. Walks the 4 BG layers + sprite layer, composites with blending/windows,
   writes to `framebuffer[y]`
4. On scanline 191, VBlank IRQ fires on both CPUs

### 5.4 3D on BG0

Main engine BG0 can be configured to display the 3D engine's frame instead of
a tile map. The 3D engine renders a full-frame `Frame3D` struct in a deferred
pass at `SWAP_BUFFERS`; the main PPU's compositor reads from that buffer when
BG0 is enabled in 3D mode. This is the single seam between 2D and 3D.

### 5.5 Output

Two 256x192 framebuffers per frame. `POWCNT1.bit15` tells the frontend which
framebuffer is "top" vs "bottom"; the PPU engines are agnostic.

### 5.6 Accepted inaccuracies

- Per-dot rendering is not implemented. Mid-scanline register writes are
  honored only at scanline start. None of the target Pokemon games rely on
  mid-scanline tricks in ways that matter.
- Sprite evaluation is not hardware-timed. We evaluate all sprites for a
  scanline in one pass, not 2-cycles-per-sprite.

---

## 6. 3D engine (geometry + rasterizer)

The DS 3D pipeline is two physically separate pieces of hardware. The
emulator mirrors this with two classes.

### 6.1 `class GeometryEngine`

**Input:** the 256-entry geometry command FIFO fed by the ARM9 via `GXFIFO`
register writes, DMA, or direct writes to `GXCMD_*`.

**State:**
- **4 matrix stacks:** projection (depth 1), modelview (depth 31), direction
  (depth 31, coupled to modelview), texture (depth 1). Each stack has a
  pointer and a "in-use bitmask" matching hardware.
- Current 4x4 matrix for each stack (20.12 fixed-point).
- Current vertex color, texture coord, normal.
- **Vertex RAM:** up to 6144 transformed vertices (double-buffered).
- **Polygon RAM:** up to 2048 polygon descriptors (double-buffered).
- Material parameters (diffuse/ambient/specular/emission), toon table
  pointer, 4 light sources.
- In-progress polygon attributes (mode, culling, fog flag, depth test, alpha,
  polygon ID).

**Per-command behavior:** state commands update state; vertex commands build
polygons. When a polygon is complete the engine:
1. Multiplies vertices by the current modelview x projection
2. Clips against the 6 frustum planes (up to ~6 new vertices)
3. Perspective divides
4. Viewport-transforms to screen space
5. Writes the polygon into polygon RAM with vertex indices into vertex RAM

**Key correctness rules:**
- Matrix stack overflow sets a dirty bit, not an exception.
- `MTX_MULT_*` is current = current x argument (right-multiply).
- `VTX_XY`/`VTX_XZ`/`VTX_YZ`/`VTX_DIFF` reuse the previous vertex's missing
  coordinate. The engine must track "last vertex."
- Winding/culling is decided post-projection in screen space.
- `SWAP_BUFFERS` is the frame boundary: vertex/polygon RAM double-buffers
  swap, the rasterizer is triggered, the next frame can begin appending to
  the new "filling" buffer.

**FIFO back-pressure:** if the FIFO has more than 128 entries, ARM9 writes to
`GXFIFO` stall. Modeled via a scheduler event that releases the stall when
the FIFO drains.

### 6.2 `class RenderingEngine` (software rasterizer)

**Input:** the snapshot produced by `GeometryEngine` at `SWAP_BUFFERS`.

**Approach:** deferred scanline rasterization.
1. Sort polygons by DS-specific rules: translucent after opaque, Y-sort within
   each group, preserve submission order within a Y bucket.
2. For each scanline (0..191):
   - Find polygons whose Y range includes this scanline (precomputed bucket).
   - Walk each polygon's edges (half-space / edge-function rasterization).
   - Perspective-correctly interpolate Z, RGB, texcoord, fog factor per pixel.
   - Depth-test against the 24-bit depth buffer (W or Z per `SWAP_BUFFERS`
     flag).
   - Texture fetch via `VramController::read_texel`.
3. Full-frame post-passes: **edge marking**, **fog**, **toon/highlight**,
   **alpha blend** (translucent polys in back-to-front order).
4. Output into `Frame3D`.

### 6.3 `Frame3D` — the hand-off contract

```cpp
struct Frame3D {
    uint16_t color[192][256];  // BGR555 or ABGR1555
    uint32_t attr [192][256];  // polygon ID + edge flag + fog flag
    uint32_t depth[192][256];  // 24-bit
};
```

The rasterizer produces a `Frame3D`. The main PPU's BG0 consumes it. A future
hardware 3D backend (Metal, OpenGL 4.1, Vulkan) would produce the same struct
and nothing else in the emulator would change.

### 6.4 Fixed-point

A templated `Fixed<I, F>` struct with overloaded `+ - * /`, sign-preserving
shifts, and explicit conversion operators. Used for vertex coords (4.12 and
20.12), matrices (20.12), texture coords (12.4), light directions (1.9), fog
densities (1.7). The type system enforces format correctness — mixing
formats without explicit conversion is a compile error. This catches the
entire category of "my polygon renders in the wrong place" bugs at build time.

### 6.5 Threading

Geometry and rendering are serialized on the main emulator thread, driven by
scheduler events. No 3D worker thread. If parallelization ever happens, it
happens inside the rasterizer's inner loop, not via thread handoff between
subsystems.

---

## 7. Audio (SPU)

### 7.1 `class Spu`

```cpp
class Spu {
    struct Channel {
        enum Format { PCM8, PCM16, ADPCM, PSG, Noise };
        Format   format;
        uint32_t sample_addr;
        uint32_t length;
        uint32_t loop_start;
        uint8_t  volume, pan;
        bool     hold, enable;
        uint32_t pos;              // fractional position
        int32_t  adpcm_accumulator;
        int32_t  adpcm_step;
    };
    std::array<Channel, 16> channels;
    int16_t mix_buffer[1024 * 2];  // stereo
    // 2 capture units
};
```

### 7.2 Channel formats

- Channels 0-7: PCM8 / PCM16 / ADPCM (IMA variant)
- Channels 8-13: add PSG square waves (8 duty cycle settings)
- Channels 14-15: add LFSR noise

One switch on `Channel::format` dispatches to the right sample generator per
channel per mix step.

### 7.3 Mixing model

SPU runs as a scheduler event. Every N cycles (chosen so we produce 2048
samples per frame at ~32.7 kHz internal rate), `Spu::tick()`:
1. For each enabled channel, advance fractional position, fetch next sample
   (PCM/ADPCM/PSG/noise), multiply by channel volume, pan L/R.
2. Sum the 16 channels into master stereo.
3. Apply master volume and write to the output ring buffer.

### 7.4 Sound capture

Two capture units record channel output back into main RAM. Used by Pokemon
title-screen reverb and some battle effects. Implemented in Phase 4.

### 7.5 Ownership model

- `class Spu` lives on `NDS`, not inside `Arm7`.
- `Arm7Bus` I/O dispatch for `0x0400_0400..0x0400_04FF` routes to
  `nds.spu.io_read/write(addr, val)`.
- `Arm9Bus` has no I/O entries for the SPU region — ARM9 cannot access it,
  matching hardware.

### 7.6 SDL integration

SDL opens a 48 kHz stereo F32 output stream. A callback pulls samples from a
lock-free SPSC ring buffer on the SDL audio thread; the scheduler's SPU tick
event pushes samples in on the emulator thread.

### 7.7 Relationship to SDAT / sequencer data

Pokemon games don't feed raw samples to the SPU. They run a software
sequencer (SSEQ files parsed by game code) on the ARM7 that generates the
SPU register writes in real time. The emulator does not need to understand
SDAT — the game does all that work and the SPU just has to respond correctly
to whatever writes land on it. This is a significant simplification.

---

## 8. Cartridge, saves, direct boot

### 8.1 `class Cartridge`

```cpp
class Cartridge {
    std::vector<uint8_t> rom;
    NdsHeader           header;
    SaveChip            save;
    CartProtocol        protocol;   // slot-1 bus state machine
public:
    bool load(fs::path);
    uint32_t read_protocol(uint32_t);
    void     write_protocol(uint32_t);
};
```

`CartProtocol` is a small state machine handling the slot-1 command sequence:
`0x9F` dummy, `0x00` header read, `0x3C` KEY1 activate, `0x40`/`0x10`
encrypted reads, `0xB7` encrypted data read, etc. ~200 lines, documented on
GBATEK.

### 8.2 Save chips

| Family | Sizes           | Used by              |
|--------|-----------------|----------------------|
| EEPROM | 512 B / 8 K / 64 K | HG/SS, Platinum, others |
| FLASH  | 256 K / 512 K / 1 M | D/P, B/W, B/W2, others |
| NAND   | (not used by targets) | Stub only |

Save type is auto-detected from `header.game_code` via a lookup table. Save
files live at `saves/<game_code>.sav`. Writes are dirty-block tracked and
periodically flushed (1-second timer + on close). No full dump per write.

### 8.3 Direct boot

When `NDS::boot_cart(path)` is called:
1. Parse the header.
2. Copy ARM9 binary to main RAM at `header.arm9_ram_address`.
3. Copy ARM7 binary similarly to its destination.
4. Set initial register state as if post-firmware: SP per mode (GBATEK
   values), PC = entry points, CPSR = supervisor, IRQs disabled.
5. Initialize ~30 I/O registers to their post-firmware values.
6. Set up the **KEY1 table**: use a hardcoded 0x1048-byte seed (same for every
   DS, documented), then mix the game code in to produce a per-game schedule
   (~150 lines total in `cart_keys.cpp`).
7. Synthesize RTC date/time from the host clock.
8. Jump both CPUs to their entry points.

No `bios9.bin`, `bios7.bin`, or `firmware.bin` is required. The firmware menu,
user profile, and touchscreen calibration are skipped entirely — games
initialize what they need on their own when direct-booted.

### 8.4 HLE BIOS SWIs

Implemented in `bios9_hle.cpp` / `bios7_hle.cpp`. Enumerable set (~20 total)
including `CpuSet`, `CpuFastSet`, `DivSigned`, `Sqrt`, `GetCRC16`,
`UnCompBitUnpack`, `LZ77UnCompVRAM`, `HuffUnComp`, `WaitByLoop`, `IntrWait`,
`VBlankIntrWait`. Each reads arguments from registers, performs the operation,
writes results back. Same pattern as the GBA HLE BIOS but scoped to the
actually-used set.

### 8.5 Cart DMA ("card DMA")

Modeled as scheduler events: start the transfer, fire completion IRQ after N
cycles. Same pattern as the rest of the DMA system.

---

## 9. Scheduler internals

### 9.1 Data

```cpp
using Cycle = uint64_t;   // ARM9 cycles since power-on

struct Event {
    Cycle    when;
    uint32_t kind;
    uint64_t payload;
    uint64_t id;          // monotonic, for cancel()
};

class Scheduler {
    std::vector<Event>          heap;       // min-heap by (when, id)
    std::unordered_set<uint64_t> cancelled; // tombstones
    Cycle                       now = 0;
    uint64_t                    next_id = 1;
public:
    uint64_t schedule_in(Cycle delta, EventKind kind, uint64_t payload = 0);
    uint64_t schedule_at(Cycle when,  EventKind kind, uint64_t payload = 0);
    void     cancel(uint64_t id);
    Cycle    now() const;
    void     set_now(Cycle t);
    Cycle    peek_next();                         // kNoEvent if empty
    bool     pop_due(Cycle target, Event& out);   // pops one due event
};
```

> The scheduler is a pure data structure. It has no reference to `NDS`, no
> callback, and no dispatch switch. Event handling lives in
> `NDS::on_scheduler_event`, called from the inner drain loop of
> `NDS::run_frame`. This keeps the scheduler trivially unit-testable and
> preserves rule #3 (no subsystem points back at another subsystem).

Flat `std::vector` maintained as a min-heap via `std::push_heap` /
`std::pop_heap`. **Not `std::priority_queue`** — that hides the underlying
container and makes cancellation impossible.

### 9.2 Cancellation via tombstones

`cancel(id)` marks the event id in a small `std::unordered_set`. `advance_to`
skips popped events whose id is in the set. The set is cleared when empty.
Typical steady-state size is 0-3. Standard technique.

### 9.3 Time model

- `now` is ARM9 cycles, 64-bit, monotonic.
- ARM7 runs at exactly half the ARM9 clock. `Arm7::run_until(arm9_target)`
  runs until `arm7_cycles == arm9_target / 2`.
- All scheduled events are in ARM9 cycles.

### 9.4 Event kinds

A single enum, dispatched via one switch in `nds.cpp::on_scheduler_event`.
The scheduler itself never sees `EventKind` as anything other than an
opaque ordering key.

```
FrameEnd,
HBlankStart, HBlankEnd, VBlankStart, VBlankEnd,
Timer9_0..3, Timer7_0..3,
Dma9_0..3, Dma7_0..3,
CartXferDone,
GeometryFifoReady, GxBufferSwapReady,
SpuSample,
RtcTick,
IpcSyncChanged,
```

No virtual callbacks on events. One switch. Adding a new event kind is
adding an enum value and a case.

### 9.5 Latency

Events that "take N cycles to propagate" on real hardware (DMA start, IRQ
delivery) are scheduled N cycles in the future rather than fired immediately.
This absorbs most naive-zero-latency inaccuracy.

### 9.6 Save state / determinism

The scheduler serializes cleanly: `{now, next_id, heap, cancelled_set}`.
Combined with every subsystem's `save_state` / `load_state`, the emulator is
deterministic — same ROM + same input stream produces the same output. This
is the foundation for rewind, netplay, TAS tools, and regression tests.

### 9.7 Debug hooks

`Scheduler::set_trace_sink(std::function<void(const Event&)>)` plugs a logger
into debug builds. The X-Ray scheduler page reads from this stream.
`Scheduler::dump_pending()` helper for interactive inspection.

### 9.8 Performance

The `peek_next -> run_until -> advance_to` loop is the hottest code after the
instruction decoders. Keeping events rare (never per-instruction) keeps
scheduler overhead at 1-3% of total runtime.

---

## 10. Frontend and debug overlay

### 10.1 `class Frontend`

Owns the SDL2 window, renderer, textures, audio device, and input state.
Talks to `NDS` only through well-defined entry points:

```cpp
class Frontend {
    SDL_Window*   window;
    SDL_Renderer* renderer;
    SDL_Texture*  tex_top;  // 256x192 ARGB8888
    SDL_Texture*  tex_bot;  // 256x192 ARGB8888
    SDL_AudioDeviceID audio;
    RingBuffer<int16_t, 8192> audio_ring;
    InputState    input;
    NDS&          nds;
public:
    void pump_events();
    void present_frame();
    void push_audio(const int16_t*, size_t);
};
```

### 10.2 Window layout

Two 256x192 framebuffers stacked vertically with a configurable inter-screen
gap. `POWCNT1.bit15` selects top vs bottom at present time.

Window scale is a CLI flag (`--scale 2`, `--scale 3`). HiDPI is enabled by
default; Retina scaling is automatic on macOS via
`SDL_WINDOW_ALLOW_HIGHDPI`.

### 10.3 Input mapping

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
| Mic (blow)        | M                |

Remappable via `keybinds.cfg` (one line per binding). MFi / Xbox / DualSense
controllers supported via SDL2's game controller API.

### 10.4 Touch

The touchscreen is the sub screen. On mouse down within the sub-screen rect,
the frontend writes scaled coordinates into `NDS::touch_state`, which ARM7
reads via the SPI bus I/O registers (TSC2046 model). Single path: Frontend ->
NDS -> ARM7 I/O.

### 10.5 Audio output

48 kHz stereo F32 output stream. SDL callback pulls from the ring buffer on
the audio thread; SPU pushes on the emulator thread. Lock-free SPSC.

### 10.6 Debug overlay (`DsXray`)

Built with `ENABLE_XRAY=ON` by default, stripped out entirely with
`ENABLE_XRAY=OFF`. Toggled with **F2**.

**Page architecture.** A collection of pages cycled with **Tab**. Each page is
a class with a `draw(const NDS&, XrayCanvas&)` method that reads subsystem
state and draws into an overlay canvas alpha-blended over the normal output.

```cpp
class XrayPage {
public:
    virtual const char* name() const = 0;
    virtual void draw(const NDS&, XrayCanvas&) = 0;
    virtual void handle_key(SDL_Keycode) {}
};
```

**Pages by phase:**

- Phase 1: `XrayCpuPage`, `XrayBusPage`
- Phase 2: `XrayVramPage`, `XrayPpuPage`, `XrayOamPage`, `XrayPalettePage`
- Phase 3: `XrayAudioPage`
- Phase 4: `XrayGeometryPage`, `XrayPolyPage`
- Phase 5+: `XraySchedulerPage`, `XrayIpcPage`

**Overlay canvas** is a software-drawn 256x384 RGBA buffer with tiny
primitives (rect, text, line, graph), alpha-blended over the PPU output at
present time. Embedded constexpr bitmap font.

### 10.7 Direct F-keys (debug builds)

| Key | Action                                         |
|-----|------------------------------------------------|
| F1  | Dump both CPUs' state to stderr                |
| F2  | Toggle X-Ray overlay                            |
| F3  | Step one instruction (ARM9 priority)            |
| F4  | Step one scanline                              |
| F5  | Step one frame                                 |
| F6  | Toggle ARM9 instruction trace                  |
| F7  | Toggle ARM7 instruction trace                  |
| F8  | Dump main RAM to `dump_mainram.bin`             |
| F9  | Dump VRAM (all banks) to `dump_vram.bin`        |
| F10 | Save state to `savestate_0.bin`                |
| F11 | Load state from `savestate_0.bin`               |

### 10.8 Platform isolation

`#include <SDL.h>` is only legal under `src/frontend/`. The NDS core builds
as a static library with zero SDL dependency. This enables future Qt,
libretro, or headless frontends without touching core code. **This rule is
non-negotiable.**

### 10.9 Headless mode

`--headless --frames N --dump-hash out.txt` runs the emulator for N frames
with no window, hashes the final framebuffer, and writes the hash. Used for
regression tests. Same binary; `Frontend` simply isn't instantiated and the
NDS runs against a null audio sink.

---

## 11. Build targets and Apple Silicon

### 11.1 Default build

Portable C++20 + SDL2. Builds on macOS arm64, macOS x86_64, Linux, and
Windows unchanged. No `#ifdef __APPLE__` in the default code path.

### 11.2 `DS_TARGET_APPLE_SILICON=ON`

Optional CMake flag. When set:
- `-mcpu=apple-m1` (or `apple-a14` / `apple-m2` variant)
- NEON SIMD rasterizer paths compiled in (`simd_neon.cpp`)
- `QOS_CLASS_USER_INTERACTIVE` thread attribute on the emulator thread via
  `pthread_set_qos_class_self_np` (keeps the emulator thread off efficiency
  cores)
- Entitlements file placeholder (`entitlements.plist`) wired into CMake's
  `install` target for ad-hoc codesigning
- `com.apple.security.cs.allow-jit` entitlement prepared (unused until a JIT
  backend is written in Phase 6+)

### 11.3 Portability rule

All Apple-specific code is confined to:
- `src/frontend/platform/macos.cpp`
- `src/gpu3d/simd_neon.cpp`

Both files compile empty (or are excluded) when `DS_TARGET_APPLE_SILICON=OFF`.
No conditional compilation outside these two files.

### 11.4 CMake targets

```
libds_core.a       -- everything under src/ except frontend/ and main.cpp
libds_frontend.a   -- src/frontend/
ds_emulator        -- main.cpp + libds_frontend.a + libds_core.a
ds_tests           -- tests/ + libds_core.a (no frontend)
```

The `ds_core` static lib has zero SDL2 dependency and is the reason headless
unit tests run in milliseconds.

---

## 12. Project layout

```
ds-emulator/
├── CMakeLists.txt
├── CLAUDE.md
├── README.md
├── LICENSE
├── .clang-format
├── entitlements.plist
├── cmake/
│   ├── CompilerWarnings.cmake
│   ├── AppleSilicon.cmake
│   └── Sanitizers.cmake
├── docs/
│   ├── architecture.md
│   └── superpowers/specs/2026-04-12-nds-emulator-design.md
├── include/
│   └── ds/
│       ├── common.hpp
│       ├── fixed.hpp
│       ├── ring_buffer.hpp
│       └── span_ext.hpp
├── src/
│   ├── main.cpp
│   ├── nds.hpp / nds.cpp
│   ├── scheduler/
│   │   ├── scheduler.hpp
│   │   └── scheduler.cpp
│   ├── cpu/
│   │   ├── cpu_core.hpp
│   │   ├── dispatcher.hpp
│   │   ├── arm9/
│   │   │   ├── arm9.hpp / arm9.cpp
│   │   │   ├── arm9_decode.cpp
│   │   │   ├── arm9_thumb.cpp
│   │   │   ├── cp15.hpp / cp15.cpp
│   │   │   └── tcm.cpp
│   │   ├── arm7/
│   │   │   ├── arm7.hpp / arm7.cpp
│   │   │   ├── arm7_decode.cpp
│   │   │   └── arm7_thumb.cpp
│   │   └── bios/
│   │       ├── bios9_hle.cpp
│   │       └── bios7_hle.cpp
│   ├── bus/
│   │   ├── arm9_bus.hpp / arm9_bus.cpp
│   │   ├── arm7_bus.hpp / arm7_bus.cpp
│   │   ├── io_regs.hpp
│   │   └── wram_control.cpp
│   ├── memory/
│   │   ├── main_ram.cpp
│   │   └── vram/
│   │       ├── vram_controller.hpp
│   │       ├── vram_controller.cpp
│   │       └── vram_page_tables.cpp
│   ├── dma/
│   │   └── dma_controller.cpp
│   ├── timer/
│   │   └── timer_bank.cpp
│   ├── ipc/
│   │   ├── ipc_sync.cpp
│   │   └── ipc_fifo.cpp
│   ├── interrupt/
│   │   └── irq_controller.cpp
│   ├── ppu/
│   │   ├── ppu_engine.hpp / ppu_engine.cpp
│   │   ├── render_bg_text.cpp
│   │   ├── render_bg_affine.cpp
│   │   ├── render_bg_extended.cpp
│   │   ├── render_bg_bitmap.cpp
│   │   ├── render_sprites.cpp
│   │   ├── compositor.cpp
│   │   └── palette.cpp
│   ├── gpu3d/
│   │   ├── geometry_engine.hpp / geometry_engine.cpp
│   │   ├── matrix_stack.cpp
│   │   ├── vertex_pipeline.cpp
│   │   ├── clipper.cpp
│   │   ├── gx_fifo.cpp
│   │   ├── rendering_engine.hpp / rendering_engine.cpp
│   │   ├── rasterizer.cpp
│   │   ├── interpolator.cpp
│   │   ├── texture_unit.cpp
│   │   ├── post_edge.cpp
│   │   ├── post_fog.cpp
│   │   ├── post_toon.cpp
│   │   ├── frame3d.hpp
│   │   └── simd_neon.cpp
│   ├── spu/
│   │   ├── spu.hpp / spu.cpp
│   │   ├── channel_pcm.cpp
│   │   ├── channel_adpcm.cpp
│   │   ├── channel_psg.cpp
│   │   ├── channel_noise.cpp
│   │   └── capture.cpp
│   ├── cartridge/
│   │   ├── cartridge.hpp / cartridge.cpp
│   │   ├── nds_header.cpp
│   │   ├── cart_protocol.cpp
│   │   ├── key1.cpp
│   │   ├── save_chip.hpp
│   │   ├── save_eeprom.cpp
│   │   ├── save_flash.cpp
│   │   ├── save_detect.cpp
│   │   └── card_dma.cpp
│   ├── rtc/
│   │   └── rtc.cpp
│   ├── input/
│   │   ├── keypad.cpp
│   │   ├── touchscreen.cpp
│   │   └── mic.cpp
│   ├── frontend/
│   │   ├── frontend.hpp / frontend.cpp
│   │   ├── audio_ring.cpp
│   │   ├── input_config.cpp
│   │   ├── platform/
│   │   │   ├── macos.cpp
│   │   │   └── generic.cpp
│   │   └── xray/
│   │       ├── xray.hpp / xray.cpp
│   │       ├── xray_canvas.cpp
│   │       ├── xray_font.hpp
│   │       ├── page_cpu.cpp
│   │       ├── page_bus.cpp
│   │       ├── page_vram.cpp
│   │       ├── page_ppu.cpp
│   │       ├── page_oam.cpp
│   │       ├── page_palette.cpp
│   │       ├── page_geometry.cpp
│   │       ├── page_poly.cpp
│   │       ├── page_scheduler.cpp
│   │       ├── page_ipc.cpp
│   │       └── page_audio.cpp
│   └── savestate/
│       ├── savestate.hpp
│       └── savestate.cpp
├── tests/
│   ├── CMakeLists.txt
│   ├── regression/
│   │   └── headless_frame_hash.cpp
│   └── unit/
│       ├── fixed_test.cpp
│       ├── scheduler_test.cpp
│       ├── clipper_test.cpp
│       └── vram_controller_test.cpp
├── roms/              # .gitignored
├── saves/             # .gitignored
└── build/             # .gitignored
```

### Layout rules

- One hardware component per file. When a file exceeds ~800 lines, split.
- No cross-subsystem includes. `ppu/` never includes from `cpu/`.
- Per-subsystem public headers live next to their source.
- `include/ds/` is reserved for truly cross-cutting types.
- `ds_core` has zero SDL dependency; `ds_frontend` is the only SDL-aware
  target.

---

## 13. Phased roadmap

Every phase has a concrete boot milestone against a real Pokemon ROM. No
phase-gate is abstract.

### Phase 0 — Scaffolding (1-2 days)
CMake + three targets; `NDS`, `Scheduler`, `Arm9`/`Arm7` skeletons;
`common.hpp`, `fixed.hpp`, `ring_buffer.hpp`; `class Frontend` opening a
window; clang-format + warnings; `AppleSilicon.cmake`.
**Milestone:** `./ds_emulator` opens a two-screen window showing a test
pattern and closes cleanly on Esc.

### Phase 1 — CPUs, buses, memory, IRQs, IPC (3-5 weeks)
Full ARMv5TE + ARMv4T decoders and executors; CP15 + TCM; Arm9Bus + Arm7Bus
with page tables; raw main/shared/VRAM storage; `VramController` skeleton (no
routing yet); 8-channel DMA; 8 timers; 2 IRQ controllers; IPC sync + FIFO;
minimum HLE SWIs; direct-boot path with KEY1 seed + per-game schedule;
scheduler and `run_frame`; `XrayCpuPage` + `XrayBusPage`.
**Milestone:** Pokemon HeartGold executes past cart boot, reaches the game's
ARM9 entry point, runs several frames without crashing. No pixels yet —
verification via instruction trace.

### Phase 2 — 2D PPU and first pixels (2-4 weeks)
`class PpuEngine` (main + sub instances); all 4 BG modes (text, affine,
extended, bitmap); full sprite engine (all sizes, 1D/2D mapping, affine);
compositor with windows + blending + master brightness; real VRAM bank
routing for all 2D consumers; PPU event scheduling (HBlank/VBlank);
framebuffer upload via frontend; `XrayVramPage`, `XrayPpuPage`, `XrayOamPage`,
`XrayPalettePage`.
**Milestone:** HeartGold **title screen renders correctly** with the Lugia
animation. "New Game" advances to the Professor Elm intro, which also renders.

### Phase 3 — Input, audio, saves (2-3 weeks)
Keypad I/O, TSC2046 touchscreen, lid, mic stub; SPU with all 16 channels,
all formats, mixing, sound capture; `XrayAudioPage`; cart save (EEPROM +
FLASH) with auto-detect and dirty-block dump; SDL audio output;
`keybinds.cfg`; MFi/Xbox/PlayStation controller support.
**Milestone:** HeartGold from title to walking around New Bark Town with
music and SFX. Save, close, reopen, continue works.

### Phase 4 — 3D engine (5-8 weeks, largest phase)
**4a.** `GeometryEngine` — command FIFO, matrix stacks, lighting, vertex
pipeline, clipper, viewport transform.
**4b.** `RenderingEngine` (software) — scanline rasterizer, Z-buffer,
perspective-correct interpolation, texture unit.
**4c.** Post-processing — edge marking, fog, toon, alpha blending.
**4d.** VramController texture + texture-palette views; `Frame3D` hand-off.
**4e.** `XrayGeometryPage` + `XrayPolyPage`.
**Milestone (4a-4b):** HeartGold's first wild Pokemon battle renders
correctly (both mons as 3D models, platforms, HP bars, animations).
**Milestone (full phase):** HG/SS playable to the first gym including
battles. Black/White title screen renders with fog.

### Phase 5 — Polish, accuracy, save states (3-5 weeks)
Full save state serialization; rewind (rolling ring of save states);
scanline-accuracy bug fixes; missing SWI handlers; full RTC implementation;
ARM9 cache timing model; tuned card DMA latencies; DPPt / BW / BW2
compatibility pass.
**Milestone:** All six target Pokemon games boot, intro plays, first gym
reachable, playable 30+ minutes without crashes or obvious regressions.

### Phase 6 — Performance and tooling (ongoing)
NEON SIMD rasterizer; optional GDB stub; headless regression harness;
cached-interpreter dispatcher; optional aarch64 dynarec; optional hardware 3D
backend (Metal / OpenGL 4.1) consuming `Frame3D`; `XraySchedulerPage` +
`XrayIpcPage`.
**Milestone:** HG/SS at full speed on M-series Macs; full main story
playable end to end.

### Timeline estimate

- Phases 0-5: **18-30 weeks** of focused solo work.
- Phase 6: indefinite polish.
- Whole-project realistic estimate: **4.5-7.5 months to all-six playable.**

### Phase-gate rule

**Do not start Phase N+1 until Phase N's milestone ROM runs reliably.**
Skipping ahead to chase the fun parts is how solo emulators die. If Phase 4
starts before Phase 2's title screens render correctly, 3D bugs will be
impossible to isolate because you won't know whether the bug is in the 3D
pipeline or in the 2D compositor layer it lands on.

---

## 14. Testing strategy

### 14.1 Unit tests (`ds_tests` target)

- `fixed_test.cpp` — `Fixed<I, F>` arithmetic, conversions, sign, saturation.
- `scheduler_test.cpp` — min-heap correctness, cancellation, event ordering,
  edge cases (same-cycle events, cancelled-then-rescheduled).
- `clipper_test.cpp` — frustum clipping of degenerate cases, zero-area
  polygons, all-outside, all-inside.
- `vram_controller_test.cpp` — bank remapping on every VRAMCNT variant,
  overlap policy, view consistency after cascaded changes.

All unit tests link against `ds_core` only — no SDL, no window, run in
milliseconds.

### 14.2 Regression tests (Phase 6+)

Headless harness: given a ROM + a recorded input stream + a target frame
number, run N frames and hash the final framebuffer. Compare against a stored
hash. Any change flags a regression.

### 14.3 Reference comparison

Instruction trace capture, compare against melonDS or DeSmuME output at
branch points when debugging CPU correctness issues. Same technique used on
the GBA project with mGBA traces.

### 14.4 Test ROMs

- **armwrestler-nds** — ARM instruction correctness (ARMv5TE variant).
- **gbatek-armtest** — timing and edge cases.
- **fatfs demo** — sanity check for cart protocol.
- **mollusk/ndsexamples** — hardware feature demos.
- **melonDS test rom suite** — scheduler/timing validation.

Place under `roms/tests/`.

---

## 15. References

- **GBATEK** (primary hardware ref): https://problemkaputt.de/gbatek.htm
- **GBATEK Markdown fork**: https://mgba-emu.github.io/gbatek/
- **melonDS source** (reference emulator): https://github.com/melonDS-emu/melonDS
- **DeSmuME source**: https://github.com/TASEmulators/desmume
- **Copetti NDS Architecture**: https://www.copetti.org/writings/consoles/nintendo-ds/
- **awesome-ndsdev**: https://github.com/pret/awesome-nds-dev
- **KEY1 / KEY2 encryption**: https://problemkaputt.de/gbatek.htm#dscartridgeencryption
- **TSC2046 touchscreen controller datasheet** (for SPI touch model)

---

## 16. Glossary

- **ARM9 / ARM7**: the two CPU cores in the DS. ARM9 is the main CPU
  (ARMv5TE, 67 MHz, with caches + TCM). ARM7 is the secondary CPU (ARMv4T,
  33 MHz, no caches), mostly handles audio, input, wifi.
- **TCM**: Tightly-Coupled Memory. ARM9-private SRAM that runs at CPU speed,
  remappable via CP15. Split into ITCM (instruction) and DTCM (data).
- **CP15**: ARM coprocessor 15, the system control coprocessor. Configures
  caches, TCMs, MMU.
- **VRAM banks A-I**: 9 physical SRAM chips on the NDS that can be mapped at
  runtime to different purposes (BG, OBJ, texture, ARM7 WRAM, etc.).
- **IPC**: Inter-Processor Communication. Sync registers + two 64-byte FIFOs
  for ARM9/ARM7 to talk to each other.
- **GXFIFO**: The 3D geometry command FIFO. ARM9 writes here; geometry engine
  reads.
- **SWAP_BUFFERS**: The geometry command that ends a 3D frame and triggers
  the rasterizer.
- **KEY1 / KEY2**: Cart encryption schemes. KEY1 protects the Secure Area
  (first 16 KB of ARM9 binary); KEY2 protects normal cart data transfers.
- **SDAT / SSEQ**: Nintendo's sound data / sequence format. The ARM7 runs a
  software sequencer that parses SSEQ files and generates SPU register
  writes — the emulator doesn't need to understand SDAT.
- **Direct boot**: Booting straight into a game by reading the cart header
  and jumping to the ARM9/ARM7 entry points, skipping the firmware menu
  entirely.
- **HLE**: High-Level Emulation. Implementing a system call or hardware
  feature in the host language rather than by running the original firmware
  code.
- **Deferred rasterization**: Collecting all polygons for a frame first, then
  rendering them in one pass at the end. Matches DS hardware and enables
  correct edge marking / fog / toon passes.
- **Fixed-point**: Representing fractional numbers as scaled integers. DS 3D
  math uses several fixed-point formats (4.12, 20.12, 12.4) because the
  hardware has no FPU.
- **Scanline-based rendering**: Producing one horizontal line of pixels at a
  time, rather than one pixel at a time. Standard for 2D emulator PPUs.
- **Software rasterizer**: CPU-only triangle rendering. Slower than a GPU
  but perfectly predictable and matches hardware exactly.
