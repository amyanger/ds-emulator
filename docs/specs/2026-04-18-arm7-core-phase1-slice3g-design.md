# ARM7 Core — Phase 1, Slice 3g Design

**Date:** 2026-04-18
**Slice:** ARM7 BIOS HLE — callback decompressors (LZ77 Vram, Huffman, RLE Vram) + re-entrant guest-call trampoline
**Status:** proposed
**Prior slice:** 3f (ReadNormal decompressors) — landed through `d26bf68`; 55 CTest binaries green
**Next slice:** 3h (proposed: ARM7 BIOS miscellaneous / GetVolumeTable / GetPitchTable / sound SWIs 0x1A-0x1D cleanup), still to be scoped

---

## 1. Summary

Slice 3f shipped the three ReadNormal decompressor variants (`0x10` BitUnPack,
`0x11` LZ77 Wram, `0x14` RLE Wram) that read source data directly from a
memory pointer. This slice ships the three ReadByCallback variants
(`0x12` LZ77 Vram, `0x13` Huffman, `0x15` RLE Vram) — the same decompression
algorithms, but reading source through a guest-supplied callback structure in
R3 rather than a raw pointer in R0.

The algorithmic core of `0x12` and `0x15` is identical to their `0x11`/`0x14`
siblings; the new work is:

1. A **synchronous re-entrant interpreter trampoline** — an HLE primitive that
   invokes a guest function pointer with controlled register setup, runs the
   CPU until the function returns to a sentinel address, and resumes the HLE
   handler with the returned value in R0. The three callback decompressors
   each call this trampoline once per source byte.
2. A **Huffman decompressor** (`0x13`) — new algorithm, built on top of the
   trampoline. Walks a prefix-coded binary tree in source memory using bits
   fetched from a 32-bit bitstream word.
3. **16-bit dest writes with buffering** — `0x12` and `0x15` write halfwords;
   each algorithm buffers two bytes and flushes to halfword-aligned dest.

After this slice lands, every BIOS decompressor Pokemon HG/SS uses during
boot — ReadNormal and ReadByCallback variants alike — is implemented with
real algorithms, not warn-stubs. The remaining ARM7 BIOS SWIs
(`0x1A` GetVolumeTable, `0x1B` GetPitchTable, `0x1C` SoundGetJumpList, legacy
GBA-only SWIs) are audio-related; slice 3h will scope them.

### Plain-language summary

A DS game ROM carries most of its data in compressed form, and the BIOS
exposes six SWIs for unpacking. Slice 3f handled the three "read the source
directly from RAM" variants. This slice handles the three "I'll give you a
function; call it every time you need the next source byte" variants.

The callback variants exist because the DS can decompress a stream that
isn't sitting in RAM all at once — for example, data streaming from the cart
through a different callback per source region. Games use them when the
source lives somewhere awkward (a region the BIOS couldn't enumerate
generically) or when they want the decompressor to write to VRAM via 16-bit
halfword writes (the "Vram" variants, versus the Wram variants we already
have which write 8 bits at a time).

The new engineering challenge is the **trampoline** — a way for our HLE BIOS
handler (written in C++) to call a function pointer that lives in guest ROM
code. Real BIOS does this naturally because real BIOS is *itself* guest code
running on the ARM7. We're not running real BIOS; our HLE handler is C++. So
when the algorithm says "now call `Get_8bit` to fetch the next source byte,"
we need to save the C++ handler's state, reconfigure the ARM7 core to execute
the guest-provided callback, run the interpreter until the callback returns,
grab the returned byte, and resume the C++ handler.

The technique is a **synchronous re-entrant step loop**:

1. Save the current ARM7 state we care about (registers the callback is
   allowed to clobber per the AAPCS calling convention — R0–R3, R12, LR).
2. Put the callback's arguments in R0–R3.
3. Set LR to a sentinel PC value that we'll watch for. When the callback
   executes its `BX LR` return, PC becomes the sentinel.
4. Set PC to the callback pointer (low bit selects ARM vs Thumb, per the
   standard ARM interworking convention).
5. Step the interpreter instruction-by-instruction in a loop; bail when
   PC hits the sentinel, or a budget expires (guards against a runaway
   callback).
6. Capture R0 as the return value.
7. Restore the saved state, return the R0 value to the HLE handler.

Because the step loop is the same one the emulator normally runs, IRQs fire
naturally, nested SWIs dispatch normally, and scheduler time advances at the
real cost of whatever instructions the callback executes. We just build the
trampoline on top of the interpreter and let it do its job.

After the trampoline works, `0x12` and `0x15` are tiny: they're the slice-3f
algorithms with two mechanical substitutions (`bus.read8(src)` →
`trampoline_call(Get_8bit, param)`, and `bus.write8(dest, byte)` → buffered
16-bit halfword writes). `0x13` Huffman is where the real algorithmic work
is — it's a tree walk plus a bitstream decoder, and it's the first decompressor
that actually needs a non-trivial amount of C++.

**What this slice builds:**
- `bios7_trampoline.{hpp,cpp}` — the re-entrant guest-call primitive.
- `bios7_decomp_lz77_vram.cpp` — `0x12` implementation.
- `bios7_decomp_rle_vram.cpp` — `0x15` implementation.
- `bios7_decomp_huffman.cpp` — `0x13` implementation.
- Updates to `bios7_decomp.cpp` / `.hpp` and `bios7_hle.cpp` to wire the three
  real entry points through the dispatcher (replacing the slice-3f warn-stubs).
- Per-algorithm unit tests with hand-crafted payloads + mock guest callbacks
  written as short ARM or Thumb subroutines assembled into main RAM.
- A capstone test exercising all three SWIs end-to-end through a real
  `Arm7Bus`, real dispatcher, real ARM7 core, and real guest-mode callbacks.

**What this slice deliberately does NOT build:**
- Cycle-accurate decompression cost. The trampoline charges real cycles for
  the callback's own instructions (naturally, via the step loop) plus 1 cycle
  for the SWI dispatcher frame. The decompression inner loop itself stays at
  the slice-3f 1-cycle charge. Still not cycle-accurate for the real BIOS's
  own algorithm cost; that's a Phase 6 polish item.
- SWI return-value reporting to the caller. Per GBATEK the SWI returns the
  decompressed length in R0, or a signed error code on callback failure. Our
  HLE returns 0 across the board (slice-3e/3f convention), which no HG/SS
  code path checks. If a future game does check it, we revisit.
- A generic "HLE calls guest function" utility beyond the trampoline needed
  here. If future SWIs want guest calls (e.g., `SoundDriverMain` if we ever
  HLE it), we generalize then, not now.
- Timing callbacks during HBlank / VBlank. Real BIOS callbacks fire during
  whatever point in the frame the CPU happens to be at; ours do the same
  because our trampoline doesn't manipulate the scheduler. Nothing special
  needed.

### Scope boundary

**In scope:** `bios7_trampoline.{hpp,cpp}` (the guest-call primitive);
the three SWI implementations `0x12`, `0x13`, `0x15`; the three
warn-stubs in `bios7_decomp.{hpp,cpp}` get replaced with real impls;
one new unit-test binary per SWI plus a trampoline unit test plus a
capstone; updates to the Huffman tree walker documented explicitly.

**Out of scope (§3 for detail):**
- Non-decompressor SWI guest calls.
- Cycle-accurate decompression.
- Return-value reporting.
- `Diff*UnFilter` (ARM7 doesn't have them; see slice 3f §5.5).
- Stress-testing the trampoline with pathological callback code (infinite
  loops, mode switches, wild SP mutations). We assert a step budget and log
  on timeout; full robustness is a hardening pass.

---

## 2. Goals

1. **Three callback decompressors return correct output for every test
   payload.** "Correct" means the destination buffer matches the known
   plaintext byte-for-byte, including the 16-bit dest-write width for `0x12`
   and `0x15` and the 32-bit dest-write width for `0x13`, and the disp=0
   caveat on `0x12` (actual_disp=1 intentionally produces wrong output,
   matching hardware).
2. **Trampoline is exercised in isolation** before any decompressor depends on
   it. A dedicated unit test runs a trampoline call with a minimal guest
   callback (e.g., `MOV R0, #0x42; BX LR`) and verifies R0 == 0x42 on return,
   state is restored, and the sentinel mechanism works for both ARM and
   Thumb callbacks.
3. **One SWI + its test per commit**. Trampoline scaffold → trampoline impl +
   test → `0x12` impl + test → `0x15` impl + test → `0x13` impl + test →
   capstone. Six commits.
4. **Bus discipline maintained.** All source reads go through the trampoline
   into guest callbacks (never direct reads from R0 — that's what makes these
   "callback" variants). All dest writes go through `bus_.write*`. 16-bit
   buffering is a local-state concern inside the algorithm, not a bus-level
   concern.
5. **No new I/O registers.** No new scheduler event kinds. No VRAM mapping
   changes. Zero cross-subsystem coupling additions.
6. **Trampoline preserves mode, SP, and banked-register state correctly.**
   Callbacks run in SVC mode (matching real BIOS behavior — the BIOS handler
   never mode-switches back to User before invoking callbacks). Nested SWIs
   during a callback work because our slice-3d exception-entry machinery is
   already re-entrant per-mode.
7. **Per-SWI tests link `ds_core` only, use `REQUIRE`, run in ms.** Same
   discipline as every prior slice.
8. **Huffman tree walk matches GBATEK byte-for-byte.** The node-offset
   formula `next = (current AND NOT 1) + Offset*2 + 2 [+1 for child 1]` and
   the MSB-first bitstream walk (`Bit31 = First Bit`) are verified against
   GBATEK with a comment block citing the section.

### 2.1 SWI coverage matrix

| # | Name | Family | This slice? | Notes |
|---|---|---|---|---|
| 0x10 | BitUnPack | decomp | landed (3f) | No change. |
| 0x11 | LZ77UnCompReadNormalWrite8bit | decomp | landed (3f) | No change. |
| 0x12 | LZ77UnCompReadByCallbackWrite16bit | decomp | **YES** | Callback source + 16-bit dest writes. |
| 0x13 | HuffUnCompReadByCallback | decomp | **YES** | New algorithm; tree walker + bitstream decoder. |
| 0x14 | RLUnCompReadNormalWrite8bit | decomp | landed (3f) | No change. |
| 0x15 | RLUnCompReadByCallbackWrite16bit | decomp | **YES** | Callback source + 16-bit dest writes. |

**3 real implementations replacing the slice-3f warn-stubs + 1 new
trampoline primitive + 5 new test binaries.** Total commits = 6 (see
Appendix A).

---

## 3. Non-goals

- **Generic HLE-calls-guest utility beyond the needs of this slice.** The
  trampoline's API is just what the three decompressors need. If slice 3h or
  a later slice wants a richer interface (multi-argument, multi-return,
  async), refactor then.
- **Non-BIOS callback conventions.** The trampoline follows the AAPCS calling
  convention that BIOS callbacks use (args in R0-R3, return in R0, R4-R11
  preserved, R12 scratch). Other callback conventions (GBA-style, custom)
  aren't a concern — BIOS decompressors are the only caller.
- **Trampoline robustness against malicious callbacks.** A callback that
  enters an infinite loop, fails to hit the sentinel, or clobbers SP will
  hit the step budget and be terminated with a warn log. Full fuzzing is out
  of scope.
- **Cycle-accurate decompression cost.** See §1.
- **Return value reporting.** See §1.
- **Source overrun on malformed payloads.** Same policy as slice 3f — our
  reads return 0 via open-bus or via a callback that eventually errors; we
  don't validate stream length against the header.
- **Dest overrun past the declared decompressed size.** Same policy as slice
  3f — the inner-loop `written < hdr.size` guard prevents it in all three
  algorithms.
- **OAM / palette destination special cases for 16-bit Vram variants.** Real
  BIOS doesn't promote 8-bit writes to halfword-duplicate when the callback
  variants target OAM/palette (the Vram variants already write halfwords, so
  the question doesn't arise). No special handling needed; the bus write
  path handles the destination region correctly.

---

## 4. Architecture

### 4.1 File layout

```
src/cpu/arm7/bios/
  bios7_trampoline.hpp          NEW. Declares the guest-call primitive:
                                  u32 arm7_bios_call_guest(
                                      Arm7& cpu,
                                      Arm7Bus& bus,
                                      u32 target_pc,     // low bit = T
                                      std::span<const u32, 4> args); // R0-R3
                                Plus the sentinel PC constant and the step
                                budget constant.
  bios7_trampoline.cpp          NEW. Implements the trampoline:
                                  - Saves R0-R3, R12, LR, PC, CPSR, SVC SP.
                                  - Writes args into R0-R3.
                                  - Sets LR to kTrampolineSentinelPC.
                                  - Sets PC to target & ~1; adjusts T bit of
                                    CPSR from target low bit.
                                  - Loops: while (PC != sentinel
                                                   && steps < budget):
                                       cpu.step_one_instruction();
                                  - Captures R0.
                                  - Restores saved registers, PC, CPSR, mode.
                                  - Returns R0.
  bios7_decomp.hpp              MODIFIED. Remove warn-stub declarations:
                                  bios7_lz77_callback_stub
                                  bios7_huff_callback_stub
                                  bios7_rl_callback_stub
                                Add real entry declarations:
                                  bios7_lz77_uncomp_vram
                                  bios7_huff_uncomp
                                  bios7_rl_uncomp_vram
                                Add helper declarations (callback fetch
                                wrapper, 16-bit buffered writer).
  bios7_decomp.cpp              MODIFIED. Remove the three warn-stubs.
                                Keep the shared header parser. Forward
                                declarations for the new entry points.
                                Current file size ~150 lines; replacements
                                stay under the 500-line soft cap by
                                splitting per-algorithm (see below).
  bios7_decomp_lz77_vram.cpp    NEW. ~100 lines. The `0x12` algorithm —
                                slice-3f LZ77 logic with callback source
                                and 16-bit buffered dest writes.
  bios7_decomp_rle_vram.cpp     NEW. ~100 lines. The `0x15` algorithm —
                                slice-3f RLE logic with callback source
                                and 16-bit buffered dest writes.
  bios7_decomp_huffman.cpp      NEW. ~180-220 lines. The `0x13` algorithm —
                                tree walker + bitstream decoder.
  bios7_hle.cpp                 MODIFIED. Dispatcher switch cases 0x12,
                                0x13, 0x15 route to the new real entry
                                points instead of the warn-stubs.

tests/unit/
  arm7_bios_trampoline_test.cpp            NEW. Trampoline isolation
                                           test — calls a guest ARM and
                                           Thumb stub assembled in main
                                           RAM. Verifies R0 return, state
                                           restore, step budget.
  arm7_bios_lz77_vram_test.cpp             NEW. `0x12` correctness with
                                           mock guest callbacks.
  arm7_bios_rle_vram_test.cpp              NEW. `0x15` correctness.
  arm7_bios_huffman_test.cpp               NEW. `0x13` correctness.
  arm7_bios_decomp_callback_sequence_test.cpp  NEW. Capstone.
  arm7_bios_decomp_dispatch_test.cpp       MODIFIED. Replace the three
                                           callback-stub assertions with
                                           smoke tests for the real
                                           dispatch path.
```

**Five new test binaries + one modified** — CTest count goes 55 → 60.

### 4.2 Trampoline API

```cpp
// src/cpu/arm7/bios/bios7_trampoline.hpp
namespace ds {

// Sentinel PC value — the trampoline writes this to LR before invoking the
// callback. When the callback's BX LR lands here, the step loop exits.
// Chosen to be in an unmapped region of the ARM7 address space. Any fetch
// from this address would fault; we detect the PC value BEFORE the fetch,
// so the address is never dereferenced.
inline constexpr u32 kTrampolineSentinelPC = 0xFFFF'FFF0u;

// Maximum guest instructions per trampoline call. Real callbacks are 3-10
// instructions. A runaway callback hits this budget, warns, and returns 0.
inline constexpr u32 kTrampolineStepBudget = 1024u;

// Invokes a guest function pointer synchronously. Args are passed in R0-R3.
// Returns the value the callback left in R0 (or 0 on budget exhaustion).
// The target_pc low bit selects ARM (0) vs Thumb (1) via the standard
// interworking convention.
u32 arm7_bios_call_guest(Arm7& cpu, Arm7Bus& bus, u32 target_pc,
                         std::span<const u32, 4> args);

} // namespace ds
```

### 4.3 Trampoline save/restore scope

**Save before call:** R0-R3, R12 (IP — scratch per AAPCS), R14 (LR), PC,
CPSR, and SVC-mode SP (R13_svc — the callback might alter it, we need the
exact value back). We do NOT save R4-R11; those are callee-saved per AAPCS,
meaning the callback is required to preserve them. If a callback violates
this, decompression output will be garbage — we log a warn if we detect
R4-R11 mutation on return, but don't attempt to restore.

**Restore after call:** R0 is the return value, held separately. Everything
else (R1-R3, R12, R14, PC, CPSR, R13_svc) gets restored from the saved
snapshot. The caller's R0 is NOT restored — the caller wrote its own R0 on
entry, and the trampoline's caller (the SWI handler) only reads the return
value, never touches its own R0 around the call.

**Mode:** callbacks run in SVC mode (matching real BIOS, which invokes
callbacks without a mode switch). CPSR T bit comes from `target_pc & 1`.
IRQs stay enabled (`I = 0`) unless the caller explicitly masks them, which
the decompressor SWIs don't. A nested IRQ during the callback fires via the
normal exception-entry path (slice 3d), which preserves SVC state correctly
via per-mode banking.

**Re-entrancy:** the trampoline is re-entrant because everything it saves
lives on the C++ stack (`std::array<u32, N> saved`). A nested SWI during
the callback pushes a new C++ stack frame; the inner trampoline's save/
restore is independent of the outer's. This is exactly the re-entrancy
model ARM hardware uses with banked R13/R14/SPSR per mode.

### 4.4 Step loop

```cpp
// Pseudocode — slice 3g trampoline core.
while (cpu.state().pc != kTrampolineSentinelPC) {
    cpu.step_one_instruction();
    if (++steps > kTrampolineStepBudget) {
        DS_LOG_WARN("arm7/bios: trampoline step budget exceeded at PC=0x%08X",
                    cpu.state().pc);
        break;
    }
}
```

`cpu.step_one_instruction()` is whatever single-instruction step primitive
the ARM7 core exposes — if one doesn't exist, the slice adds it as a thin
wrapper over `arm7_execute_one` (already internal). We prefer to call the
public CPU interface so IRQ checks and instruction-boundary bookkeeping
happen naturally.

**Scheduler advancement:** each instruction step advances the ARM7
cycle counter normally. The trampoline never manipulates the scheduler
directly. If VBlank fires during a long callback, the interrupt is
serviced inside the step loop via the normal IRQ-entry path. Decompression
throughput is therefore real-cycle correct for the callback's own code,
even if the SWI dispatcher frame itself only charges 1 cycle overhead.

**Sentinel detection:** checked before each step so the final `BX LR`
landing at the sentinel doesn't get processed. The sentinel address is
`0xFFFFFFF0`, outside any mapped ARM7 region (ARM7 main RAM stops at
0x023FFFFF, ROM mirror / cart bus stops below 0x10000000, I/O and VRAM
don't map that high). A fetch from the sentinel would hit open-bus and
return garbage; we don't fetch because we detect the PC value first.

### 4.5 Callback structure access

```cpp
// src/cpu/arm7/bios/bios7_decomp.hpp (addition)
namespace ds {

// Layout of the callback structure at R3 per GBATEK.
struct DecompCallbacks {
    u32 open_and_get_32bit;  // +0x00
    u32 close;               // +0x04 (0 = no-op)
    u32 get_8bit;             // +0x08
    u32 get_16bit;            // +0x0C (documented as unused by all three)
    u32 get_32bit;            // +0x10 (Huffman only)
};

DecompCallbacks read_decomp_callbacks(Arm7Bus& bus, u32 r3);

} // namespace ds
```

The SWI fetches R3 at entry, reads the five function pointers via
`bus.read32`, and uses them with the trampoline for the lifetime of one
decompression. Pointers are not re-fetched per call (real BIOS doesn't
either).

### 4.6 `0x12` LZ77 Vram algorithm shape

```cpp
// Pseudocode — slice 3g LZ77 Vram.
u32 bios7_lz77_uncomp_vram(Arm7State& s, Arm7Bus& bus, Arm7& cpu) {
    // GBATEK: entry R0 is the "source address" — an opaque handle passed
    // as R0 to every callback (Open, Get_8bit, Close). Entry R2 is the
    // user-defined callback parameter, passed to Open only.
    const u32 callback_param = s.r[0];
    const DecompCallbacks cb = read_decomp_callbacks(bus, s.r[3]);

    // Open_and_get_32bit receives (R0=entry_R0, R1=dest, R2=user_param)
    // and returns the header word in R0.
    const u32 hdr_word = trampoline_call(cpu, bus, cb.open_and_get_32bit,
                                          {callback_param, s.r[1], s.r[2], 0});
    const DecompHeader hdr = split_header(hdr_word);

    u32 dest = s.r[1];
    u32 written = 0;
    u16 halfword_buffer = 0;
    u32 halfword_bits = 0;

    auto emit_byte = [&](u8 byte) {
        halfword_buffer |= u16{byte} << halfword_bits;
        halfword_bits += 8;
        if (halfword_bits == 16) {
            bus.write16(dest, halfword_buffer);
            dest += 2;
            halfword_buffer = 0;
            halfword_bits = 0;
        }
    };

    while (written < hdr.size) {
        const u8 flags = u8(trampoline_call(cpu, bus, cb.get_8bit,
                                             {callback_param, 0, 0, 0}));
        for (int bit = 7; bit >= 0 && written < hdr.size; --bit) {
            const bool compressed = (flags >> bit) & 1u;
            if (!compressed) {
                const u8 byte = u8(trampoline_call(cpu, bus, cb.get_8bit,
                                                    {callback_param, 0, 0, 0}));
                emit_byte(byte);
                ++written;
            } else {
                const u8 b0 = u8(trampoline_call(...));
                const u8 b1 = u8(trampoline_call(...));
                const u32 len  = ((b0 >> 4) & 0x0Fu) + 3u;
                const u32 disp = (((b0 & 0x0Fu) << 8) | b1) + 1u;
                // disp=1 (actual) is broken on Vram: the byte we want to
                // read hasn't been flushed to dest yet. Per GBATEK we do
                // NOT compensate; output is intentionally garbage for
                // disp=1. Tested explicitly.
                for (u32 i = 0; i < len && written < hdr.size; ++i) {
                    const u8 byte = bus.read8(dest + (halfword_bits / 8) - disp);
                    emit_byte(byte);
                    ++written;
                }
            }
        }
    }

    // Flush final partial halfword (written count is always even in
    // well-formed payloads; this handles malformed ones deterministically).
    if (halfword_bits != 0) {
        bus.write16(dest, halfword_buffer);
    }

    if (cb.close != 0) {
        trampoline_call(cpu, bus, cb.close, {callback_param, 0, 0, 0});
    }
    return 1;
}
```

**disp=1 caveat explicitly preserved.** Per GBATEK, the Vram variant reads
from `[dest-disp-1]` where dest has already advanced to a halfword boundary
but the latest bytes haven't flushed yet. We model this by computing the
effective read address as `dest + (halfword_bits / 8) - disp` — when
halfword_bits is 8 (one byte pending), the effective "current dest" is
`dest + 1 - disp`. With disp=1, that's `dest`, which hasn't been written yet
(still in the halfword buffer). The read returns stale memory. Exactly
matches hardware.

### 4.7 `0x15` RLE Vram algorithm shape

Structurally identical to LZ77 Vram: reuse the slice-3f RLE state machine,
swap source reads for callback calls, swap dest writes for a 16-bit
buffered emit. No disp caveat (RLE has no back-references).

### 4.8 `0x13` Huffman algorithm shape

GBATEK-derived facts:

- **Header** (fetched via `Open_and_get_32bit`):
  - Bit 0-3: data size in bits (4 or 8; determines output chunk size).
  - Bit 4-7: type = 2 (Huffman).
  - Bit 8-31: decompressed size in bytes.
- **Tree size** (one byte, fetched via `Get_8bit`): tree table size /
  2 - 1. The tree occupies the next `(tree_size + 1) * 2` bytes.
- **Tree table**: variable-length prefix-coded binary tree.
  - Root and non-data nodes have: bit 0-5 = offset to next child, bit 6 =
    node1 end flag, bit 7 = node0 end flag.
  - Next child node0 address: `(current_addr AND NOT 1) + offset*2 + 2`.
  - Next child node1 address: same + 1.
  - Data nodes: bit 0-7 = the raw data value (upper bits zero if data size
    is less than 8).
- **Bitstream**: stored in 32-bit little-endian units (fetched via
  `Get_32bit`). Within each word, bit 31 is consumed first (MSB-first).
  Bit value 0 = go to node0, 1 = node1. Walk from root to a data node;
  emit the data; repeat.
- **Output**: assembled into 32-bit words (chunks of `data_size` bits each,
  LSB-first packing into the word), written to dest via `bus.write32`.

Implementation strategy:

1. Open → get header word → split type/size/data_size_bits.
2. Get_8bit → tree_size. Compute tree_table_size_bytes = `(tree_size + 1) * 2`.
3. Copy the tree table to the temp buffer (R2) byte-by-byte via
   `Get_8bit`, so we can walk it with cheap array indexing instead of a
   callback per node. (Real BIOS does this copy for the same reason.)
4. While written < decompressed_size:
   a. If no active bitstream word, `Get_32bit` → `bit_buffer`, reset
      bit counter to 32.
   b. Walk the tree from root. At each node, extract bit 31 of the bit
      buffer, shift left by 1. If bit = 0 go to node0, else node1. Check
      end flag; if set, we landed on a data node — pull its data value.
   c. Append data value to an output accumulator (LSB-first packing).
   d. When 32 bits are accumulated, flush via `bus.write32`.
5. Close callback if non-null.

**Trampoline calls per byte:** Huffman is the heaviest consumer of
trampoline calls — 1 call per `Get_32bit` (every 32 bits consumed) plus
the initial tree setup. Budget impact: each `Get_32bit` is ~3-5 guest
instructions; the step budget of 1024 is ample.

### 4.9 Dispatcher wire-up

`bios7_hle.cpp` switch cases `0x12`, `0x13`, `0x15` call the new real entry
points with the same `(state, bus)` signature as every other SWI, plus
implicit access to the ARM7 core via `NDS&` if needed (the trampoline wants
the core, not just state + bus).

**API question:** the real entry points need `Arm7&` not just `Arm7State& +
Arm7Bus&`, because the trampoline steps the interpreter. The dispatcher
signature `arm7_bios_hle_dispatch_swi(Arm7State&, Arm7Bus&, u32)` doesn't
carry the core. Options:

1. **Change the dispatcher signature** to take `Arm7&`. Touches every SWI
   handler site; most don't need it. Clean but invasive.
2. **Pass the core through a wrapper** — introduce `arm7_bios_hle_dispatch_swi_v2(Arm7&, u32)` that internally uses `cpu.state()` and `cpu.bus()`. Callers switch over time.
3. **Thread the core via `NDS&`** — each HLE handler can reach the core via a back-pointer. Violates the "no pointers between subsystems" rule (#3).
4. **Stash a raw `Arm7*` in thread-local storage** around the dispatch call
   and look it up from the trampoline. Ugly.

**Decision:** option 1 is the only one that's clean. Slice 3g commit 1
changes the dispatcher signature to `(Arm7&, u32)` and threads
`cpu.state()` / `cpu.bus()` to every existing SWI handler via brief
wrappers. This is mechanical — each handler becomes:

```cpp
case 0x09: cycles = bios7_div(cpu.state(), cpu.bus()); break;
```

No handler signature changes; only the dispatcher entry point changes.
Callers of the dispatcher (the SWI exception entry in `arm7_decode.cpp` and
the test support) update to pass `cpu` instead of `cpu.state()`.

### 4.10 Known technical debt deferred

- **Full cycle accuracy.** See §1.
- **Return-value reporting.** See §1.
- **R4-R11 mutation detection in the trampoline.** We log a warn if we
  detect it, but don't attempt to restore the caller's values. Hardening
  deferred to a future slice.
- **Trampoline for non-decompressor SWIs.** If slice 3h wants to HLE a sound
  SWI that takes a callback, we reuse the primitive; no generalization
  pass planned.

---

## 5. Hardware details

### 5.1 Callback structure (R3)

GBATEK: "Callback structure (five 32bit pointers to callback functions):
`Open_and_get_32bit` (eg. LDR r0,[r0], get header), `Close` (optional,
0=none), `Get_8bit` (eg. LDRB r0,[r0]), `Get_16bit` (not used), `Get_32bit`
(used by Huffman only)."

**Offsets:** +0x00, +0x04, +0x08, +0x0C, +0x10. All 32-bit, little-endian.
**Thumb dispatch:** bit 0 of each pointer selects Thumb (1) or ARM (0).

**R0 semantics (all three SWIs):** GBATEK calls this the "source address,"
but for the callback variants it is simply an opaque handle — typically a
pointer to game-maintained callback state. The BIOS passes entry-R0 as
the R0 argument to *every* callback invocation (Open, Get_8bit,
Get_32bit, Close). Our HLE does the same.

**R1 semantics (all three SWIs):** initial destination address. Passed
to Open as its R1 argument. Not passed to Get_8bit / Get_32bit / Close.

**R2 semantics:**
- LZ77 / RLE callbacks (0x12, 0x15): "user-defined callback parameter
  passed on to Open function" — passed only to Open, as its R2
  argument. Not passed to Get_8bit or Close.
- Huffman (0x13): R2 is a pointer to a temp buffer (≤ 0x200 bytes) used
  internally by the BIOS to copy the Huffman tree for cheap walking.
  It is passed to Open as its R2 argument (since Open receives the
  register file as-is), but most callbacks ignore it. No Get_* callback
  receives R2.

**Call ABI summary:**
- `Open(R0=entry_R0, R1=entry_R1=dest, R2=entry_R2)` → returns header in R0.
- `Get_8bit(R0=entry_R0)` → returns byte in R0.
- `Get_32bit(R0=entry_R0)` → returns word in R0 (Huffman only).
- `Close(R0=entry_R0)` → return value ignored.

### 5.2 Return values

Per GBATEK: "The SWI returns the length of decompressed data, or the signed
errorcode from the Open/Close functions." Our HLE returns 0 from every SWI
(slice-3e/3f convention). Callback-returned errors are NOT propagated; we
log a warn if Open returns a value > 0xFFFFFF (likely an error code) and
continue with whatever header word it gave.

### 5.3 LZ77 Vram disp=0 caveat

Per GBATEK: "Writing 16bit units to [dest-1] instead of 8bit units to
[dest] means that reading from [dest-1] won't work, ie. the 'Vram'
function works only with disp=001h..FFFh, but not with disp=000h."

Our impl matches: the read-back uses the current effective dest address
(accounting for the unflushed halfword buffer), which for actual_disp=1
reads stale memory. Tested explicitly — `0x12` with disp=0 encoding
produces different output than `0x11` with the same encoding.

### 5.4 Huffman tree layout

Per GBATEK (copied verbatim for accuracy):

> Data Header (32bit)
>   Bit0-3   Data size in bit units (normally 4 or 8)
>   Bit4-7   Compressed type (must be 2 for Huffman)
>   Bit8-31  24bit size of decompressed data in bytes

> Tree Size (8bit)
>   Bit0-7   Size of Tree Table/2-1 (ie. Offset to Compressed Bitstream)

> Root Node and Non-Data-Child Nodes are:
>   Bit0-5   Offset to next child node,
>            Next child node0 is at (CurrentAddr AND NOT 1)+Offset*2+2
>            Next child node1 is at (CurrentAddr AND NOT 1)+Offset*2+2+1
>   Bit6     Node1 End Flag (1=Next child node is data)
>   Bit7     Node0 End Flag (1=Next child node is data)

> Data nodes are (when End Flag was set in parent node):
>   Bit0-7   Data (upper bits should be zero if Data Size is less than 8)

> Compressed Bitstream (stored in units of 32bits)
>   Bit0-31  Node Bits (Bit31=First Bit)  (0=Node0, 1=Node1)

Our implementation cites this section in a comment block and implements
each rule exactly. The `AND NOT 1` on current address means the tree walker
ignores the low bit of the current node's offset — a subtle hardware detail.

### 5.5 Huffman dest writes

Per GBATEK: "Data is written in units of 32bits". Dest must be 32-bit
aligned. We accumulate output bits into a 32-bit `out_buffer` (LSB-first
packing of `data_size`-bit chunks) and flush via `bus.write32` when 32 bits
are accumulated. Final partial word flushes to match malformed-input
determinism (same as BitUnPack).

### 5.6 Mode during callbacks

Real BIOS invokes callbacks from SVC mode without mode switching. Our
trampoline matches: CPSR stays SVC during the callback. If the callback
mode-switches via `MSR CPSR_c` or similar, our CPU core handles the switch
normally, and the trampoline's restore (which overwrites CPSR at end)
brings us back to SVC regardless.

---

## 6. Testing strategy

Five new test binaries (55 → 60), one modified.

### 6.1 `arm7_bios_trampoline_test.cpp`

Coverage:
- **ARM callback, returns constant.** Assemble `E3A00042` (MOV R0, #0x42)
  + `E12FFF1E` (BX LR) in main RAM. Call trampoline, assert R0 == 0x42.
- **Thumb callback, returns constant.** Assemble `20 42` (MOV R0, #0x42)
  + `47 70` (BX LR) in main RAM at an odd address. Call trampoline with
  low-bit-set pointer. Assert R0 == 0x42.
- **Callback uses args R0-R3.** Callback computes R0 + R1 + R2 + R3 and
  returns it. Trampoline called with known args; assert sum.
- **Callback reads from bus and returns.** Callback does `LDRB R0, [R0]`
  then `BX LR`. Memory pre-seeded at callback's R0 arg. Assert returned
  byte matches seeded value.
- **Step budget exceeded.** Callback is `B .` (infinite loop). Trampoline
  returns 0, warn logged, CPU state restored without corruption.
- **State preservation.** Before call: seed R4-R11 with known values, R13
  SVC with known SP, CPSR with known mode. Use a callback that does NOT
  touch R4-R11. After call: assert R4-R11, R13_svc, CPSR all unchanged.
- **Caller R1-R3, R12, LR are restored after call.** Even if the callback
  mutates them.

### 6.2 `arm7_bios_lz77_vram_test.cpp`

- **Mock callback callback-structure:** build in main RAM a struct with
  five pointers, each pointing to a short ARM stub that reads a byte from
  a stream array (via a static incrementing index held in R0's callback
  param) and returns it.
- **Cases:**
  - Single uncompressed block (1 raw byte).
  - Single compressed block (raw byte + len=3 disp=2 back-ref).
  - Length-8 disp=2 back-ref — exercises the post-halfword-boundary case
    where the effective read address is halfway into the current pending
    halfword.
  - **disp=0 (actual_disp=1) — GARBAGE output expected, matching hardware.**
    The test asserts the output is NOT equal to the all-ones self-overlap
    plaintext that the Wram variant would produce.
  - Zero decompressed size.

### 6.3 `arm7_bios_rle_vram_test.cpp`

- **Cases:**
  - Uncompressed length 1 (flag 0x00).
  - Compressed length 3 (flag 0x80).
  - Compressed length 130 (flag 0xFF).
  - Mixed stream with run/raw/run.
  - Zero size.

### 6.4 `arm7_bios_huffman_test.cpp`

- **Cases:**
  - **Data size = 8, trivial tree (two data nodes).** Simplest possible
    tree: one non-data root + two data children. Output alternates between
    two values based on input bits.
  - **Data size = 8, balanced 8-leaf tree.** All 256 / 8 = 32 possible
    codes of length 3 bits; decompress a known payload.
  - **Data size = 4, nibble output.** Verifies the `data_size_bits != 8`
    path and LSB-first nibble packing into 32-bit output words.
  - **Bitstream that spans exactly one 32-bit word.** Tests the per-word
    refetch boundary.
  - **Zero decompressed size.**

### 6.5 `arm7_bios_decomp_callback_sequence_test.cpp` — capstone

Three sub-cases, each running end-to-end through the real dispatcher and
trampoline path:

1. **LZ77 Vram `0x12`.** Real callback in guest code (ARM stub in main
   RAM), 64-byte plaintext payload decompressed to a target buffer,
   halfword-aligned.
2. **RLE Vram `0x15`.** Real callback, 32-byte mixed-stream plaintext.
3. **Huffman `0x13`.** Real callbacks (`Open_and_get_32bit` + `Get_8bit` +
   `Get_32bit`), 48-byte plaintext with a 4-leaf balanced tree.

Each sub-case invokes `arm7_bios_hle_dispatch_swi(cpu, swi_num)` with the
post-exception-entry SVC state setup, and asserts the output matches the
known plaintext byte-for-byte.

### 6.6 Test binary count

End of slice 3f: **55** binaries.
Adds:
- `arm7_bios_trampoline_test`
- `arm7_bios_lz77_vram_test`
- `arm7_bios_rle_vram_test`
- `arm7_bios_huffman_test`
- `arm7_bios_decomp_callback_sequence_test`

End of slice 3g: **60** binaries.

---

## 7. Cross-references

- **Slice 3d** — exception entry / IRQ path; the trampoline relies on this
  being re-entrant.
- **Slice 3e** — SWI dispatch table + implicit MOVS PC, R14; the
  trampoline's save/restore mirrors this for guest-call contexts.
- **Slice 3f** — ReadNormal decompressors; `0x12` and `0x15` reuse the
  algorithms with callback + 16-bit write substitutions.
- **Bus** — `arm7_bus.{hpp,cpp}`; `bus.read8/16/32` and `bus.write8/16/32`
  used throughout.
- **ARM7 core** — `arm7.{hpp,cpp}`; the trampoline calls `cpu.step_one_instruction()` (to be added as a public entry point if not
  already exposed).
- **GBATEK BIOS Decompression Functions** — https://problemkaputt.de/gbatek-bios-decompression-functions.htm — primary spec.
- **DeSmuME `src/bios.cpp`** — cross-reference for the Huffman and
  BitUnPack implementations; melonDS also has the Huffman variant.

---

## 8. Risk and rollback

### Risk 1 — Step primitive not exposed on `Arm7`

The trampoline calls `cpu.step_one_instruction()`. The ARM7 core may only
expose `run_until(cycles)`. **Mitigation:** commit 1 adds a public
`step_one_instruction()` that the existing `run_until` already loops over;
no behavioral change.

### Risk 2 — Step budget too small or too large

1024 is a guess. Too small → legit callbacks hit the budget and fail. Too
large → runaway callbacks burn CPU. **Mitigation:** the test suite includes
the longest realistic callback (Huffman's `Get_32bit` is ~5 instructions;
a far-jump sequence with register save/restore might be 15-20); 1024 is
50x headroom. If an HG/SS callback hits it in practice, bump to 4096.

### Risk 3 — Sentinel address collision with game memory

Chosen address 0xFFFFFFF0 is in the unmapped high address space on ARM7.
**Mitigation:** we detect the sentinel via PC comparison *before* any bus
access. No actual fetch happens at the sentinel address. If a ROM someday
maps memory there (extremely unlikely on DS), we pick a different address.

### Risk 4 — Callback SP_svc corruption

A buggy callback that mutates R13_svc without restoring leaves the BIOS's
SVC stack in an inconsistent state. Real BIOS would crash; ours does too
unless we save/restore R13_svc. **Mitigation:** the trampoline saves and
restores R13_svc (§4.3).

### Risk 5 — Nested IRQ during callback re-entering a decompression SWI

The outer decompression SWI is on the C++ stack; an IRQ during the callback
dispatches normally. If the IRQ handler itself issues a decompression SWI
(bizarre but possible), we get nested trampoline calls. The C++ stack
supports this; each trampoline save/restore is independent.
**Mitigation:** the test suite's trampoline test includes a nested-SWI case
to verify.

### Risk 6 — Huffman tree walker off-by-one on the AND-NOT-1 address math

GBATEK is explicit: `(CurrentAddr AND NOT 1) + Offset*2 + 2`. Easy to drop
the NOT-1 or change +2 to +1. **Mitigation:** the `arm7_bios_huffman_test`
cases use a tree where the NOT-1 masking matters (tree node at an odd
address — possible because tree bytes are packed back-to-back). If the
masking is wrong, the test fails immediately with wrong output.

### Risk 7 — Dispatcher signature change touching too many sites

Changing `arm7_bios_hle_dispatch_swi` from `(Arm7State&, Arm7Bus&, u32)` to
`(Arm7&, u32)` modifies every caller. **Mitigation:** the change is
mechanical (each SWI handler becomes `handler(cpu.state(), cpu.bus())`);
no handler logic changes. Commit 1 does the signature change as a single
atomic step with tests re-running green.

---

## 9. Slice completion criteria

- [ ] `arm7_bios_call_guest` exists in `bios7_trampoline.cpp` and has a
      public declaration in `bios7_trampoline.hpp`.
- [ ] `bios7_lz77_uncomp_vram`, `bios7_rl_uncomp_vram`,
      `bios7_huff_uncomp` exist in their respective `.cpp` files and are
      reachable via `arm7_bios_hle_dispatch_swi(cpu, 0x12/0x13/0x15)`.
- [ ] The three warn-stubs (`bios7_lz77_callback_stub`,
      `bios7_huff_callback_stub`, `bios7_rl_callback_stub`) are deleted.
- [ ] `arm7_bios_decomp_dispatch_test.cpp` updated to smoke-test the real
      dispatch path for `0x12/0x13/0x15`.
- [ ] Five new test binaries land:
      `arm7_bios_trampoline_test`,
      `arm7_bios_lz77_vram_test`,
      `arm7_bios_rle_vram_test`,
      `arm7_bios_huffman_test`,
      `arm7_bios_decomp_callback_sequence_test`.
- [ ] `ctest --output-on-failure` reports 60/60 passing in Debug.
- [ ] `clang-format` clean on all new files.
- [ ] `ds-architecture-rule-checker` and `gbatek-reviewer` both return clean.
- [ ] `quality-reviewer` returns clean.
- [ ] No file exceeds the 500-line soft cap.

---

## Appendix A. Commit sequence

Six commits, shippable individually:

### Commit 1 — Trampoline scaffold + dispatcher signature change

- Expose `Arm7::step_one_instruction()` as a public method if not already.
- Add `src/cpu/arm7/bios/bios7_trampoline.{hpp,cpp}` with
  `arm7_bios_call_guest`. Implementation is complete (not a stub) because
  commit 2 can only test a non-stub implementation.
- Wait — this is "commit 2 depends on commit 1 being fully functional."
  Revised: commit 1 is scaffold ONLY (returns 0, warn). Commit 2 is the
  real implementation + trampoline_test.
- Change `arm7_bios_hle_dispatch_swi` signature from `(Arm7State&,
  Arm7Bus&, u32)` to `(Arm7&, u32)`. Update every caller (SWI dispatch
  in `arm7_decode.cpp`, test support). Internal handler calls become
  `handler(cpu.state(), cpu.bus())`.
- `ctest`: 55/55 still green (no new tests yet; the signature change is
  mechanical and doesn't alter behavior).

### Commit 2 — Trampoline implementation + test

- Replace the trampoline scaffold with the real implementation (save/
  restore, sentinel, step loop, budget).
- Add `tests/unit/arm7_bios_trampoline_test.cpp` with the §6.1 cases.
- Add CMake entry.
- `ctest`: 56/56.

### Commit 3 — LZ77 Vram `0x12` + test

- Add `src/cpu/arm7/bios/bios7_decomp_lz77_vram.cpp` with the §4.6
  algorithm.
- Update `bios7_decomp.hpp` / `bios7_hle.cpp` — remove `bios7_lz77_callback_stub`, add and wire `bios7_lz77_uncomp_vram`.
- Add `tests/unit/arm7_bios_lz77_vram_test.cpp` with the §6.2 cases
  (including the disp=0 hardware-accurate garbage case).
- Update `arm7_bios_decomp_dispatch_test.cpp` to smoke-test `0x12`.
- `ctest`: 57/57.

### Commit 4 — RLE Vram `0x15` + test

- Add `src/cpu/arm7/bios/bios7_decomp_rle_vram.cpp` with the §4.7 algorithm.
- Replace `bios7_rl_callback_stub` with `bios7_rl_uncomp_vram`.
- Add `tests/unit/arm7_bios_rle_vram_test.cpp` with the §6.3 cases.
- Update dispatch test for `0x15`.
- `ctest`: 58/58.

### Commit 5 — Huffman `0x13` + test

- Add `src/cpu/arm7/bios/bios7_decomp_huffman.cpp` with the §4.8 algorithm.
- Replace `bios7_huff_callback_stub` with `bios7_huff_uncomp`.
- Add `tests/unit/arm7_bios_huffman_test.cpp` with the §6.4 cases.
- Update dispatch test for `0x13`.
- `ctest`: 59/59.

### Commit 6 — Capstone end-to-end

- Add `tests/unit/arm7_bios_decomp_callback_sequence_test.cpp` with the
  §6.5 three sub-cases running through the full dispatcher + trampoline +
  interpreter path.
- `ctest`: 60/60.

---

## Appendix B. Trampoline save/restore reference

| Register | Save? | Restore? | Reason |
|---|---|---|---|
| R0 | yes | no | Callback's R0 is the return value; caller's R0 is overwritten by args. |
| R1-R3 | yes | yes | AAPCS scratch, caller-saved from callee perspective. |
| R4-R11 | no | no | AAPCS callee-saved. Callback preserves. Warn if detected otherwise. |
| R12 | yes | yes | AAPCS scratch. |
| R13 (SP_svc) | yes | yes | Callback may mutate; must restore. |
| R14 (LR) | yes | yes | Overwritten with sentinel PC for sentinel detection. |
| PC | yes | yes | Overwritten to `target & ~1`. |
| CPSR | yes | yes | T bit from `target & 1`; mode stays SVC. |
| SPSR_svc | no | no | Not touched by the trampoline. |

---

## Appendix C. Callback-pointer calling convention

Per AAPCS (ARM Architecture Procedure Call Standard), inherited by DS BIOS
callbacks:

- Args R0-R3; further args on stack (not used here).
- Return in R0.
- R4-R11 callee-saved.
- R12 scratch.
- R13 stack pointer (preserve).
- R14 link register (BL sets; BX reads).
- ARM/Thumb mode selected by low bit of target pointer via BX/BLX semantics.

Callbacks are BX-called (not BL-called), so the caller must set LR
explicitly before transferring control. The trampoline does this by setting
LR to the sentinel PC before jumping to the callback's target.
