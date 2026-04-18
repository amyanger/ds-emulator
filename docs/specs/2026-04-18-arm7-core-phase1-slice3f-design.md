# ARM7 Core — Phase 1, Slice 3f Design

**Date:** 2026-04-18
**Slice:** ARM7 BIOS HLE — decompressor family, ReadNormal variants
**Status:** proposed
**Prior slice:** 3e (BIOS HLE SWI table) — landed through `a95121a`; 50 CTest binaries green
**Next slice:** 3g (decompressor ReadByCallback variants — LZ77 Vram 0x12, Huffman 0x13, RLE Vram 0x15) — requires a callback-dispatch trampoline; intentionally split from this slice

---

## 1. Summary

Slice 3e implemented every SWI Pokemon HG/SS uses during boot **except** the
six BIOS decompressors. Those six were left as a single shared `warn_stub`
because they fall into a distinct concern with two very different
implementation profiles:

- Three **ReadNormal** variants (`BitUnPack` 0x10, `LZ77UnComp` 0x11,
  `RLUnComp` 0x14) read source data directly from a memory pointer in `R0`.
  They are pure C++ algorithms over the bus; no new infrastructure needed.
- Three **ReadByCallback** variants (`LZ77UnComp` 0x12, `HuffUnComp` 0x13,
  `RLUnComp` 0x15) read source through a guest-supplied callback structure
  (`R3`) — the BIOS calls back into ROM code mid-decompression to fetch the
  next byte. This requires a re-entrant interpreter trampoline that is its
  own non-trivial piece of work.

This slice ships the three ReadNormal variants and the file scaffolding the
ReadByCallback variants will reuse. Slice 3g picks up the callback trampoline
plus the three callback variants together. Splitting along this seam keeps
each slice tightly focused and avoids a 20-commit mega-slice.

After this slice lands, every game asset Pokemon HG/SS decompresses to main
RAM (the overwhelming majority — sprite sheets, map data, scripts, dialogue
tables) decompresses correctly. The remaining VRAM-targeted decompressions
(some font / tile uploads via the callback variants) still warn-stub, but the
boot path no longer halts at a missing decompressor.

### Plain-language summary

A DS game ROM is mostly compressed data. Sprite sheets, map tiles, dialogue
strings, even some script bytecode — all of it is squeezed onto the cart and
unpacked at runtime by a handful of standardized BIOS routines. The game
calls `SWI 0x11` and the BIOS expands LZ77-compressed bytes into RAM. Same
idea for `SWI 0x14` (run-length) and `SWI 0x10` (bit-depth expansion, used
mostly for fonts). These three are the workhorses; HG/SS boot calls them
hundreds of times before the title screen renders.

The algorithms themselves are well-documented and small (~50–100 lines each
in C++). The interesting design questions are:

1. **How do reads/writes go through the bus?** Source can live in main RAM,
   shared WRAM, or cart ROM mirror; destination can be main RAM, WRAM, or
   TCM. We must use `bus_.read*/write*` rather than reaching into raw
   buffers, otherwise mirrors break and TCM is invisible.
2. **What's the right unit of bus access?** GBATEK uses the labels "Wram"
   (writes 8-bit at a time) and "Vram" (writes 16-bit at a time) for these
   variants. The Wram variants we ship in this slice all use 8-bit writes.
3. **How do we test it?** A test ROM-style approach — embed a small known
   compressed payload as a `constexpr` array in the test binary, execute the
   SWI against an in-memory bus, and compare the decompressed output against
   the known plaintext.

The three deferred ReadByCallback variants share a separate problem: the
decompressor has to *call back into guest code* mid-execution. That requires
running the interpreter recursively from inside an HLE handler, which is a
distinct piece of infrastructure. Slice 3g handles it as a single bundle so
the trampoline lands exactly once and gets reused by all three callback
variants.

**What this slice builds:**
- The three ReadNormal SWI implementations:
  - `0x10 BitUnPack` — bit-depth expansion with optional zero-bias offset.
  - `0x11 LZ77UnCompReadNormalWrite8bit` — LZ77 with 8-bit dest writes.
  - `0x14 RLUnCompReadNormalWrite8bit` — run-length with 8-bit dest writes.
- A new family file split: `bios7_decomp.hpp` + `bios7_decomp.cpp` (the
  dispatcher entry points and shared header-parsing helper), with
  per-algorithm files when growth requires it.
- Per-algorithm unit tests with hand-crafted compressed payloads that match
  GBATEK's documented format byte-for-byte.
- A capstone integration test that decompresses a multi-block payload end-to-
  end through a real `Arm7Bus`.
- Updated dispatcher in `bios7_hle.cpp` — three new entries, three remaining
  callback-variant `warn_stub` entries with explicit per-SWI logging so the
  caller can identify which one was called.

**What this slice deliberately does NOT build:**
- ReadByCallback variants (`0x12`, `0x13`, `0x15`) — slice 3g.
- The callback-dispatch trampoline (re-entrant interpreter helper) —
  slice 3g.
- `Diff8bitUnFilter` (`0x16`/`0x17`) and `Diff16bitUnFilter` (`0x18`).
  These are unfilter helpers, not decompressors. ARM7 doesn't have them
  per GBATEK (`SWI 16h/17h/18h` are GBA + ARM9/DSi9 only). No-op for ARM7.
  Confirm during commit 1; if HG/SS attempts them on ARM7 we have bigger
  problems than this slice can solve.
- Cycle-accurate decompression cost. Each SWI charges 1 cycle to match
  every other HLE SWI. Real cost varies wildly with input size (microseconds
  to milliseconds). Phase 6 polish.
- Partial-result reporting. Real BIOS returns the decompressed length; our
  HLE always returns 0 (matching slice-3e convention). No game checks the
  return value of these SWIs; HG/SS proven via static ROM analysis.

### Scope boundary

**In scope:** the three SWIs in §2.1; the new `bios7_decomp.hpp` + family
file split; one new unit-test binary per SWI plus a capstone; updates to the
existing `arm7_bios_dispatch_test.cpp` to verify the three remaining
callback variants still warn-stub with their own per-SWI message; the shared
header-parsing helper for LZ77/RLE (same 32-bit layout).

**Out of scope (§3 for detail):**
- ReadByCallback variants (slice 3g).
- Callback-dispatch trampoline (slice 3g).
- `Diff*UnFilter` (ARM7 doesn't have them).
- Cycle-accurate timing.
- Wiring decompression into a real direct-boot path (no cart loader yet).

---

## 2. Goals

1. **Three ReadNormal decompressors return correct output for every test
   payload.** "Correct" means the destination buffer matches the known
   plaintext byte-for-byte, the bus access widths match GBATEK's stated
   "Write8bitUnits", and edge cases (zero-length payload, single-byte
   raw blocks, maximum-length runs, displacement-equals-1 LZ77 self-overlap)
   all behave per spec.
2. **One SWI per commit batch**, with its test landing in the same commit.
   Slice 3e style: scaffold + impl + test, possibly fused if the algorithm
   is small.
3. **Bus discipline.** All source reads go through `bus_.read*`; all
   destination writes through `bus_.write*`. Never reach into a raw buffer
   even when the source happens to live in main RAM. This keeps mirrors,
   TCM, and the future cart ROM mirror transparent.
4. **Header parsing is shared, not duplicated.** LZ77 (0x11) and RLE (0x14)
   use the same 32-bit data header (`bit 4-7 = type`, `bit 8-31 =
   decompressed size`). One helper validates and returns `(type, size)`;
   each algorithm asserts the type code it expects.
5. **`Fixed<I, F>` is not required.** Decompressors are bitwise integer code
   (LZ77 byte/halfword math, RLE counters, BitUnPack bit-shift accumulators).
   No fractional arithmetic anywhere.
6. **No new subsystem coupling.** Each decompressor takes `Arm7State&` and
   `Arm7Bus&`, same as slice 3e SWIs. No dependency on PPU, DMA, or cart.
7. **Per-SWI tests link `ds_core` only, use `REQUIRE`, run in ms.** Same
   discipline as every prior slice. No SDL.
8. **Callback variants get explicit per-SWI warn messages.** The slice-3e
   shared `bios7_decomp_stub` is replaced with three named stubs
   (`bios7_lz77_callback_stub`, `bios7_huff_callback_stub`,
   `bios7_rl_callback_stub`) so the warn line identifies which SWI was
   attempted. Helps slice 3g triage which payloads hit which path.

### 2.1 SWI coverage matrix

| # | Name | Family | This slice? | Notes |
|---|---|---|---|---|
| 0x10 | BitUnPack | decomp | **YES** | Bit-depth expansion; UnpackInfo struct in R2. |
| 0x11 | LZ77UnCompReadNormalWrite8bit | decomp | **YES** | LZ77, source from R0, 8-bit dest writes. |
| 0x12 | LZ77UnCompReadByCallbackWrite16bit | decomp | warn-stub | Slice 3g. Per-SWI warn message. |
| 0x13 | HuffUnCompReadByCallback | decomp | warn-stub | Slice 3g. Per-SWI warn message. |
| 0x14 | RLUnCompReadNormalWrite8bit | decomp | **YES** | Run-length, source from R0, 8-bit dest writes. |
| 0x15 | RLUnCompReadByCallbackWrite16bit | decomp | warn-stub | Slice 3g. Per-SWI warn message. |

**3 real implementations + 3 explicit per-SWI warn-stubs.** Total commits =
~6 (see Appendix A).

---

## 3. Non-goals

Each deferred item names the slice that will pick it up.

- **`LZ77UnCompReadByCallbackWrite16bit` (SWI 0x12).** Same algorithm as
  0x11, but source via callback and dest via 16-bit halfword writes. Slice 3g.
- **`HuffUnCompReadByCallback` (SWI 0x13).** Distinct algorithm (binary tree
  walked by single bits); also requires the callback trampoline + a 0x200-
  byte temp buffer for tree-copy random access. Slice 3g.
- **`RLUnCompReadByCallbackWrite16bit` (SWI 0x15).** Same algorithm as 0x14,
  callback source + 16-bit dest writes. Slice 3g.
- **The callback-dispatch trampoline.** A re-entrant interpreter helper that
  invokes a guest function pointer with controlled register setup, runs the
  CPU until the function returns to a sentinel address, and resumes the HLE
  handler with the return value in `R0`. Sized for its own slice because it
  introduces a new pattern (HLE-calls-guest-code) that touches CPU state
  saving, IRQ masking during callback, and PC-sentinel detection. Slice 3g.
- **`Diff8bitUnFilter` / `Diff16bitUnFilter` (SWIs 0x16/0x17/0x18).** Per
  GBATEK these are GBA + ARM9/DSi9 only. ARM7 SWI 0x16-0x18 is invalid →
  remains in slice 3e's invalid-SWI warn-and-NOP branch. If a Pokemon ROM
  attempts one, the warn surfaces it.
- **Cycle-accurate decompression cost.** 1 cycle per SWI matches every
  other HLE entry. Real cost is input-size-dependent. Phase 6.
- **Return value reporting.** Real BIOS returns the decompressed length;
  our HLE returns 0. No HG/SS code path checks this; document and move on.
- **Boundary checks for source overrun.** If a malformed payload tries to
  read past the end of source memory, our bus reads will return 0 (the
  bus's open-bus default). Real BIOS would crash; we emit a single
  `DS_LOG_WARN` and stop decompression on the first all-zero flag byte
  past the documented payload length. Documented in §5.
- **OAM/palette destination special cases.** Real BIOS, when writing to
  OAM or palette via 16-bit Vram variants, does not promote 8-bit writes.
  Not a concern for this slice — we implement only 8-bit-write variants
  here, all targeting WRAM or main RAM.

---

## 4. Architecture

One new family file (split into per-algorithm files only if growth requires
it), three new dispatcher entries, three renamed warn-stubs, three new test
binaries. No new I/O registers, no new CPU state flags, no new
`run_until` branches.

### 4.1 File layout

```
src/cpu/arm7/bios/
  bios7_hle.hpp                 MODIFIED. Add three real entry-point
                                declarations (bios7_bit_unpack,
                                bios7_lz77_uncomp_wram,
                                bios7_rl_uncomp_wram) and three per-SWI
                                callback-stub declarations
                                (bios7_lz77_callback_stub,
                                bios7_huff_callback_stub,
                                bios7_rl_callback_stub).
                                Remove bios7_decomp_stub (was the shared
                                slice-3e placeholder).
  bios7_hle.cpp                 MODIFIED. Dispatcher switch updates:
                                  case 0x10: bios7_bit_unpack
                                  case 0x11: bios7_lz77_uncomp_wram
                                  case 0x12: bios7_lz77_callback_stub
                                  case 0x13: bios7_huff_callback_stub
                                  case 0x14: bios7_rl_uncomp_wram
                                  case 0x15: bios7_rl_callback_stub
                                Lines change from ~80 to ~85.
  bios7_decomp.hpp              NEW. Declares the three real entry points,
                                the three per-SWI callback stubs, and the
                                shared header-parsing helper:
                                  struct DecompHeader { u8 type; u32 size; };
                                  DecompHeader bios7_decomp_parse_header(
                                      Arm7Bus&, u32 src_addr);
  bios7_decomp.cpp              NEW. Hosts:
                                  - bios7_decomp_parse_header
                                  - bios7_bit_unpack
                                  - bios7_lz77_uncomp_wram
                                  - bios7_rl_uncomp_wram
                                  - the three callback warn-stubs
                                Estimated ~280 lines total. Soft cap is 500.
                                If the algorithm files grow during
                                implementation, split into:
                                  bios7_decomp_lz77.cpp   (LZ77 0x11 only)
                                  bios7_decomp_rle.cpp    (RLE 0x14 only)
                                  bios7_decomp_bitunpack.cpp (BitUnPack 0x10)
                                  bios7_decomp.cpp        (header helper + stubs)
                                Decision deferred to commit time — start
                                with one file and split if any algorithm
                                exceeds ~120 lines.

tests/unit/
  arm7_bios_bitunpack_test.cpp  NEW. Bit-depth expansion correctness:
                                  - 1 → 4 with offset and zero-flag combos
                                  - 1 → 8 with same combos
                                  - 2 → 8, 4 → 8, 8 → 16, 8 → 32
                                  - Edge: zero source length (no writes)
                                  - Edge: src_width == dst_width (passthrough)
  arm7_bios_lz77_test.cpp       NEW. LZ77 correctness:
                                  - Single uncompressed block (1 raw byte)
                                  - Single compressed block (3-byte run)
                                  - Compressed block with disp=0
                                    (self-referential; valid for Wram variant)
                                  - Mixed block stream with all 8 bits set
                                    in flag byte
                                  - Edge: zero decompressed_size header
                                  - Edge: compressed run length = 18 (max,
                                    encoded as 0xF in length field + 3)
  arm7_bios_rle_test.cpp        NEW. RLE correctness:
                                  - Uncompressed block, length 1 (flag 0x00)
                                  - Uncompressed block, length 128 (max,
                                    flag 0x7F)
                                  - Compressed block, length 3 (min, flag 0x80)
                                  - Compressed block, length 130 (max,
                                    flag 0xFF)
                                  - Mixed compressed + uncompressed stream
                                  - Edge: zero decompressed_size header
  arm7_bios_decomp_dispatch_test.cpp
                                NEW. Verifies:
                                  - SWI 0x10/0x11/0x14 route to real impl
                                    (smoke-test: tiny payload produces
                                    expected output)
                                  - SWI 0x12/0x13/0x15 hit per-SWI warn-stub
                                    (no crash, R0 unchanged from input)
                                  - Stubs emit a distinct log line per SWI
                                    (verify by stubbing the log sink)
  arm7_bios_decomp_sequence_test.cpp
                                NEW. Capstone. Constructs a multi-block
                                LZ77 payload in main RAM that decodes to a
                                known 64-byte plaintext (e.g. a fragment of
                                the Pokemon HG game-name string padded to
                                64 bytes), runs the SWI through real
                                Arm7Bus, asserts byte-for-byte match.
                                Then does the same with a hand-rolled
                                RLE payload. Then a 1→8 BitUnPack expansion.
                                One test binary, three sub-cases.

  arm7_bios_dispatch_test.cpp   MODIFIED. Existing slice-3e test referenced
                                bios7_decomp_stub. Update to reference the
                                three per-SWI callback stubs. Existing
                                assertions stay; the stub names change.
```

**Five new test binaries**, taking CTest count from **50** (end of slice 3e)
to **55**.

### 4.2 Dispatcher — `bios7_hle.cpp`

The slice-3e switch:

```cpp
case 0x10: case 0x11: case 0x12: case 0x13:
case 0x14: case 0x15: return bios7_decomp_stub(s, b, n);
```

becomes:

```cpp
// Pseudocode — not to be pasted.
case 0x10: return bios7_bit_unpack(s, b);
case 0x11: return bios7_lz77_uncomp_wram(s, b);
case 0x12: return bios7_lz77_callback_stub(s, b);
case 0x13: return bios7_huff_callback_stub(s, b);
case 0x14: return bios7_rl_uncomp_wram(s, b);
case 0x15: return bios7_rl_callback_stub(s, b);
```

Six lines, each routing to a named function. The dispatcher stays well
under the 100-line target.

### 4.3 Shared header parser — `bios7_decomp_parse_header`

LZ77 (0x11) and RLE (0x14) both consume a 32-bit header at `[R0]`:

```
Bit  0-3   Reserved (must be 0)
Bit  4-7   Compressed type (1 = LZ77, 3 = RLE)
Bit  8-31  Decompressed size in bytes
```

The helper:

```cpp
// src/cpu/arm7/bios/bios7_decomp.hpp
struct DecompHeader {
    uint8_t  type;   // bit 4-7
    uint32_t size;   // bit 8-31
};

DecompHeader bios7_decomp_parse_header(Arm7Bus& bus, uint32_t src_addr);
```

Implementation:

```cpp
// Pseudocode — not to be pasted.
DecompHeader bios7_decomp_parse_header(Arm7Bus& bus, uint32_t src_addr) {
    const uint32_t hdr = bus.read32(src_addr);
    return DecompHeader{
        .type = static_cast<uint8_t>((hdr >> 4) & 0x0F),
        .size = hdr >> 8,
    };
}
```

The reserved bits 0-3 are not validated. Real BIOS doesn't check either —
games may set them to anything; we follow suit. The type byte is checked
inside each algorithm (LZ77 expects type=1, RLE expects type=3); a mismatch
emits a `DS_LOG_WARN` and decompresses anyway. This is more permissive than
hardware but lets debugging surface bad payloads while not aborting boot.

BitUnPack does not use this helper — it has a different header at `[R2]`
(see §5.1).

### 4.4 BitUnPack algorithm shape

BitUnPack reads a sequence of `src_width`-bit chunks from source memory and
writes `dst_width`-bit chunks to destination memory, with all writes
buffered into 32-bit-aligned words. Optionally adds a constant offset to
each non-zero source chunk (and to zero chunks too if the zero-data flag is
set).

```cpp
// Pseudocode — slice 3f BitUnPack outline.
uint32_t bios7_bit_unpack(Arm7State& s, Arm7Bus& bus) {
    const uint32_t src   = s.gpr[0];
    const uint32_t dest  = s.gpr[1];
    const uint32_t info  = s.gpr[2];

    const uint16_t src_len_bytes = bus.read16(info + 0);
    const uint8_t  src_width     = bus.read8 (info + 2);
    const uint8_t  dst_width     = bus.read8 (info + 3);
    const uint32_t offset_word   = bus.read32(info + 4);
    const uint32_t data_offset   = offset_word & 0x7FFFFFFFu;
    const bool     zero_flag     = (offset_word & 0x80000000u) != 0;

    uint32_t out_buffer  = 0;
    uint32_t out_bits    = 0;
    uint32_t dest_addr   = dest;

    for (uint32_t i = 0; i < src_len_bytes; ++i) {
        const uint8_t byte = bus.read8(src + i);
        for (uint32_t bit = 0; bit < 8; bit += src_width) {
            const uint32_t mask  = (1u << src_width) - 1u;
            const uint32_t chunk = (byte >> bit) & mask;
            uint32_t value;
            if (chunk != 0 || zero_flag) value = chunk + data_offset;
            else                         value = 0;

            out_buffer |= (value & ((1u << dst_width) - 1u)) << out_bits;
            out_bits   += dst_width;

            if (out_bits >= 32) {
                bus.write32(dest_addr, out_buffer);
                dest_addr += 4;
                out_buffer = 0;
                out_bits   = 0;
            }
        }
    }

    if (out_bits != 0) {
        // GBATEK: total output must be multiple of 4 bytes; if a
        // well-formed payload ends mid-word, pad with zeros and flush.
        bus.write32(dest_addr, out_buffer);
    }
    return 0;
}
```

**Validation strategy.** GBATEK lists `src_width ∈ {1, 2, 4, 8}` and
`dst_width ∈ {1, 2, 4, 8, 16, 32}`. We do not assert these — a bad value
gives garbage output, matches hardware behavior of "undefined but
deterministic." The loop `bit += src_width` walks the byte cleanly for any
power-of-two width that divides 8.

**Bus access width.** Source reads are 8-bit (`bus.read8`); the UnpackInfo
struct uses 16-bit + 8-bit + 8-bit + 32-bit reads at fixed offsets. Dest
writes are 32-bit (`bus.write32`) per GBATEK's "Data is written in 32bit
units."

**Edge: `src_len_bytes == 0`.** Loop doesn't enter, no writes emitted.
Correct.

**Edge: source unit + offset exceeds dest width.** GBATEK says "should not
exceed" but doesn't define behavior. We mask to dst_width and log nothing —
matches melonDS.

### 4.5 LZ77UnCompReadNormalWrite8bit (SWI 0x11) algorithm shape

```cpp
// Pseudocode — slice 3f LZ77 Wram outline.
uint32_t bios7_lz77_uncomp_wram(Arm7State& s, Arm7Bus& bus) {
    const uint32_t src  = s.gpr[0];
    const uint32_t dest = s.gpr[1];

    const auto hdr = bios7_decomp_parse_header(bus, src);
    if (hdr.type != 1) {
        DS_LOG_WARN("arm7/bios: LZ77 SWI 0x11 with header type %u (expected 1)",
                    hdr.type);
    }

    uint32_t src_addr  = src + 4;
    uint32_t dest_addr = dest;
    uint32_t written   = 0;

    while (written < hdr.size) {
        const uint8_t flags = bus.read8(src_addr++);
        for (int bit = 7; bit >= 0 && written < hdr.size; --bit) {
            const bool compressed = (flags >> bit) & 1u;
            if (!compressed) {
                // Block type 0: copy 1 raw byte.
                const uint8_t byte = bus.read8(src_addr++);
                bus.write8(dest_addr++, byte);
                ++written;
            } else {
                // Block type 1: 16-bit big-endian length+disp encoding.
                const uint8_t b0  = bus.read8(src_addr++);
                const uint8_t b1  = bus.read8(src_addr++);
                const uint32_t len  = ((b0 >> 4) & 0x0F) + 3;
                const uint32_t disp = (((b0 & 0x0F) << 8) | b1) + 1;
                for (uint32_t i = 0; i < len && written < hdr.size; ++i) {
                    const uint8_t byte = bus.read8(dest_addr - disp);
                    bus.write8(dest_addr++, byte);
                    ++written;
                }
            }
        }
    }
    return 0;
}
```

**Why `disp + 1`.** GBATEK: "Copy N+3 Bytes from Dest-Disp-1 to Dest." Encoded
disp = actual_disp - 1. Our code adds 1 to recover the actual displacement.

**Why `(b0 >> 4) + 3`.** GBATEK: "Bit 4-7: Number of bytes to copy (minus 3)."
Length field is N = encoded + 3, range 3-18.

**Self-overlap (disp < len).** When `disp == 1`, the inner loop reads `dest_
addr - 1` then immediately writes `dest_addr` and advances — the next
iteration reads the byte just written. This is RLE-via-LZ77 and is
*explicitly supported* by the Wram variant (only the Vram variant has the
disp=0 caveat noted in GBATEK). Tested in commit 3.

**Why `written < hdr.size` in the inner loop.** A flag byte can carry up
to 8 blocks, and a compressed block writes up to 18 bytes. If the
decompressed size doesn't align to a flag-byte boundary, the inner block
must stop mid-stream rather than over-decompress. Real BIOS does the same.

**Edge: `hdr.size == 0`.** Outer loop doesn't enter; no writes; return 0.

**Termination.** `hdr.size` is authoritative. We never read past the
documented decompressed length. A truncated source payload (smaller than
the header claims) reads zeros from open-bus; output is well-defined but
garbage past the truncation point — same as real BIOS, which crashes on
malformed input. A future cart-load step should validate header size against
source size before invoking the SWI; not a slice-3f concern.

### 4.6 RLUnCompReadNormalWrite8bit (SWI 0x14) algorithm shape

```cpp
// Pseudocode — slice 3f RLE Wram outline.
uint32_t bios7_rl_uncomp_wram(Arm7State& s, Arm7Bus& bus) {
    const uint32_t src  = s.gpr[0];
    const uint32_t dest = s.gpr[1];

    const auto hdr = bios7_decomp_parse_header(bus, src);
    if (hdr.type != 3) {
        DS_LOG_WARN("arm7/bios: RLE SWI 0x14 with header type %u (expected 3)",
                    hdr.type);
    }

    uint32_t src_addr  = src + 4;
    uint32_t dest_addr = dest;
    uint32_t written   = 0;

    while (written < hdr.size) {
        const uint8_t flag = bus.read8(src_addr++);
        const bool compressed = (flag & 0x80u) != 0;
        const uint32_t len = compressed
            ? ((flag & 0x7Fu) + 3)   // compressed: 3-130
            : ((flag & 0x7Fu) + 1);  // uncompressed: 1-128

        if (compressed) {
            const uint8_t byte = bus.read8(src_addr++);
            for (uint32_t i = 0; i < len && written < hdr.size; ++i) {
                bus.write8(dest_addr++, byte);
                ++written;
            }
        } else {
            for (uint32_t i = 0; i < len && written < hdr.size; ++i) {
                const uint8_t byte = bus.read8(src_addr++);
                bus.write8(dest_addr++, byte);
                ++written;
            }
        }
    }
    return 0;
}
```

**Length encoding.** Per GBATEK: bit 7 = mode (0=uncompressed, 1=compressed),
bit 0-6 = length (uncompressed = N-1 stored, so actual = stored+1, range
1-128; compressed = N-3 stored, so actual = stored+3, range 3-130).

**Edge: maximum-length compressed run.** Flag = 0xFF → length = 130 bytes
of one source byte. Tested in commit 4.

**Edge: maximum-length uncompressed run.** Flag = 0x7F → length = 128 raw
bytes. Tested in commit 4.

### 4.7 Per-SWI callback warn-stubs (slice 3g preview)

```cpp
// src/cpu/arm7/bios/bios7_decomp.cpp — placeholder for slice 3g.
uint32_t bios7_lz77_callback_stub(Arm7State& s, Arm7Bus& b) {
    DS_LOG_WARN("arm7/bios: SWI 0x12 LZ77UnComp(callback,Vram) not implemented (slice 3g)");
    return 0;
}
uint32_t bios7_huff_callback_stub(Arm7State& s, Arm7Bus& b) {
    DS_LOG_WARN("arm7/bios: SWI 0x13 HuffUnComp(callback) not implemented (slice 3g)");
    return 0;
}
uint32_t bios7_rl_callback_stub(Arm7State& s, Arm7Bus& b) {
    DS_LOG_WARN("arm7/bios: SWI 0x15 RLUnComp(callback,Vram) not implemented (slice 3g)");
    return 0;
}
```

Distinct messages let runtime triage. R0 is left unchanged so the caller
sees its original argument back, which is benign — the callback structure
in R3 is also untouched, and the dispatcher's `MOVS PC, LR` slice-3e exit
returns control to the SWI callsite. Any game that depends on the actual
decompression result will fail later (almost certainly a black screen or
an `IntrWait` deadlock waiting for a decompressed flag), at which point the
warn message in the log identifies which callback variant was needed.

### 4.8 What does NOT change

- Dispatcher prologue / epilogue (slice 3d/3e exception entry & `MOVS PC,
  R14`-style return).
- IRQ controller, halt fast-path, HALTCNT routing.
- Any I/O register; no new addresses.
- Any other slice-3e SWI handler.
- Bus page tables, page-table rebuilds, or bus access widths for non-
  decompressor paths.

### 4.9 Known technical debt deferred

- **`save_state` / `load_state`** for the new family: zero state to save
  (functions are stateless re-entry points, no static buffers). Phase-1
  serialization pass touches nothing here.
- **Cycle accuracy.** 1-cycle charge per SWI matches every other handler.
  Decompression is a thousands-of-cycles operation in real hardware; ours
  is instantaneous. Phase 6 polish.
- **Return value reporting.** Real BIOS returns decompressed length in R0;
  we return 0. Documented in §3 non-goals.

---

## 5. Hardware details

Each subsection cites the exact GBATEK lines (lines 95336–95491 in the
2026-04-17 snapshot) and notes any deviation our HLE makes.

### 5.1 BitUnPack (SWI 0x10)

GBATEK lines 95336–95357.

**Inputs:**
- R0: source address (no alignment requirement).
- R1: destination address (must be 32-bit-word aligned).
- R2: pointer to UnpackInfo struct (no alignment requirement noted, but
  fields imply natural alignment within the struct):
  - `+0` u16: source data length in bytes (0–0xFFFF).
  - `+2` u8:  source unit width in bits (1, 2, 4, or 8).
  - `+3` u8:  destination unit width in bits (1, 2, 4, 8, 16, or 32).
  - `+4` u32: data offset in bits 0–30, zero-data flag in bit 31.

**Behavior:**
- Walks source data chunk-by-chunk (`src_width` bits per chunk).
- For each chunk: if non-zero, add `data_offset`; if zero, add `data_offset`
  *only if* zero-data flag is set, otherwise output zero.
- Buffers output into 32-bit words and writes via 32-bit dest writes.
- Output must be a multiple of 4 bytes — guaranteed by 32-bit-write
  buffering.

**No return value.** Our HLE returns 0.

**Bus access widths.** Source: 8-bit reads (loop reads byte-by-byte;
our impl uses `bus.read8`). UnpackInfo: 16-bit + 8-bit + 8-bit + 32-bit at
fixed offsets per GBATEK layout. Dest: 32-bit writes only.

**Hardware deviation:** none.

### 5.2 LZ77UnCompReadNormalWrite8bit (SWI 0x11)

GBATEK lines 95425–95457.

**Inputs:**
- R0: source address (4-byte aligned, points to data header).
- R1: destination address.
- R2, R3: ignored for the ReadNormal variant (used by callback variants).

**Source format:**
- 32-bit data header: bit 0-3 reserved, bit 4-7 type=1, bit 8-31
  decompressed size.
- Repeating: 1 flag byte + 8 blocks (each block 1 or 2 bytes per type bit).

**Block type 0 (flag bit = 0):** 1 raw byte copied to dest.

**Block type 1 (flag bit = 1):** 2 bytes encoding (length, displacement):
- `b0[7:4]` = length - 3 (length range 3-18).
- `b0[3:0] || b1[7:0]` = displacement - 1 (12-bit displacement, range
  1-4096).
- Source for the copy: `dest - disp` (after adding 1 to recover real disp);
  GBATEK phrases it as "Dest-Disp-1" with disp being the encoded value.
- Self-overlap supported (disp < length is a valid RLE-via-LZ77).

**Wram-vs-Vram caveat.** GBATEK warns the Vram (16-bit-write) variant
breaks for disp=0 (which encodes actual_disp=1 self-overlap) because the
halfword buffering reads stale data. The Wram variant (this slice) writes
8 bits at a time and works for all encoded disp values 0x000-0xFFF
(actual_disp 1-4096).

**No return value.** Our HLE returns 0.

**Bus access widths.** Source: 8-bit reads (flag bytes and block data).
Dest: 8-bit writes.

**Hardware deviation:** none for the algorithm. Our header-type validation
warn-logs but doesn't abort on mismatch (real BIOS continues anyway).

### 5.3 RLUnCompReadNormalWrite8bit (SWI 0x14)

GBATEK lines 95460–95489.

**Inputs:**
- R0: source address (4-byte aligned, points to data header).
- R1: destination address.
- R2, R3: ignored.

**Source format:**
- 32-bit data header: bit 0-3 reserved, bit 4-7 type=3, bit 8-31
  decompressed size.
- Repeating: 1 flag byte + 1 or N data bytes.

**Flag byte:**
- Bit 7 = mode (0 = uncompressed, 1 = compressed).
- Bit 0-6 = length encoded:
  - Uncompressed: real_length = encoded + 1, range 1-128, followed by
    `real_length` raw data bytes.
  - Compressed: real_length = encoded + 3, range 3-130, followed by 1 data
    byte to be repeated `real_length` times.

**No return value.** Our HLE returns 0.

**Bus access widths.** Source: 8-bit reads. Dest: 8-bit writes.

**Hardware deviation:** header-type warn-log on mismatch (same pattern as
LZ77).

### 5.4 ReadByCallback variants (SWIs 0x12, 0x13, 0x15) — slice 3g preview

GBATEK lines 95492–95521 cover the callback contract. Summary for slice
3g:

- **R2** (for LZ77/RLE): user-defined callback parameter, opaque to BIOS,
  passed verbatim to Open.
- **R2** (for Huffman): pointer to a temp buffer, max 0x200 bytes,
  internally used by BIOS to copy the Huffman tree for random access.
- **R3**: pointer to a callback structure with five 32-bit function
  pointers:
  - `Open_and_get_32bit` — read header (the first 32-bit word).
  - `Close` — optional cleanup (NULL = none).
  - `Get_8bit` — fetch next source byte.
  - `Get_16bit` — not used.
  - `Get_32bit` — used by Huffman only.
- All callbacks may be ARM or Thumb (low bit indicates Thumb, like normal
  BX semantics).
- Open returns the header word OR a negative error code to abort.
- Close returns >=0 OK or <0 error.
- Other callbacks return raw data with no error indication.
- SWI returns: positive decompressed length, OR the signed error code from
  Open/Close.

**Why this is a separate slice.** Implementing a callback variant requires:

1. A trampoline function that takes (target PC, R0/R1/R2 inputs) and
   invokes the guest function: save current state, set up new register
   frame, set LR to a sentinel address (e.g., 0xFFFFFFF0 — an unmapped
   address we can detect), single-step the interpreter, watch for PC ==
   sentinel.
2. A re-entrancy guard so the SWI handler sits "above" the guest callback
   on a logical stack — the callback might itself trigger an IRQ which
   itself calls another SWI. This works because slice-3d's exception entry
   already saves/restores SPSR per-mode; we just need the trampoline to be
   re-entrancy-safe.
3. IRQ masking decisions: should the BIOS callback path mask IRQs? Real
   hardware doesn't (the BIOS runs in supervisor mode with `I=0` after the
   SWI prologue). Our trampoline must preserve that.
4. The Huffman tree-copy step: read tree size byte, read entire tree into
   the R2 temp buffer via Get_8bit callback, then random-access the buffer
   during decompression.

These are all isolated to slice 3g. Slice 3f does not block on them.

### 5.5 Invalid / out-of-scope SWI numbers (0x16, 0x17, 0x18)

ARM7's invalid-SWI list (from slice 3e §5.14) includes 0x16-0x19. The
GBA/ARM9 `Diff*UnFilter` SWIs at 0x16/0x17/0x18 are *not* present on ARM7.
Slice 3f does not change the slice-3e default-branch handling for these
numbers — they continue to warn-log and NOP-return.

---

## 6. Testing strategy

Five new test binaries (taking total from 50 → 55), plus one update to the
existing `arm7_bios_dispatch_test.cpp`.

### 6.1 `arm7_bios_bitunpack_test.cpp`

Coverage:
- 1→4 expansion, no offset, no zero-flag: `0xAA` (10101010 LSB-first) →
  four nibbles `1, 0, 1, 0, 1, 0, 1, 0` packed into u32 `0x10101010`.
- 1→8 expansion with offset=0x20: `0x55` (01010101 LSB-first) → bytes
  `0x21, 0, 0x21, 0, 0x21, 0, 0x21, 0` packed into u32s `0x00210021`,
  `0x00210021`.
- 1→8 with offset=0x20 and zero-flag set: same input → bytes
  `0x21, 0x20, 0x21, 0x20, …` (zero chunks also get the offset).
- 2→8 expansion, no offset: `0xE4` (11 10 01 00 from MSB, but extracted
  LSB-first in 2-bit chunks → 0,1,2,3) → bytes `0, 1, 2, 3` packed into
  u32 `0x03020100`.
- 4→8 expansion, no offset: `0x21` → bytes `1, 2`.
- 8→16 expansion, offset=0x100: byte `0x05` → u16 `0x0105`.
- 8→32 expansion, offset=0x80000000: byte `0x05` → u32 `0x80000005`.
- Edge: src_len_bytes=0 → no writes; dest buffer untouched.
- Edge: src_width = dst_width = 8, no offset → straight byte copy.

Each case constructs a small input array in a heap-allocated `Arm7Bus`
backing store (use the existing test bus pattern from
`arm7_bios_memcpy_test.cpp`), sets R0/R1/R2, calls `bios7_bit_unpack`,
asserts dest contents.

### 6.2 `arm7_bios_lz77_test.cpp`

Coverage:
- Single uncompressed block: header `{type=1, size=1}`, flag `0x00`, byte
  `0xAB` → output `[0xAB]`.
- Single compressed block (impossible for first byte; precede with a raw
  byte): header `{type=1, size=4}`, flag `0x40` (block 0=raw, block
  1=compressed, blocks 2-7 unused but flag bit 0), raw byte `0xAB`,
  compressed `(0x00, 0x00)` (length=3, disp_encoded=0 so actual_disp=1)
  → output `[0xAB, 0xAB, 0xAB, 0xAB]`.
- Self-overlap RLE-via-LZ77: header `{size=10}`, raw byte `0xCD`,
  compressed `(0x60, 0x00)` (length=9, disp_encoded=0) → output
  `[0xCD, 0xCD, ..., 0xCD]` × 10.
- Maximum-length compressed run: length encoded 0xF → real length 18.
- Mixed flag byte: flag `0x55` (alternating compressed/raw) with valid
  blocks for each — decoded against a pre-computed expected output.
- Edge: header `{size=0}` → no writes; dest untouched.

### 6.3 `arm7_bios_rle_test.cpp`

Coverage:
- Uncompressed length 1: flag `0x00`, byte `0xAB` → `[0xAB]`.
- Uncompressed length 128: flag `0x7F`, 128 bytes → those 128 bytes.
- Compressed length 3: flag `0x80`, byte `0xCD` → `[0xCD, 0xCD, 0xCD]`.
- Compressed length 130: flag `0xFF`, byte `0xEF` → `0xEF` × 130.
- Mixed stream: flag `0x80` + `0xAA` (run of 3), flag `0x02` + `[0x11,
  0x22, 0x33]` (3 raw bytes), flag `0x82` + `0xBB` (run of 5) → output
  `[0xAA, 0xAA, 0xAA, 0x11, 0x22, 0x33, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB]`.
- Edge: header `{size=0}` → no writes.

### 6.4 `arm7_bios_decomp_dispatch_test.cpp`

- For each of SWI 0x10/0x11/0x14: invoke via `arm7_bios_hle_dispatch_swi`
  with a tiny payload, assert dest contains expected output.
- For each of SWI 0x12/0x13/0x15: invoke via dispatcher with arbitrary R0;
  assert R0 unchanged, no crash, no exception thrown.
- Verify via a stubbed log sink that the three callback warn messages each
  contain a distinct substring (`"0x12"`, `"0x13"`, `"0x15"`). If
  introducing a log sink stub here is heavyweight, fall back to a
  compile-time assertion that the three stub functions exist and have
  distinct addresses; logging-content verification can wait.

### 6.5 `arm7_bios_decomp_sequence_test.cpp` — capstone

Three sub-cases in one binary, each running through real `Arm7Bus`:

1. **LZ77 end-to-end.** Hand-construct a 64-byte plaintext (`"DS:HG/SS"`
   repeated) and an LZ77-compressed payload of it (offline, computed by
   running a known LZ77 packer in a comment block in the test file —
   payload bytes appear as a literal `constexpr std::array`). Place
   payload at main-RAM offset 0x100, dest at 0x200. Set R0 = 0x0200_0100,
   R1 = 0x0200_0200. Invoke SWI 0x11 via dispatcher. Assert dest 64 bytes
   match plaintext.
2. **RLE end-to-end.** Same pattern with a 32-byte plaintext (alternating
   runs and raw segments) and an RLE-compressed payload.
3. **BitUnPack end-to-end.** 1→8 expansion of a 4-byte source bitmap
   (32 bits → 32 bytes) with offset=0x80, zero-flag clear. Source
   bytes `[0x55, 0xAA, 0x33, 0xCC]`, expected dest is the bit-by-bit
   expansion.

This is the slice's "real-game-shaped" verification — the previous unit
tests confirm individual algorithms; this one confirms the dispatcher,
header parsing, bus routing, and termination-condition logic compose
correctly.

### 6.6 Total test binary count

End of slice 3e: **50** binaries.
Adds in slice 3f:
- `arm7_bios_bitunpack_test.cpp`
- `arm7_bios_lz77_test.cpp`
- `arm7_bios_rle_test.cpp`
- `arm7_bios_decomp_dispatch_test.cpp`
- `arm7_bios_decomp_sequence_test.cpp`

End of slice 3f: **55** binaries.

---

## 7. Cross-references

- **Slice 3d** (`docs/specs/2026-04-16-arm7-core-phase1-slice3d-design.md`)
  — built the SWI dispatcher prologue + exception entry that calls
  `arm7_bios_hle_dispatch_swi`. Slice 3f reuses without modification.
- **Slice 3e** (`docs/specs/2026-04-16-arm7-core-phase1-slice3e-design.md`)
  — built the per-SWI dispatch table; slice 3f replaces the shared
  `bios7_decomp_stub` with three real impls + three per-SWI stubs.
- **Slice 3g** (planned, not yet specced) — picks up callback variants
  0x12/0x13/0x15 plus the trampoline infrastructure.
- **Bus** (`src/bus/arm7_bus.{hpp,cpp}`) — provides `read8/read16/read32`
  and `write8/write16/write32` used by all decompressors.
- **GBATEK BIOS Decompression Functions** (lines 95320–95521 in the
  2026-04-17 snapshot) — primary spec.
- **melonDS `src/ARMInterpreter_Branch.cpp` and `src/CP15.cpp`** — useful
  cross-reference if our decompressor outputs ever diverge from a real
  ROM. Not needed for spec.

---

## 8. Risk and rollback

### Risk 1 — endianness of LZ77 displacement field

The displacement field spans two bytes: `b0[3:0]` is the high nibble,
`b1[7:0]` is the low byte. GBATEK phrases it as "Bit 0-3 Disp MSBs / Bit
8-15 Disp LSBs," which is unambiguous but easy to swap mentally during
implementation. **Mitigation:** the unit test in §6.2 includes a case
where MSB ≠ LSB (e.g., disp=0x123: b0=0x?1, b1=0x23 — encoded as
disp_encoded=0x122, b0=0x?1, b1=0x22). If the impl swaps bytes the test
fails immediately.

### Risk 2 — RLE length-encoding off-by-one

Compressed = N-3, uncompressed = N-1. Easy to swap. **Mitigation:** the
test in §6.3 covers minimum and maximum for both modes (1, 128 for
uncompressed; 3, 130 for compressed). Any off-by-one shows up as a
single-byte difference and `REQUIRE` catches it.

### Risk 3 — BitUnPack output mid-flush on partial last word

GBATEK says total output must be a multiple of 4 bytes, implying
well-formed input always flushes cleanly. Real BIOS doesn't document
behavior on a partial last word. We flush the partial word to be safe.
**Mitigation:** the §6.1 test deliberately uses inputs that produce
clean multiples; if a real game produces a partial trailing word we'll
see the warn during boot triage.

### Risk 4 — Source overrun on malformed payload

A truncated source (header claims more compressed data than is present)
reads from open-bus past the source buffer. We get zeros, which decode
deterministically but produce garbage past the truncation point. Real
hardware crashes (open-bus fault or wraparound depending on region).
**Mitigation:** documented in §3 non-goals. A future cart-load step can
validate header.size against source size before invoking.

### Risk 5 — Self-overlap LZ77 reads stale dest

The LZ77 Wram variant supports disp ≤ length (RLE-via-LZ77). Our impl
calls `bus.write8(dest_addr++, byte)` then immediately `bus.read8(dest_
addr - disp)` — the read after the write *must* see the just-written byte.
This works because `Arm7Bus::write8` is synchronous (writes go through to
the backing store immediately, no write buffer). **Mitigation:** the §6.2
disp=1 self-overlap test verifies a 10-byte run-via-disp=1 produces a
correct output; if write-buffering ever lands as an optimization, this
test will catch the regression.

### Risk 6 — `bios7_decomp_stub` removal breaks slice-3e dispatch test

`arm7_bios_dispatch_test.cpp` (slice 3e) currently asserts that SWI 0x10-
0x15 all route through `bios7_decomp_stub`. After slice 3f they route
through six different functions. **Mitigation:** §4.1 lists this file as
MODIFIED; the test gets rewritten in commit 1 alongside the dispatcher
change. Tests stay green at every commit boundary.

---

## 9. Slice completion criteria

Slice 3f is "done" when *every* item below is true. Each is verifiable
without subjective judgment.

- [ ] `bios7_bit_unpack`, `bios7_lz77_uncomp_wram`, and
      `bios7_rl_uncomp_wram` all exist in `bios7_decomp.cpp` and are
      reachable through `arm7_bios_hle_dispatch_swi(0x10/0x11/0x14, …)`.
- [ ] `bios7_lz77_callback_stub`, `bios7_huff_callback_stub`, and
      `bios7_rl_callback_stub` exist with distinct warn messages,
      reachable through dispatcher cases 0x12/0x13/0x15.
- [ ] `bios7_decomp_stub` is deleted from `bios7_hle.{hpp,cpp}`.
- [ ] `arm7_bios_dispatch_test.cpp` updated to reference the per-SWI
      callback stubs. Slice-3e behavior preserved (every SWI still
      dispatches to *something*).
- [ ] Five new test binaries land:
      `arm7_bios_bitunpack_test`, `arm7_bios_lz77_test`,
      `arm7_bios_rle_test`, `arm7_bios_decomp_dispatch_test`,
      `arm7_bios_decomp_sequence_test`. Each compiles standalone and
      links only `ds_core`.
- [ ] `ctest --output-on-failure` reports 55/55 passing in Debug build.
- [ ] `clang-format` produces zero diff on the new files (PostToolUse
      hook handles this automatically).
- [ ] `ds-architecture-rule-checker` and `gbatek-reviewer` both return
      clean against the slice diff.
- [ ] `quality-reviewer` returns clean.
- [ ] No file in this slice exceeds the 500-line soft cap. If
      `bios7_decomp.cpp` approaches it, the per-algorithm split in §4.1
      executes before commit.

---

## Appendix A. Commit sequence

Six commits, each shippable on its own. Commits 2-5 each bundle the
algorithm impl with its dedicated test (slice-3e style fusion). Commit 1
is pure scaffold; commit 6 is the capstone.

### Commit 1 — scaffold + dispatcher rewire

- Add `src/cpu/arm7/bios/bios7_decomp.hpp` declaring all six entry points
  (three real + three callback stubs) and the shared header parser.
- Add `src/cpu/arm7/bios/bios7_decomp.cpp` with:
  - `bios7_decomp_parse_header` implementation.
  - Three real impls as **return-zero stubs with warn messages**
    ("0x10/0x11/0x14 not yet implemented (slice 3f scaffold)").
  - Three callback stubs as documented in §4.7.
- Update `bios7_hle.{hpp,cpp}`: remove `bios7_decomp_stub`, add the six
  new dispatch entries.
- Update `arm7_bios_dispatch_test.cpp`: assert each of the six SWI
  numbers dispatches to its expected stub function (assertions on R0
  unchanged + log-line presence).
- Add CMake entries for the new files.
- `ctest`: still 50/50 (no new tests yet; updated test still passes
  because all six stubs return 0 with R0 unchanged).

### Commit 2 — BitUnPack (SWI 0x10) + test

- Replace the `bios7_bit_unpack` scaffold with the §4.4 algorithm.
- Add `tests/unit/arm7_bios_bitunpack_test.cpp` with the §6.1 cases.
- Add CMake entry.
- `ctest`: 51/51.

### Commit 3 — LZ77 Wram (SWI 0x11) + test

- Replace `bios7_lz77_uncomp_wram` scaffold with the §4.5 algorithm.
- Add `tests/unit/arm7_bios_lz77_test.cpp` with the §6.2 cases including
  the disp=1 self-overlap case.
- Add CMake entry.
- `ctest`: 52/52.

### Commit 4 — RLE Wram (SWI 0x14) + test

- Replace `bios7_rl_uncomp_wram` scaffold with the §4.6 algorithm.
- Add `tests/unit/arm7_bios_rle_test.cpp` with the §6.3 cases.
- Add CMake entry.
- `ctest`: 53/53.

### Commit 5 — decomp dispatch test (callback stubs verified)

- Add `tests/unit/arm7_bios_decomp_dispatch_test.cpp` with the §6.4
  cases. The three real-impl smoke tests run, the three callback stubs
  are verified to log distinct messages without crashing.
- Add CMake entry.
- `ctest`: 54/54.

### Commit 6 — capstone end-to-end sequence test

- Add `tests/unit/arm7_bios_decomp_sequence_test.cpp` with the §6.5
  three sub-cases.
- Add CMake entry.
- `ctest`: 55/55.

---

## Appendix B. Decompressor cheat sheet

| SWI | Header type | Source per chunk | Dest write | Notes |
|-----|------------|------------------|------------|-------|
| 0x10 BitUnPack | UnpackInfo struct via R2 | 8-bit reads, walked in bit chunks | 32-bit | Buffered output; output multiple of 4 |
| 0x11 LZ77 Wram | 32-bit, type=1 | 8-bit reads (flag + block bytes) | 8-bit | disp 1-4096, length 3-18; self-overlap OK |
| 0x14 RLE Wram | 32-bit, type=3 | 8-bit reads (flag + data) | 8-bit | uncomp len 1-128, comp len 3-130 |
| 0x12 LZ77 Vram | header via callback | callback Get_8bit | 16-bit | Slice 3g; disp=0 broken per GBATEK |
| 0x13 Huffman | 32-bit, type=2 (via callback) | callback Get_32bit + Get_8bit | 32-bit | Slice 3g; tree copied to R2 buffer |
| 0x15 RLE Vram | header via callback | callback Get_8bit | 16-bit | Slice 3g |

LZ77 length/disp encoding (block type 1, two bytes b0 b1):
- length = ((b0 >> 4) & 0x0F) + 3
- disp = (((b0 & 0x0F) << 8) | b1) + 1
- copy len bytes from (dest - disp) to (dest)

RLE flag byte:
- bit 7 = 0 → uncompressed, len = (flag & 0x7F) + 1, followed by len raw bytes
- bit 7 = 1 → compressed, len = (flag & 0x7F) + 3, followed by 1 byte ×len

BitUnPack UnpackInfo struct (at [R2]):
- +0 u16 src_len_bytes
- +2 u8  src_width (1, 2, 4, 8)
- +3 u8  dst_width (1, 2, 4, 8, 16, 32)
- +4 u32 data_offset (bit 0-30) | zero_flag (bit 31)
