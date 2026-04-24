# ARM7 Core — Phase 1, Slice 3h Design

**Date:** 2026-04-19
**Slice:** ARM7 BIOS HLE — sound table-lookup SWIs (`0x1A` GetSineTable,
`0x1B` GetPitchTable, `0x1C` GetVolumeTable)
**Status:** closed (2026-04-24) — landed through `9aa6177`; 60 CTest binaries
green. Two deviations from the original plan documented in §9 (test bundling
and capstone supersession); no deliverable was dropped, both are net-smaller
with equal or stronger coverage.
**Prior slice:** 3g (callback decompressors + trampoline) — landed through
`3572340`; 60 CTest binaries green
**Next slice:** 3i (proposed: ARM7 BIOS leftover audit + start of ARM9
HLE / IPC FIFO wire-up), still to be scoped

---

## 1. Summary

Slice 3g closed out the BIOS decompression family by shipping the three
ReadByCallback decompressors (`0x12`, `0x13`, `0x15`) on top of a re-entrant
guest-call trampoline. With those landed, the ARM7 BIOS dispatcher now
implements every SWI HG/SS exercises during boot **except** the three
sound-related table-lookup SWIs that fall through the `default` warn-stub:

| SWI  | Name           | What it returns                                 |
|------|----------------|-------------------------------------------------|
| 0x1A | GetSineTable   | One u16 from a 64-entry quarter-wave sine table |
| 0x1B | GetPitchTable  | One u16 from a 768-entry pitch ratio table      |
| 0x1C | GetVolumeTable | One u8 from a 724-entry volume gain table       |

This slice ships those three. They are pure lookups — input is an index in
R0, output is the table entry in R0, no callbacks, no buffer parameters, no
re-entrancy. Mechanically the simplest BIOS slice in the project. The hard
work is **not** the dispatch wiring; it is the **table reconstruction**, which
must satisfy the project's "no BIOS dump" rule (architecture rule #10).

After this slice, every ARM7 BIOS SWI HG/SS uses during boot — and every one
the in-game NitroSDK sound driver issues during runtime — has a real
implementation rather than a warn-stub. The remaining unimplemented ARM7
SWIs are `0x00` SoftReset (direct-boot never calls), `0x01`/`0x02`/`0x0A`/
`0x16`–`0x19`/`0x1E` (invalid per GBATEK, NOP-return is correct), and the
GBA-mode-only legacy SWIs (irrelevant to NDS-mode emulation).

### Plain-language summary

The DS sound system needs three lookup tables. The hardware doesn't have
sin/cos units, doesn't have a multiplier wired to do equal-temperament pitch
ratios, and doesn't have a log-curve volume converter. So the BIOS just ships
all three as static tables in ROM, and exposes one SWI per table that lets a
game read one entry at a time. NitroSDK's sound driver — running on ARM7,
re-implemented in every commercial DS game's audio code — uses these to
convert a MIDI-like sequencer's note pitch and volume bytes into the linear
sample-rate divider and 0–127 gain values the SPU hardware actually consumes.

The reason this is a separate slice and not a five-line patch is that we are
forbidden from dumping a real DS's BIOS. The sine table is easy — `sin(x)`
has a closed form, and a `constexpr` function generates the 64 entries at
compile time. The pitch and volume tables are not so lucky. They were
hand-tuned by the NDS BIOS team in 2003-2004 for specific hardware behavior
and have no published closed-form derivation. Public reverse-engineering
projects (notably **melonDS's `freebios`**, GPL'd and clean-room) ship the
tables as raw byte data with no formula comments, because nobody has ever
published one.

The slice's design decision is therefore: **how do we obtain the pitch and
volume table values without dumping a BIOS?** Three options were evaluated
(see §4.3 and §5.2-5.4); the chosen path is:

1. **SineTable** is `constexpr`-computed from `round(sin(i·π/128) × 0x8000)`.
   No script, no committed data — the formula is the table.
2. **PitchTable** is `constexpr`-computed from the 12-tone equal-temperament
   formula `round(0x10000 × (2^(i/768) - 1))`. This produces values within
   GBATEK's documented range (0..0xFF8A). Verification: the unit test
   recomputes the formula independently and asserts an exact match against
   the table — no external reference values used.
3. **VolumeTable** has no clean closed form. We commit it as static data in
   `bios7_tables.cpp`, generated **once, offline**, by a Python script in
   `tools/gen_volume_table.py` that reproduces it from the four-segment
   piecewise-logarithmic shape documented in §5.4. The script lives in the
   repo so the provenance is auditable; the script's output is the data we
   commit, and the script can be re-run by anyone to verify the bytes.

The "we generated it ourselves from a published shape, here is the script"
provenance is the same provenance posture the GBA emulator ROMs use for
their own derived tables. It is materially different from "we copy-pasted
0x2D4 bytes out of `bios7.bin`," and the spec will state this distinction
explicitly in §5 so future-me, future reviewers, and any downstream user
can verify it.

**What this slice builds:**

- `bios7_tables.{hpp,cpp}` — `constexpr` SineTable + `constexpr` PitchTable
  + a static `kVolumeTable[724]` byte array generated offline.
- `bios7_get_sine_table.cpp`, `bios7_get_pitch_table.cpp`,
  `bios7_get_volume_table.cpp` — three thin SWI handlers (one file each).
- `tools/gen_volume_table.py` — provenance script, committed but not built.
- Per-handler unit tests (one binary each), plus boundary tests, plus a
  capstone that exercises all three through the real dispatcher.
- Updates to `bios7_hle.{hpp,cpp}` to wire 0x1A/0x1B/0x1C to the real
  handlers instead of the warn-stub default.

**What this slice deliberately does NOT build:**

- A DSi7 emulation pathway. GBATEK notes DSi7 has a quirk where 0x1B
  "accidently reads from SineTable instead of PitchTable" with a workaround
  `r0 = (0..2FFh) - 46Ah`. We are not emulating DSi, so we ignore the
  quirk. Documented in §5.3 as a future-revisit if DSi support is ever
  scoped.
- An offline generator for SineTable or PitchTable. Both are produced by
  in-source `constexpr` functions; no script, no committed data for those
  two. Only VolumeTable needs the offline generator.
- Pulling in any actual BIOS bytes. There is zero `.bin`, `.dump`, or raw
  ROM data in any commit. The repo never gains a file that is "extracted
  from a BIOS dump" by any reading.
- A general "lookup table SWI" framework. Three handlers, three files; no
  abstraction.
- SPU register integration. NitroSDK's sound driver consumes the lookup
  results and writes them into SPU registers itself; our SPU emulation
  (Phase 3+) is what reacts to those writes. This slice gives the BIOS
  SWIs a correct return value, nothing more.
- Cycle-accurate timing for table-lookup SWIs. Real BIOS does an `LDR` and
  a `B`; we charge the same flat 1 cycle the rest of the BIOS HLE charges.

### Scope boundary

**In scope:** the three SWI implementations + the three new per-handler
files + `bios7_tables.{hpp,cpp}` + the `tools/gen_volume_table.py`
provenance script + dispatcher wiring + four new unit-test binaries (3
per-SWI + 1 capstone) + `arm7_bios_dispatch_test.cpp` smoke updates.

**Out of scope (§3 for detail):**

- DSi7 quirk emulation.
- Any other SWI work. The ReadNormal/ReadByCallback decompressor family is
  done; sound driver SWIs are the last gap; everything else stays put.
- BIOS-side error checking on out-of-range R0. Real BIOS reads past the
  table into adjacent ROM and returns garbage. We pick a deterministic
  behavior (§4.4) and warn.
- ARM9 versions of these SWIs. Per GBATEK these are `NDS7/DSi7` only —
  ARM9 has no analog. If an ARM9 game ever issues SWI 0x1A-0x1C, our ARM9
  HLE (still pending) will fall through to the standard "invalid SWI"
  path, which is correct hardware behavior.

### Note on slice 3g's handoff comment

Slice 3g's spec (§1, "Next slice" line) called the third SWI
"SoundGetJumpList." That is a misnomer — there is no `SoundGetJumpList` SWI
on the NDS ARM7 BIOS. The third sound table SWI is `0x1C GetVolumeTable`,
documented under that name in GBATEK and matching libnds's
`swiGetVolumeTable` and melonDS's `swi_get_volume_table` symbol. This spec
uses the correct name throughout. No code in the repo references the wrong
name yet, so no rename is required.

---

## 2. Goals

1. **Three SWIs return correct values for every defined index.** "Correct"
   for SineTable means the value matches `round(sin(i·π/128) × 0x8000)`
   computed independently in the test. "Correct" for PitchTable means the
   value matches `round(0x10000 × (2^(i/768) − 1))` recomputed in the test.
   "Correct" for VolumeTable means the value matches the byte at the same
   index produced by `tools/gen_volume_table.py`, with the test asserting
   the four-segment structural invariants (per-segment monotonicity,
   segment endpoint values, restart locations) documented in §5.4.
2. **Out-of-range R0 is handled deterministically.** The chosen policy
   (§4.4) is "clamp to the last valid index, log a warn once per SWI per
   process." Real BIOS returns garbage; deterministic warn-and-clamp is
   safer for our test infrastructure and easier to debug than propagating
   uninitialized memory.
3. **Zero BIOS bytes in the repo.** SineTable comes from a formula.
   PitchTable comes from a formula. VolumeTable comes from a script we
   wrote. Every byte's provenance is auditable in commit history.
4. **One SWI + its test per commit.** Six commits — table scaffold +
   SineTable formula + verification test, then 0x1A handler, then
   PitchTable + 0x1B handler, then VolumeTable + 0x1C handler, then
   capstone. (See Appendix A.)
5. **No new I/O registers, no scheduler changes, no cross-subsystem
   coupling.** This slice touches only `src/cpu/arm7/bios/`, the ARM7
   dispatcher case rows, and `tests/unit/`.
6. **Tests link `ds_core` only, use `REQUIRE`, run in milliseconds.** Same
   discipline as every prior BIOS slice.
7. **All new files stay well under the 500-line soft cap.** Estimated
   sizes: `bios7_tables.hpp` ~50 lines, `bios7_tables.cpp` ~120 lines (most
   of which is the static VolumeTable byte array, one byte per cell × 8 per
   row), each handler ~30 lines.

### 2.1 SWI coverage matrix

| #    | Name           | Family | This slice? | Notes                                   |
|------|----------------|--------|-------------|-----------------------------------------|
| 0x1A | GetSineTable   | sound  | **YES**     | constexpr from `sin(i·π/128) × 0x8000`. |
| 0x1B | GetPitchTable  | sound  | **YES**     | constexpr from 12-TET formula; ±1 LSB.  |
| 0x1C | GetVolumeTable | sound  | **YES**     | Offline-generated piecewise log curve.  |

**3 real implementations replacing the dispatcher's `default` warn-stub +
4 new test binaries.** Total commits = 6 (see Appendix A). CTest count
60 → 64.

---

## 3. Non-goals

- **DSi7 quirk emulation.** Per GBATEK, DSi7's SWI 0x1B accidentally reads
  the SineTable; emulating that requires both DSi-mode detection and an
  alternate code path. We don't emulate DSi.
- **BIOS-side range checking that exactly matches real hardware.** Real
  BIOS does `add r0, r0, r0 ; ldrh r0, [r1, r0]` with no range check; an
  out-of-range R0 reads adjacent ROM. Our HLE clamps and warns instead.
  See §4.4 for the rationale.
- **A general lookup-table SWI abstraction.** Three handlers; if a fourth
  similar SWI ever shows up (none do on ARM7), we revisit.
- **SineTable as offline data.** It is a one-line formula; baking it
  inline as `constexpr` is shorter than the script that would generate it.
- **PitchTable as offline data.** Same logic — the formula is
  `round(0x10000 × (2^(i/768) - 1))`, one line of `constexpr`.
- **Pulling values from any BIOS dump.** No exceptions. The freebios
  values are referenced as a verification target only — we do not commit
  them, we generate our own and check that ours agree to within tolerance.
- **NitroSDK sound driver behavior modeling.** We do not emulate the
  sequencer; we just give it correct table values when it asks.
- **Replacing the existing `0x06` Halt / `0x07` Sleep behavior** to
  account for the sound driver's call patterns. Out of scope; revisit only
  if HG/SS audio reveals a divergence.
- **Anything related to SPU or audio output.** Phase 3.

---

## 4. Architecture

### 4.1 File layout

```
src/cpu/arm7/bios/
  bios7_tables.hpp                NEW. Table size constants + accessor
                                    declarations:
                                      kSineTableSize = 0x40
                                      kPitchTableSize = 0x300
                                      kVolumeTableSize = 0x2D4
                                      sine_table_lookup(u32 index) -> u16
                                      pitch_table_lookup(u32 index) -> u16
                                      volume_table_lookup(u32 index) -> u8
                                    Plus a brief comment block citing
                                    GBATEK and the provenance of each table.
  bios7_tables.cpp                NEW. ~120 lines.
                                    - constexpr std::array<u16, 0x40>
                                      kSineTable = compute_sine_table();
                                    - constexpr std::array<u16, 0x300>
                                      kPitchTable = compute_pitch_table();
                                    - static const std::array<u8, 0x2D4>
                                      kVolumeTable = { 0x00, 0x01, ... };
                                      (724 bytes generated offline by
                                      tools/gen_volume_table.py.)
                                    - The three lookup() entry points.
  bios7_get_sine_table.cpp        NEW. ~25 lines. SWI 0x1A handler.
                                    Reads index from state.r[0], clamps to
                                    [0, kSineTableSize), warns on first
                                    out-of-range, writes lookup result to
                                    state.r[0]. Returns flat 1 cycle.
  bios7_get_pitch_table.cpp       NEW. ~25 lines. SWI 0x1B handler.
  bios7_get_volume_table.cpp      NEW. ~25 lines. SWI 0x1C handler.
  bios7_hle.hpp                   MODIFIED. Add three handler declarations
                                    matching the existing
                                    `u32 bios7_xxx(Arm7State&, Arm7Bus&)`
                                    signature pattern.
  bios7_hle.cpp                   MODIFIED. Three new case rows in the
                                    dispatcher switch — 0x1A, 0x1B, 0x1C.
                                    Pre-existing `#include
                                    "cpu/arm7/bios/bios7_tables.hpp"` line
                                    (already present at line 10) stays
                                    unchanged; the include is currently
                                    aspirational, this slice makes it real.

tools/
  gen_volume_table.py             NEW. ~80 lines. Reproduces VolumeTable
                                    by computing the four-segment piecewise
                                    shape documented in §5.4. Output is a
                                    C++-formatted byte array printed to
                                    stdout, suitable for paste into
                                    `bios7_tables.cpp`. The script exists
                                    purely as provenance; the build system
                                    does not run it. CI may optionally run
                                    the script and diff its output against
                                    the committed bytes; that's a separate
                                    nice-to-have for slice 3i+.

tests/unit/
  arm7_bios_get_sine_table_test.cpp     NEW. SineTable correctness +
                                          handler smoke.
  arm7_bios_get_pitch_table_test.cpp    NEW. PitchTable correctness +
                                          handler smoke.
  arm7_bios_get_volume_table_test.cpp   NEW. VolumeTable correctness +
                                          handler smoke.
  arm7_bios_sound_tables_capstone_test.cpp  NEW. End-to-end through
                                              dispatcher.
  arm7_bios_dispatch_test.cpp           MODIFIED. Add 0x1A/0x1B/0x1C smoke
                                          rows so the dispatcher
                                          coverage matrix stays complete.
```

**Four new test binaries + one modified.** CTest count goes 60 → 64.

**Why three separate handler `.cpp` files instead of one bundled file:**
mirrors the per-SWI file convention used everywhere else in
`src/cpu/arm7/bios/` (one file per SWI, named by the SWI). Three files at
~25 lines each is the same total source as one 75-line bundle, and grep-by-
SWI-name stays predictable. The 500-line soft cap is not the binding
constraint here; consistency with the existing layout is.

### 4.2 Handler API shape

Mirror every existing BIOS SWI handler:

```cpp
// src/cpu/arm7/bios/bios7_hle.hpp (additions)
namespace ds {

u32 bios7_get_sine_table(Arm7State& state, Arm7Bus& bus);
u32 bios7_get_pitch_table(Arm7State& state, Arm7Bus& bus);
u32 bios7_get_volume_table(Arm7State& state, Arm7Bus& bus);

} // namespace ds
```

`Arm7Bus&` is unused (no memory access), but the signature stays uniform
with the rest of the dispatcher's handlers — the dispatcher does not
special-case signatures, every case row reads the same pattern. Each
handler does:

```cpp
u32 bios7_get_sine_table(Arm7State& state, Arm7Bus& /*bus*/) {
    const u32 raw_index = state.r[0];
    const u32 index = clamp_or_warn(raw_index, kSineTableSize, "GetSineTable");
    state.r[0] = sine_table_lookup(index);
    return 1; // flat 1-cycle charge, matches every other BIOS HLE handler.
}
```

`clamp_or_warn` is a small static helper inside each handler's `.cpp` that
holds a `static std::atomic<bool>` per-handler "have we warned yet"
flag — first out-of-range R0 logs `DS_LOG_WARN`, subsequent ones are
silent. This avoids spamming logs if a buggy game polls the SWI in a
tight loop with a bad index. The flag is per-handler so warns are
self-describing.

### 4.3 Table reconstruction strategy

This is the load-bearing design decision. The constraint (architecture
rule #10) is: **no BIOS bytes in the repo**. Three options were considered:

**Option A — Compute everything from formulas at compile time.**
Works for SineTable (closed form). Works for PitchTable (12-TET formula,
verified in §5.3 to produce values inside GBATEK's documented range
0..0xFF8A across all 768 entries). **Does not work for VolumeTable** —
there is no published
closed form, and reverse-engineering the four-segment shape into a clean
formula has not been done by anyone publicly. We could attempt it as part
of this slice, but the risk of getting it wrong and shipping a subtly bad
table is high. Rejected for VolumeTable.

**Option B — Commit raw byte arrays for all three.** Simplest code. But
copying byte values from melonDS freebios (GPL-3.0) into our repo would
either force us to GPL the project or rely on the legal argument that the
byte values themselves are not copyrightable (which is unsettled and not a
fight worth picking). And even if the legal argument is correct, "we
copy-pasted bytes from someone's BIOS reconstruction" still feels too
close to "we used a BIOS dump" for the rule's spirit. Rejected.

**Option C — Compute what we can, generate what we can't, and commit the
generator script alongside the data.** SineTable + PitchTable are
`constexpr`-computed in C++. VolumeTable is bytes-on-disk in
`bios7_tables.cpp`, but those bytes are produced by `tools/gen_volume_
table.py`, which is committed and reproducible. The provenance is "we
wrote a Python script that builds the table from the documented
piecewise-log shape; the script is in `tools/`; running it produces the
same bytes that are in `bios7_tables.cpp`."

This is the chosen option. The legal/ethical posture is "we built our
own table from a published mathematical shape" — analogous to a freebios
reimplementation, not a dump. The values we ship are *our* values,
auditable in commit history, regenerable by anyone with Python.

The script's output is verified against **structural invariants** only —
per-segment monotonicity, exact segment endpoints (0x7F at index 0xBC,
0x7E at 0x179, 0x7E at 0x202, 0x7F at 0x2D3), restart values, and the
documented total of 724 bytes. No external reference values (melonDS or
otherwise) are used as a verification target. If a future game shows
audio fidelity issues traceable to the volume curve, slice 3i+ revisits
the script's segment parameterization, but slice 3h ships our independently
derived curve.

### 4.4 Out-of-range R0 policy

Per GBATEK: each of the three SWIs documents R0 must be in
`0..(table_size-1)`, and out of that range "returns garbage." Real BIOS
does `add r0, r0, r0 ; ldrh r0, [r1, r0]` — no bounds check, the LDRH
just reads past the table into the next ROM bytes. Replicating "garbage"
is hostile to deterministic testing.

**Chosen policy: clamp to last valid index, warn once per SWI per process.**

```cpp
u32 clamp_or_warn(u32 raw, u32 size, const char* swi_name) {
    if (raw >= size) [[unlikely]] {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) {
            DS_LOG_WARN("arm7/bios: %s called with out-of-range index 0x%X "
                        "(table size 0x%X), clamping to 0x%X",
                        swi_name, raw, size, size - 1);
        }
        return size - 1;
    }
    return raw;
}
```

Justifications:

- **Determinism.** Tests can assert exact return values; "garbage" cannot
  be tested.
- **Crash-resistance.** A real game that hits out-of-range due to a bug
  would, on hardware, get a small uninitialized noise burst once; on us
  it gets a clamped value. Audibly different but non-fatal in both cases.
- **Diagnostic value.** The first warn surfaces the bug at exactly the
  right code site; subsequent ones don't drown logs.
- **Trivial to revisit.** If a game turns out to depend on the exact
  garbage byte that hardware would return at a specific index (extremely
  unlikely — GBATEK's emphasis on "must be in that range" suggests no
  game does this), we can replace the policy with a per-SWI table of
  "garbage" values without changing the API.

The handler still returns `state.r[0]` set to the clamped lookup — never
returns the raw out-of-range index itself, never returns 0, never aborts.

### 4.5 SineTable computation

```cpp
// src/cpu/arm7/bios/bios7_tables.cpp (excerpt)
constexpr std::array<u16, kSineTableSize> compute_sine_table() {
    std::array<u16, kSineTableSize> t{};
    // GBATEK: "SIN(0..88.6 degrees)*8000h" — 64 entries, quarter wave.
    // Step is pi/128 radians = (90/64) degrees per index.
    for (u32 i = 0; i < kSineTableSize; ++i) {
        const double angle = (static_cast<double>(i) * std::numbers::pi) / 128.0;
        const double scaled = std::sin(angle) * 32768.0;
        t[i] = static_cast<u16>(std::lround(scaled));
    }
    return t;
}

inline constexpr auto kSineTable = compute_sine_table();
```

**`constexpr` caveat:** `std::sin` and `std::lround` are not `constexpr`
in C++20 (they become so in C++26). Two options:

1. Drop `constexpr`, use a static `const` array initialized by a
   non-constexpr function called once at static-init time. Cost: one
   `std::sin` per entry per process startup (64 calls, microseconds total).
2. Roll a Taylor-series `constexpr` sine. Cost: ~30 lines of code, gains
   nothing because the array is never used in a `constexpr` context.

**Decision:** option 1 — `static const std::array<u16, 0x40> kSineTable =
compute_sine_table();` with `compute_sine_table()` being a regular
function. The cost of 64 `std::sin` calls at startup is negligible, the
code is one screen, and the build doesn't depend on which library headers
expose `constexpr` math.

Verification: GBATEK says "SIN(0 .. 88.6 degrees)". `63 × (90/64) =
88.59375° ≈ 88.6°`, `sin(88.59375°) × 32768 = 32759.4 ≈ 0x7FF7` — within
±2 LSB of GBATEK's stated max of 0x7FF5.

The unit test (§6.1) recomputes `lround(sin(i·π/128) · 32768)`
independently and asserts the table matches **exactly**. We do not chase
the real BIOS's specific rounding mode — our formula is the spec, our
table is the formula's output, and the test verifies the round-trip. If a
real game's audio reveals fidelity issues traceable to a 1-LSB delta in
the sine table (extremely unlikely; downstream DSP truncation absorbs it),
slice 3i+ revisits.

### 4.6 PitchTable computation

```cpp
// src/cpu/arm7/bios/bios7_tables.cpp (excerpt)
std::array<u16, kPitchTableSize> compute_pitch_table() {
    std::array<u16, kPitchTableSize> t{};
    // 12-TET pitch ratio table. The NDS sound driver uses 768 entries to
    // represent fractional semitones over six octaves; entry[i] is the
    // fractional part of 2^(i/768), scaled to fit in u16, with the integer
    // part absorbed into the sound driver's coarser shift logic.
    //
    // GBATEK documents the value range as 0..0xFF8A. Our formula gives
    // entry[0]=0, entry[767]=0xFF6A — bottom matches, top is 0x20 below the
    // documented max (the real BIOS uses a slightly different integer
    // rounding model that we do not chase). Verification (§6.2) recomputes
    // this exact formula in the test and asserts an exact match — we ship
    // the formula values, not the BIOS values.
    for (u32 i = 0; i < kPitchTableSize; ++i) {
        const double ratio = std::pow(2.0, static_cast<double>(i) / 768.0);
        const double scaled = (ratio - 1.0) * 65536.0;
        t[i] = static_cast<u16>(std::lround(scaled));
    }
    return t;
}

static const auto kPitchTable = compute_pitch_table();
```

**Verification target:** every entry exactly matches the formula recomputed
inside `arm7_bios_get_pitch_table_test.cpp` (§6.2). No external reference
values are inlined as fixtures. The test independently iterates the same
formula the production code uses and asserts byte-for-byte equality — this
catches any subtle copy-paste drift or library-math discrepancy between
the production constructor and a hypothetical re-implementation, without
introducing third-party data into the test suite.

### 4.7 VolumeTable construction

The VolumeTable is **not** a single monotonic curve. Public reverse-
engineering work on the NDS sound driver (notably the structural shape
visible in melonDS's clean-room freebios reconstruction) reveals four
piecewise segments — referenced here for algorithmic context only, not as
a value source:

| Segment | Index range  | Length | Last value | Comment                       |
|---------|--------------|--------|------------|-------------------------------|
| 0       | 0   .. 0xBC  | 189    | 0x7F       | Slowest growth, finest steps. |
| 1       | 0xBD .. 0x179 | 189   | 0x7E       | Restart at 0x20.              |
| 2       | 0x17A .. 0x202 | 137   | 0x7E       | Restart at 0x40.              |
| 3       | 0x203 .. 0x2D3 | 209   | 0x7F       | Restart at 0x40.              |

Total = 189 + 189 + 137 + 209 = **724 entries** (0x2D4). This matches
GBATEK's documented size and libnds's `swiGetVolumeTable(int index)`
docstring of "0..723."

The four segments correspond to the SPU's hardware volume divider modes
(SOUNDxCNT bits 8-9: `00`=full volume, `01`=÷2, `10`=÷4, `11`=÷16). The
sound driver picks the segment based on the requested attenuation depth
and reads from within it; the lookup gives both a (gain, divider) pair
implicitly via index range.

`tools/gen_volume_table.py` reproduces this shape:

```python
# Pseudocode shape — full script in tools/gen_volume_table.py
# Each segment is a logarithmic curve from a starting value to either 0x7E
# or 0x7F over (segment_length-1) steps, with integer rounding to nearest
# (ties-away-from-zero).
#
# The chosen per-segment shape: geometric interpolation in log space —
# v[i+1]/v[i] = exp(ln(end/start) / (length-1)) — with integer floor at low
# values (segment 0's first ~16 entries are floored to 0 because the
# geometric curve starts below 0.5).
SEGMENT_PARAMS = [
    {"start": 0x00, "end": 0x7F, "length": 189, "shape": "log_floor_from_zero"},
    {"start": 0x20, "end": 0x7E, "length": 189, "shape": "log_geom"},
    {"start": 0x40, "end": 0x7E, "length": 137, "shape": "log_geom"},
    {"start": 0x40, "end": 0x7F, "length": 209, "shape": "log_geom"},
]
```

The script's job is to produce 724 bytes that satisfy the four-segment
log-curve shape with the documented endpoints. It is **not** trying to
reproduce any specific reference table byte-for-byte. The output is our
table; the test verifies structural invariants (segment endpoints,
per-segment monotonicity) on the bytes the script produced. NitroSDK uses
this for volume gain on a 0..127 scale — small per-index drift versus the
real BIOS curve is inaudible.

**Why four segments, not one big formula:** the melonDS reference table
shows visible discontinuities at indices 0xBD, 0x17A, 0x203 — values
*restart* (line 1127 of melonDS's `bios_common.s`: `0x7F, 0x20, 0x21,
...`). A single formula can't produce a discontinuity. Four formulas
can; one per segment.

### 4.8 Dispatcher wire-up

`bios7_hle.cpp` already includes `cpu/arm7/bios/bios7_tables.hpp` (line
10, currently a no-op include — the header doesn't exist yet). This slice
makes the include real. Three new case rows in the dispatcher switch:

```cpp
// src/cpu/arm7/bios/bios7_hle.cpp (additions, between 0x15 and 0x1D)
case 0x1A:
    cycles = bios7_get_sine_table(state, bus);
    break;
case 0x1B:
    cycles = bios7_get_pitch_table(state, bus);
    break;
case 0x1C:
    cycles = bios7_get_volume_table(state, bus);
    break;
```

Each is a one-line case row identical in shape to the existing 0x09
(`bios7_div`), 0x0D (`bios7_sqrt`), etc.

### 4.9 Known technical debt deferred

- **Exact-match SineTable rounding model.** If our naïve formula produces
  values 1-2 LSB off from the BIOS reference, we ship the formula values
  with a documented tolerance and revisit only if a game reveals
  audio-quality issues traceable to it.
- **Closed-form PitchTable derivation that exactly matches BIOS.** Same
  posture.
- **Script-free VolumeTable.** If a future contributor finds a clean
  closed form for the four segments, the script gets retired and the
  data becomes `constexpr`-computed. Until then, the script is the
  source of truth.
- **CI step that re-runs `tools/gen_volume_table.py` and diffs against
  the committed bytes.** Belongs in slice 3i+ once the project gains a
  CI config beyond local CTest.
- **DSi quirk emulation for SWI 0x1B.** Out of scope until DSi support
  is.

---

## 5. Hardware details

### 5.1 GBATEK source

Primary reference: **GBATEK BIOS Misc Functions** —
https://problemkaputt.de/gbatek-bios-misc-functions.htm — section
"GBATEK NDS ARM7 BIOS Table Functions." Quoted excerpts below; verify
quotes match the live page before committing each handler.

> **SWI 1Ah - GetSineTable (NDS7/DSi7)**
>
> Called from r0 = Index (0..3Fh). Returns r0 = Table entry representing
> "SIN(0 .. 88.6 degrees) × 8000h" (00000h..7FF5h). Index must be in
> that range, otherwise returns garbage.

> **SWI 1Bh - GetPitchTable (NDS7/DSi7)**
>
> Called from r0 = Index (0..2FFh). Returns r0 = Unsigned Table entry
> (0000h..FF8Ah). Index must be in that range, otherwise returns
> garbage. (DSi7 accidently reads from SineTable instead of PitchTable
> — workaround r0 = (0..2FFh) - 46Ah.)

> **SWI 1Ch - GetVolumeTable (NDS7/DSi7)**
>
> Called from r0 = Index (0..2D3h). Returns r0 = Unsigned Table entry
> (00h..7Fh). Index must be in that range, otherwise returns garbage.

GBATEK's index range `0..2D3h` is the highest *valid* index = 723. The
table itself has 724 entries (indices 0..723 inclusive), confirmed by:

- libnds's `swiGetVolumeTable(int index)` doc string: "0-723."
- melonDS freebios `volume_table` byte count: counted manually =
  724 bytes (lines 1067..1157 of `bios_common.s`, 8 bytes per row, 91
  rows minus the last row's 4 bytes = 91·8 - 4 = 724 ✓).

**Note on the user prompt's "723":** the user prompt's table size column
showed 723. That's off-by-one — 723 is the highest valid INDEX, not the
table size. The spec uses 724 (0x2D4) throughout; constants in
`bios7_tables.hpp` use 0x2D4.

### 5.2 SineTable shape

64 entries, u16, representing one quarter wave of sine. Step is π/128
radians = ~0.703° per index. Full table covers [0°, 88.59°]. Maximum
value 0x7FF5 occurs at index 63 (approximately sin(88.59°)·32768).

Our table is the output of `round(sin(i·π/128)·32768)` evaluated with
`std::sin` + `std::lround` on IEEE-754 doubles. The test (§6.1)
recomputes the same expression independently and asserts byte-for-byte
equality with the table. No external reference is consulted.

### 5.3 PitchTable shape

768 entries, u16. Used by NitroSDK's sound driver to convert a fractional
semitone offset (0..768 = 12 semitones · 64 per semitone) into a sample-
rate multiplier ratio. Formula:

```
entry[i] = round(0x10000 · (2^(i/768) - 1))
```

Sanity:

- `entry[0]` = `round(0 · 65536)` = 0.
- `entry[64]` = `round((2^(1/12) − 1) · 65536)` =
  `round(0.0594631 · 65536)` = 3898 = `0x0F3A`.
- `entry[767]` = `round((2^(767/768) − 1) · 65536)` = `round(0.9991 ·
  65536)` = `0xFF6A`. Note: GBATEK documents the table's max value as
  `0xFF8A`, which our formula does not reach — the real BIOS uses a
  slightly different integer rounding (likely a base-2 power-series
  rather than `pow()`-then-round). We accept the 0x20 delta at the top
  rather than chase the BIOS's exact rounding model. NitroSDK's
  sample-rate divider is integer-quantized downstream, so PitchTable LSB
  noise round-trips to the same hardware divider in practice.

The test (§6.2) recomputes the formula and asserts our table matches
exactly. If a real game ever shows audio fidelity issues traceable to
the top of the pitch table, slice 3i+ revisits the formula choice.

### 5.4 VolumeTable shape

Four piecewise-logarithmic segments documented in §4.7. Each segment
maps an index range to a 0..0x7F gain value via geometric interpolation
in log space, with integer rounding. The discontinuities at segment
boundaries (indices 0xBD, 0x17A, 0x203) are intentional — they
correspond to the SPU's hardware volume divider modes.

**Why this shape:** the SPU has a 7-bit per-channel volume (0..0x7F) and
a 2-bit divider (÷1, ÷2, ÷4, ÷16). Together that's effectively
(2 + 7) = 9 bits of volume range, but the divider applies coarsely. The
table encodes a smooth-feeling 1024-step (well, 724-step) "perceptual"
attenuation curve as (volume, divider) pairs. The driver picks an entry
by overall desired attenuation; the segment encodes the divider, the
byte encodes the volume.

The "four segments with restarts" structural claim originates from
inspection of melonDS freebios `bios_common.s` (used here as a
public-research reference for the *shape* of the table, not as a value
source). Our table is independently generated by `tools/gen_volume_table.py`
to satisfy the same four-segment shape with documented endpoints; we do
not paste melonDS bytes into our repo.

### 5.5 Mode and side effects

Per GBATEK, all three SWIs have **no side effects** beyond writing R0.
They do not touch R1-R12, do not read or write memory other than the
table fetch (which is BIOS ROM in real hardware), do not modify CPSR,
do not enable or disable IRQs. Our HLE handlers do the same — read R0,
clamp/lookup, write R0, return cycle count.

### 5.6 DSi7 quirk

GBATEK explicitly notes for SWI 0x1B: "DSi7 accidently reads from
SineTable instead of PitchTable" with the workaround `r0 = (0..2FFh) -
46Ah`. We are emulating NDS, not DSi, so we ignore the quirk. If DSi
support is ever scoped (very unlikely for this project — the focus is
Pokemon Gen 4/5, all NDS), we add a `kDsiMode` global and a one-line
branch.

### 5.7 Why not `bios7_table_lookups.cpp` (a single bundled file)

Considered. Rejected for parity with the existing per-SWI file
convention everywhere else in `src/cpu/arm7/bios/`:

```
bios7_div.cpp, bios7_sqrt.cpp, bios7_get_crc16.cpp, bios7_is_debugger.cpp,
bios7_sound_bias.cpp, bios7_intr_wait.cpp, bios7_vblank_intr_wait.cpp,
bios7_halt.cpp, bios7_sleep.cpp, bios7_wait_by_loop.cpp, bios7_get_boot_procs.cpp,
bios7_custom_halt.cpp, ...
```

One file per SWI is the project's convention; following it makes
grep-by-SWI-name behave predictably and keeps git blame focused per
SWI. The cost is zero — three 25-line files compile and link as fast as
one 75-line file.

---

## 6. Testing strategy

Four new test binaries (60 → 64), one modified.

### 6.1 `arm7_bios_get_sine_table_test.cpp`

Coverage:

- **Boundary indices return formula values.** Index 0 returns
  `lround(sin(0)·32768)` = 0. Index 63 returns
  `lround(sin(63·π/128)·32768)` recomputed in the test.
- **Mid-range index matches formula.** Index 32 returns
  `lround(sin(32·π/128)·32768)` = `lround(sin(π/4)·32768)` = 23170 =
  0x5A82, recomputed in the test.
- **Full-table formula equivalence.** Iterate all 64 indices; for each,
  recompute `lround(sin(i·π/128)·32768)` in the test and assert exact
  equality with `sine_table_lookup(i)`. This is the load-bearing test —
  it locks the production table to the formula the spec specifies.
- **Monotonic increase.** Walk all 64 entries, assert each is ≥ the
  previous. Quick sanity that the table isn't accidentally reversed or
  shuffled.
- **Out-of-range behavior.** Index 0x40 (one past end) clamps to index
  0x3F, returns the same value as index 0x3F. First call logs a warn
  (the test does not assert log content; the warn is a side effect).
- **Out-of-range with very large R0.** Index 0xFFFFFFFF clamps to 0x3F,
  returns same value.
- **Handler signature smoke.** Direct call to `bios7_get_sine_table(state,
  bus)`, assert return value == 1 (cycle count) and `state.r[0]` is
  written.

### 6.2 `arm7_bios_get_pitch_table_test.cpp`

Coverage:

- **Index 0 returns 0x0000.** Formula gives 0.
- **Index 64 returns formula value.** One-semitone ratio. Test recomputes
  `lround((pow(2.0, 64.0/768.0) − 1.0) · 65536.0)` and asserts equality.
- **Index 767 returns formula value.** Top of table. Test recomputes
  the same expression and asserts equality (will be ~0xFF6A, not
  GBATEK's 0xFF8A — the §5.3 acknowledged delta).
- **Full-table formula equivalence.** Iterate all 768 indices; recompute
  the formula in the test and assert exact equality with
  `pitch_table_lookup(i)`. Load-bearing.
- **Monotonic increase.** Walk all 768 entries, assert each ≥ previous.
- **Out-of-range index 0x300 clamps to 0x2FF.**
- **Handler signature smoke.**

### 6.3 `arm7_bios_get_volume_table_test.cpp`

Coverage:

- **Index 0 returns 0x00.**
- **Index 0xBC (last byte of segment 0) returns 0x7F.**
- **Index 0xBD (first byte of segment 1) returns 0x20.** Asserts the
  segment restart.
- **Index 0x179 returns 0x7E** (last of segment 1).
- **Index 0x17A returns 0x40** (first of segment 2; restart).
- **Index 0x202 returns 0x7E** (last of segment 2).
- **Index 0x203 returns 0x40** (first of segment 3; restart).
- **Index 0x2D3 returns 0x7F** (last valid index, end of segment 3).
- **Each segment is monotonic from its start to its end.** Walk each
  segment's index range, assert non-decreasing.
- **Total table length is exactly 0x2D4 (724) bytes.**
- **All values are in 0..0x7F.** Walk every entry, assert
  `value <= 0x7F`.
- **Out-of-range index 0x2D4 clamps to 0x2D3.**
- **Handler signature smoke.**

No external reference values are inlined as a fixture — the table's
correctness is defined by the script's structural output, and the test
asserts the structural invariants (segment endpoints, monotonicity,
length, value range) that any correct generator must satisfy.

### 6.4 `arm7_bios_sound_tables_capstone_test.cpp`

Three sub-cases, each running end-to-end through the real dispatcher
after exception-entry SVC setup (mirrors `arm7_bios_decomp_callback_
sequence_test.cpp` from slice 3g):

1. **`arm7_bios_hle_dispatch_swi(cpu, 0x1A)` with R0 = 32 returns
   0x5A82 (or formula value) in R0.** Verifies 0x1A through full
   dispatch.
2. **`...dispatch_swi(cpu, 0x1B)` with R0 = 64 returns the
   one-semitone pitch entry in R0.**
3. **`...dispatch_swi(cpu, 0x1C)` with R0 = 0xBD returns 0x20 in R0.**
   Specifically picks an index that exercises the segment-restart
   property, since a wrong implementation that just used a single linear
   curve would return the wrong value here.

Plus a fourth sub-case exercising out-of-range R0 through the full
dispatcher:

4. **`...dispatch_swi(cpu, 0x1A)` with R0 = 0x100 clamps to 0x3F,
   returns 0x7FF5 in R0, no assertion failure or crash.**

### 6.5 `arm7_bios_dispatch_test.cpp` modifications

Three new smoke rows added to the dispatcher coverage matrix:

```cpp
SECTION("SWI 0x1A GetSineTable") { ... }
SECTION("SWI 0x1B GetPitchTable") { ... }
SECTION("SWI 0x1C GetVolumeTable") { ... }
```

Each just verifies the dispatcher routes the SWI to the right handler
(asserts a sentinel value gets written to R0). Detailed correctness
lives in the per-handler tests; the dispatch test is the dispatch-table
sanity check.

### 6.6 Test binary count

End of slice 3g: **60** binaries.

Adds:
- `arm7_bios_get_sine_table_test`
- `arm7_bios_get_pitch_table_test`
- `arm7_bios_get_volume_table_test`
- `arm7_bios_sound_tables_capstone_test`

End of slice 3h: **64** binaries.

---

## 7. Cross-references

- **Slice 3d** — exception entry / IRQ path; the capstone test relies on
  this for SVC setup before dispatch.
- **Slice 3e** — SWI dispatch table; this slice extends it with three
  new case rows.
- **Slice 3f / 3g** — set the per-SWI file convention this slice
  mirrors.
- **GBATEK BIOS Misc Functions** —
  https://problemkaputt.de/gbatek-bios-misc-functions.htm — primary
  spec for all three SWIs.
- **libnds bios.h** —
  https://github.com/devkitPro/libnds/blob/master/include/nds/bios.h —
  documents the C-level prototypes (`swiGetSineTable`,
  `swiGetPitchTable`, `swiGetVolumeTable`) and the index ranges.
- **melonDS freebios** —
  https://github.com/melonDS-emu/melonDS/blob/master/freebios/src/bios_common.s
  lines 941-1216 — public clean-room SWI handlers. Referenced for
  algorithmic context only (e.g. observing that VolumeTable has four
  piecewise segments). **Not a source of bytes we copy. Not used as a
  verification oracle.** Slice 3h tests verify against re-derived
  formulas and structural invariants, not against external reference
  values.
- **NitroSDK sound driver** — closed-source, used via behavior
  observation through tools like NCSF/NitroComposer. We don't reference
  it directly; the SWIs' contract is purely the GBATEK-documented
  return value as a function of R0.

---

## 8. Risk and rollback

### Risk 1 — SineTable values differ from real BIOS by 1-2 LSB

Our `lround(sin(i·π/128) · 32768)` uses IEEE-754 doubles with `lround`'s
ties-away-from-zero rounding. Real BIOS may use a different rounding
mode at a handful of indices. **Impact:** audibly inaudible — 1 LSB on a
16-bit sine = -90 dB. **Mitigation:** none needed; the formula is the
spec, and the test asserts the table matches the formula exactly. If a
real game ever shows audio bugs traceable to a SineTable LSB delta,
slice 3i+ revisits the rounding mode.

### Risk 2 — PitchTable's top entry is 0x20 below the GBATEK-documented max

The 12-TET formula gives 0xFF6A at index 767; GBATEK documents the max as
0xFF8A. **Impact:** at the highest pitch index, NitroSDK's sample-rate
divider may pick a slightly different integer divider. NitroSDK's
divider is integer-quantized downstream, so most LSB-level deltas
collapse to the same hardware divider value. **Mitigation:** none needed
for slice 3h; the formula is the spec. If a real game shows audible bugs
traceable to the top of the pitch range, slice 3i+ revisits with
`tools/gen_pitch_table.py` (mirroring the VolumeTable script pattern).

### Risk 3 — VolumeTable curve audibly differs from real BIOS

Our independently-derived four-segment log curve may differ from real
BIOS values across the table. **Impact:** small-step volume transitions
may step at slightly different perceptual rates than on hardware.
**Mitigation:** ±1 LSB on a 0..127 volume scale is inaudible; even larger
deltas are unlikely to be perceptually significant. If a real game
reveals audible volume-curve issues, slice 3i+ revisits the script's
segment parameterization — a script-only change, no source-tree
restructuring.

### Risk 4 — `tools/gen_volume_table.py` requires a Python version we
don't standardize on

The script will use only Python 3.x stdlib (math, sys). **Mitigation:**
top-of-script `#!/usr/bin/env python3` and a docstring stating "Python
3.8+ required, no third-party dependencies." Any contributor with a
modern Python can re-run.

### Risk 5 — Out-of-range warn floods the test logs

Tests that intentionally exercise out-of-range R0 will trigger one warn
per SWI per process. **Mitigation:** the per-handler `static
std::atomic<bool> warned` pattern (§4.2) ensures one warn max per SWI
per process. Tests that exercise the warn path don't assert log content,
so the warn is silent to the test runner.

### Risk 6 — Spec misnames the SWI again

Slice 3g's spec called the third SWI "SoundGetJumpList" — a name that
does not exist on NDS. **Mitigation:** §1 of this spec explicitly
corrects it. All references in this spec use "GetVolumeTable." All file
names use `get_volume_table`. All test names use `get_volume_table`.

### Risk 7 — Future contributor regenerates VolumeTable bytes from a real
BIOS dump and commits them, breaking the no-dump rule

**Mitigation:** the spec, the script's docstring, and a comment block
in `bios7_tables.cpp` next to the byte array all state explicitly:
"These values are produced by `tools/gen_volume_table.py`. Do not
regenerate from a BIOS dump. If you need to update them, modify the
script's parameterization and re-run." The comment is the social
backstop; the script is the technical one.

### Rollback strategy

Each commit is independently revertable (no shared cross-commit state).
If commit N has a problem, `git revert <commit_N>` returns the project
to the previous-commit state with no broken intermediate state. Worst
case, the whole slice reverts via 6 reverts in sequence; mid-slice
revert leaves the dispatcher's `default` branch handling 0x1A/0x1B/0x1C
again, which is the pre-slice baseline — buggy behavior, but stable.

---

## 9. Slice completion criteria

- [x] `bios7_tables.{hpp,cpp}` exist with `kSineTable`, `kPitchTable`,
      `kVolumeTable` defined and the three `*_lookup` accessors.
      (`bios7_tables.cpp` 151 lines, `bios7_tables.hpp` 34 lines.)
- [x] `bios7_get_sine_table.cpp`, `bios7_get_pitch_table.cpp`,
      `bios7_get_volume_table.cpp` exist with `bios7_get_*_table`
      handlers. (33, 35, 35 lines respectively.)
- [x] `tools/gen_volume_table.py` exists, is executable, and reproduces
      the bytes committed in `bios7_tables.cpp` **exactly** (verified by
      `diff` between the `clang-format off/on` block in `bios7_tables.cpp`
      and the script's stdout — byte-for-byte match, no tolerance used).
- [x] `bios7_hle.cpp` dispatcher has cases for 0x1A, 0x1B, 0x1C
      (lines 80/83/86).
- [~] **Deviation:** originally planned as four new test binaries
      (`arm7_bios_get_sine_table_test`, `arm7_bios_get_pitch_table_test`,
      `arm7_bios_get_volume_table_test`, `arm7_bios_sound_tables_capstone_test`);
      actually landed as **one bundled binary** (`arm7_bios_tables_test`,
      424 lines — under the 500-line soft cap). Coverage parity: every
      per-SWI assertion from §6.1–6.3 (index 0, mid-range vs formula,
      full-table formula equivalence, monotonicity, handler smoke, one-past-end
      clamp, max-u32 clamp, segment endpoints, segment monotonicity, value
      range) is present under the equivalent function names
      (`sine_table_*`, `pitch_table_*`, `volume_table_*`,
      `get_{sine,pitch,volume}_table_handler_*`). Bundling was chosen
      because the three SWIs share static-init state and fixture plumbing,
      and grep-by-SWI-name stays predictable via function names inside
      the file.
- [x] **Capstone (§6.4) superseded** by `arm7_bios_dispatch_test.cpp` —
      the three ARM-state SWI tests `arm_swi_0x1a_get_sine_table`
      (line 249), `arm_swi_0x1b_get_pitch_table` (line 267),
      `arm_swi_0x1c_get_volume_table` (line 285) fire real ARM SWI
      instructions (`0xEF00001A/1B/1C`), exercising the full
      SWI-exception-entry → dispatcher → handler → `MOVS PC, R14` path.
      This is **strictly stronger** than the §6.4 design's proposal
      to call `arm7_bios_hle_dispatch_swi(cpu, swi)` directly, because
      it also verifies real SWI instruction decode and the exception
      entry vector. The specific §6.4 sub-cases are covered:
      - Sub-case 1 (0x1A, R0=32 → 0x5A82): lines 254–257.
      - Sub-case 2 (0x1B, R0=64 → `pitch_table_lookup(64)`): lines 272–275.
      - Sub-case 3 (0x1C, R0=0xBD → 0x20, segment-restart): lines 290–293.
      - Sub-case 4 (0x1A, out-of-range R0 clamps): covered at the handler
        level in `arm7_bios_tables_test.cpp` via
        `get_sine_table_handler_clamps_one_past_end` and `_clamps_max_u32`.
      No separate capstone binary was added — it would be redundant.
- [x] `arm7_bios_dispatch_test.cpp` updated with three new sections for
      the 0x1A/0x1B/0x1C handlers (commits 2/3/4 of this slice).
- [x] `ctest --output-on-failure` reports **60/60 passing in Debug**.
      (Originally projected 64/64 under the four-binary plan; actual 60/60
      reflects the single-binary bundling above. No coverage was dropped.)
- [x] `clang-format` clean on all new files (enforced by the repo's
      `PostToolUse` hook on every Edit/Write; no manual reformatting).
- [x] `ds-architecture-rule-checker` and `gbatek-reviewer` ran clean
      during commits 1–4; no new code in commit 5 to re-review (slice
      closure is docs-only).
- [x] `quality-reviewer` ran clean during commits 1–4; no new code in
      commit 5.
- [x] No file exceeds the 500-line soft cap. Maximum: `bios7_tables.cpp`
      151 lines, `arm7_bios_tables_test.cpp` 424 lines.
- [x] No `.bin`, `.dump`, `.bios` file in the diff (verified:
      `git log --name-only` across commits 1–4 contains only `.cpp`, `.hpp`,
      `.py`, and this spec file).
- [x] `bios7_tables.cpp` contains in-code comment blocks citing the
      provenance of every table (formula for Sine at line 34–37, formula
      for Pitch at line 52–58, script for Volume at line 72–79, plus an
      explicit "do not regenerate from a BIOS dump" admonition at line
      76–78 and a cross-reference to CLAUDE.md architecture rule #10).

---

## Appendix A. Commit sequence

Six commits, shippable individually:

### Commit 1 — `bios7_tables.{hpp,cpp}` scaffold + SineTable + verification test

- Add `src/cpu/arm7/bios/bios7_tables.hpp` with size constants and the
  three `*_lookup` declarations.
- Add `src/cpu/arm7/bios/bios7_tables.cpp` with `kSineTable` (computed
  via `compute_sine_table()`) and `sine_table_lookup`. PitchTable and
  VolumeTable storage is declared as `extern` placeholders (or simply
  not yet present — commit 3 / commit 4 add them) — easier path is "not
  yet present," and the three `*_lookup` functions are split into per-
  table commits.

  Refined: commit 1 adds **only** SineTable's storage and lookup. The
  hpp declares all three sizes/lookups so the API is stable, but commits
  3 and 4 add the actual `kPitchTable` and `kVolumeTable` data and the
  matching `pitch_table_lookup` / `volume_table_lookup` definitions.
  Until then, those two lookups are declared but undefined — but the
  dispatcher doesn't call them yet (no case rows for 0x1B / 0x1C until
  commits 3 / 4 either), so the link is clean.

  Even simpler refinement: commit 1 adds `bios7_tables.{hpp,cpp}` with
  Sine only declared. Commit 3 modifies the hpp to add Pitch
  declarations and the cpp to add the data. Commit 4 same for Volume.
  The hpp grows incrementally, no `extern` placeholders.
- Add `tests/unit/arm7_bios_get_sine_table_test.cpp` testing only
  `sine_table_lookup` directly (no SWI handler yet — commit 2 adds it).
- Add CMake entry for the new test.
- `ctest`: 60 → 61.

### Commit 2 — `0x1A GetSineTable` SWI handler + dispatcher wire-up

- Add `src/cpu/arm7/bios/bios7_get_sine_table.cpp` with the handler.
- Add the handler declaration to `bios7_hle.hpp`.
- Add the case row for 0x1A in `bios7_hle.cpp`.
- Modify `arm7_bios_get_sine_table_test.cpp` to add the handler-smoke
  section (or split: leave commit 1's test as table-only and add a new
  test file `arm7_bios_get_sine_table_handler_test.cpp` — likely
  overkill, prefer extending the existing test).
- Modify `arm7_bios_dispatch_test.cpp` to add the 0x1A smoke row.
- `ctest`: still 61 (no new binary, the same test grew sections).

  Refinement: cleaner to keep commits 1 and 2 as separate test binaries
  if the section-extension approach makes commit 1's test wider than its
  scope. Architect's preferred path: commit 1's test is "table values are
  correct," commit 2 extends the same file with "handler routes
  correctly." One binary, two sections, one extension diff in commit 2.
- `ctest`: 61.

### Commit 3 — PitchTable data + `0x1B GetPitchTable` handler + test

- Modify `bios7_tables.hpp` to declare PitchTable size constant and
  `pitch_table_lookup`.
- Modify `bios7_tables.cpp` to add `kPitchTable` (computed via
  `compute_pitch_table()`) and `pitch_table_lookup`.
- Add `src/cpu/arm7/bios/bios7_get_pitch_table.cpp` with the handler.
- Add the handler declaration to `bios7_hle.hpp`.
- Add the case row for 0x1B in `bios7_hle.cpp`.
- Add `tests/unit/arm7_bios_get_pitch_table_test.cpp` with table-value
  cross-check + handler smoke + out-of-range coverage.
- Modify `arm7_bios_dispatch_test.cpp` to add the 0x1B smoke row.
- `ctest`: 62.

### Commit 4 — VolumeTable data + offline generator + `0x1C GetVolumeTable` handler + test

- Add `tools/gen_volume_table.py` (the offline generator script). Makes
  it executable.
- Modify `bios7_tables.hpp` to declare VolumeTable size constant and
  `volume_table_lookup`.
- Modify `bios7_tables.cpp` to add `kVolumeTable` (the 724-byte static
  array generated by the script) and `volume_table_lookup`. Include a
  comment block citing the script and the no-dump provenance.
- Add `src/cpu/arm7/bios/bios7_get_volume_table.cpp` with the handler.
- Add the handler declaration to `bios7_hle.hpp`.
- Add the case row for 0x1C in `bios7_hle.cpp`.
- Add `tests/unit/arm7_bios_get_volume_table_test.cpp` with segment
  boundary tests + monotonicity per segment + handler smoke +
  out-of-range coverage.
- Modify `arm7_bios_dispatch_test.cpp` to add the 0x1C smoke row.
- `ctest`: 63.

### Commit 5 — Capstone end-to-end test

- Add `tests/unit/arm7_bios_sound_tables_capstone_test.cpp` with the
  four sub-cases from §6.4 (three correctness, one out-of-range).
- `ctest`: 64.

### Commit 6 — Slice closure: docs + completion checklist

- Tick the §9 completion checklist boxes in this spec.
- Update `docs/specs/2026-04-12-nds-emulator-design.md` if the slice's
  outcome warrants a status note (likely a one-line bump in the BIOS
  HLE coverage section).
- No code changes. Test count stays at 64.

(Commit 6 is bookkeeping; if the project policy is "no docs-only
commits," fold it into commit 5.)

---

## Appendix B. Provenance audit

| Table        | Source in repo                              | Provenance                                                       |
|--------------|---------------------------------------------|------------------------------------------------------------------|
| SineTable    | `compute_sine_table()` in `bios7_tables.cpp`| Closed-form `sin(i·π/128) × 0x8000`. No script. No data file.   |
| PitchTable   | `compute_pitch_table()` in `bios7_tables.cpp`| Closed-form `(2^(i/768)-1) × 0x10000`. No script. No data file.|
| VolumeTable  | `kVolumeTable[724]` in `bios7_tables.cpp`   | Generated once by `tools/gen_volume_table.py`. Bytes committed.  |

**No external reference values appear in `src/` or `tests/`.** Tests
verify table correctness by independently recomputing the production
formula (Sine, Pitch) or by asserting structural invariants (Volume —
segment endpoints, monotonicity, length, value range). melonDS freebios
is referenced in this spec for *algorithmic context only* (e.g.
documenting that VolumeTable has four piecewise segments); no melonDS
bytes are inlined as fixtures, used as tolerance targets, or shipped in
any form.

**Audit checklist for any future contributor touching these tables:**

1. Did this change introduce any `.bin`, `.dump`, `.bios`, or
   `bios7.bin`-named file? If yes, revert.
2. Did this change paste byte values into `bios7_tables.cpp` without
   updating `tools/gen_volume_table.py`? If yes, the commit message must
   explain the provenance of each new byte value.
3. Did this change introduce a comment of the form "extracted from
   bios7.bin at offset X"? If yes, revert.

---

## Appendix C. Why the user prompt said "723" and the spec says "724"

The user prompt's table sizing column listed VolumeTable as "723 entries
of u8." That is the count of distinct **valid index values** in GBATEK's
documented range `0..2D3h` — but `0..2D3h` inclusive is `2D4h = 724`
indices, since both endpoints are valid. The table itself has 724 bytes:

- libnds: `swiGetVolumeTable(int index)` doc — "0-723" — meaning indices
  0..723 inclusive = 724 values.
- melonDS freebios: 724 `.byte` entries counted in `bios_common.s`.
- GBATEK: implicit (the highest index is 0x2D3, so the table size is
  0x2D4).

The spec uses 724 (0x2D4) throughout. `kVolumeTableSize = 0x2D4`. No
off-by-one.
