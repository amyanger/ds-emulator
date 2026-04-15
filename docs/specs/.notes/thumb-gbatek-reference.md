# Thumb Instruction Set — GBATEK reference condensed for slice 3c

Source: `mgba-emu/gbatek` index.md, lines 120175–120825 (THUMB Binary Opcode
Format through THUMB.17). Scraped 2026-04-15.

**Scope:** ARMv4T (ARM7TDMI). ARMv5-only forms (BKPT, Thumb BLX label, Thumb
BLX Rs / BLX register with MSBd=1) are called out but **not implemented in
slice 3c** — those land in the ARM9 delta slice.

---

## Top-level format table (bits 15..0)

```
 Form|_15|_14|_13|_12|_11|_10|_9_|_8_|_7_|_6_|_5_|_4_|_3_|_2_|_1_|_0_|
 __1_|_0___0___0_|__Op___|_______Offset______|____Rs_____|____Rd_____|Shifted
 __2_|_0___0___0___1___1_|_I,_Op_|___Rn/nn___|____Rs_____|____Rd_____|ADD/SUB
 __3_|_0___0___1_|__Op___|____Rd_____|_____________Offset____________|Immed
 __4_|_0___1___0___0___0___0_|______Op_______|____Rs_____|____Rd_____|AluOp
 __5_|_0___1___0___0___0___1_|__Op___|Hd_|Hs_|____Rs_____|____Rd_____|HiReg/BX
 __6_|_0___1___0___0___1_|____Rd_____|_____________Word______________|LDR PC
 __7_|_0___1___0___1_|__Op___|_0_|___Ro______|____Rb_____|____Rd_____|LDR/STR
 __8_|_0___1___0___1_|__Op___|_1_|___Ro______|____Rb_____|____Rd_____|H/SB/SH
 __9_|_0___1___1_|__Op___|_______Offset______|____Rb_____|____Rd_____|imm {B}
 _10_|_1___0___0___0_|Op_|_______Offset______|____Rb_____|____Rd_____|imm H
 _11_|_1___0___0___1_|Op_|____Rd_____|_____________Word______________|SP rel
 _12_|_1___0___1___0_|Op_|____Rd_____|_____________Word______________|ADD PC/SP
 _13_|_1___0___1___1___0___0___0___0_|_S_|___________Word____________|ADD SP,nn
 _14_|_1___0___1___1_|Op_|_1___0_|_R_|____________Rlist______________|PUSH/POP
 _15_|_1___1___0___0_|Op_|____Rb_____|____________Rlist______________|STM/LDM
 _16_|_1___1___0___1_|_____Cond______|_________Signed_Offset_________|B{cond}
 _17_|_1___1___0___1___1___1___1___1_|___________User_Data___________|SWI
 _18_|_1___1___1___0___0_|________________Offset_____________________|B
 _19a|_1___1___1___1___0_|_________________________var___________|___|BL hi
 _19b|_1___1___1___1___1_|_________________________var___________|___|BL lo
```

Discriminator bits (15..13) give you 8 top-level buckets. Within each bucket,
bits 12..10 (or narrower ranges) select the specific format.

---

## Per-format details

### THUMB.1 — move shifted register

- Encoding: `000 op2 off5 Rs3 Rd3`
- Ops (bits 12..11): 00=LSL, 01=LSR, 10=ASR, 11=reserved (→ THUMB.2)
- Offset = imm5 (0..31). Special case: `LSR/ASR #0` encodes shift amount 32.
  `LSL #0` means "no shift, C unchanged."
- Flags: **N, Z, C** updated. V unchanged. C unchanged for `LSL #0`.
- Timing: 1S.
- **Delegates to:** existing ARM-state barrel shifter (`shift_imm` /
  `compute_shifter_operand_immediate_shift`) — same shift-amount semantics.

### THUMB.2 — add/subtract (register or 3-bit immediate)

- Encoding: `00011 I op Rn3/imm3 Rs3 Rd3`
- Ops (bits 10..9): 0=ADDreg, 1=SUBreg, 2=ADDimm3, 3=SUBimm3
- Imm3 = 0..7. With op=2 and imm=0, assembler emits `MOV Rd,Rs` pseudo.
- Flags: **N, Z, C, V** updated (full NZCV, even for imm=0 MOV pseudo).
- Timing: 1S.
- **Delegates to:** ARM-state `exec_add` / `exec_sub` helpers after building
  operand2.

### THUMB.3 — move/compare/add/subtract immediate (8-bit)

- Encoding: `001 op2 Rd3 imm8`
- Ops (12..11): 0=MOV, 1=CMP, 2=ADD, 3=SUB
- Imm8 = 0..255
- Flags: MOV updates **N, Z only**. CMP/ADD/SUB update full **N, Z, C, V**.
- Timing: 1S.
- **Delegates to:** ARM helpers, but note MOV's NZ-only flag update is
  different from Thumb add/sub/cmp — must pass the "only NZ" variant.

### THUMB.4 — ALU operations (register/register)

- Encoding: `010000 op4 Rs3 Rd3`
- Sixteen ops, opcodes 0..F: AND, EOR, LSL, LSR, ASR, ADC, SBC, ROR, TST, NEG,
  CMP, CMN, ORR, MUL, BIC, MVN.
- **NEG** = `Rd = 0 - Rs` (ARM equivalent: RSB Rd,Rs,#0).
- Shift ops (LSL/LSR/ASR/ROR) use `Rs & 0xFF` as shift amount — same "register
  shift" path as ARM reg-shifted operands.
- Flags:
  - ADC/SBC/NEG/CMP/CMN: N, Z, C, V
  - LSL/LSR/ASR/ROR: N, Z, C (C unchanged if shift amount == 0)
  - **MUL on ARMv4: N, Z set, C destroyed** (unpredictable/trashed). V
    unchanged. This is the ARM7 behavior.
  - AND/EOR/TST/ORR/BIC/MVN: N, Z only
- Timing: 1S for most; 1S+1I for shifts; **1S+mI for MUL (m=1..4 on ARMv4
  depending on MSBs of Rd)**.
- **Delegates to:** ARM-state data-processing helpers — Thumb.4 is a direct
  mirror of a subset of `dispatch_dp` (Rd,Rs → Rd,Rd,Rs via Rn=Rd).

### THUMB.5 — Hi register operations / BX

- Encoding: `010001 op2 Hd Hs Rs3 Rd3`
- Ops (bits 9..8): 0=ADD, 1=CMP, 2=MOV (or NOP = `MOV R8,R8`), 3=BX (ARMv5: BLX)
- Full register number: `Rs_full = (Hs<<3)|Rs`, `Rd_full = (Hd<<3)|Rd`.
- **Restrictions:**
  - ADD/CMP/MOV: **Hd or Hs must be set** (not both zero). Zero/zero is
    UNPREDICTABLE on ARMv4. Old GBA assemblers use `MOV R8,R8` encoding
    (Hd=1, Hs=1, Rs=Rd=0) as NOP.
  - BX: Hd=0 (Rd unused). MSBs (Hs) is the top bit of `Rs`.
  - BLX: Hd=1 (on ARMv4, this encoding is UNDEFINED — ARM9 only).
- **Flag semantics (critical):** ADD/MOV do **not** touch CPSR. Only CMP does.
  That's the opposite of THUMB.1–THUMB.4 and is a common bug source.
- **BX rules:**
  - `PC = Rs & ~1`
  - `CPSR.T = Rs & 1`
  - If `Rs & 1 == 0`: switch to ARM state. Bit 1 of Rs must be 0 (word
    align). `BX PC` issued from a word-aligned address is the standard
    ARM↔Thumb switch idiom; destination is PC+4 (the next halfword is
    skipped because PC is read as `instr_addr + 4` in Thumb).
- **PC-as-source rule:** When Rs or Rd is R15, its read value is
  `(instr_addr + 4) & ~2` for ADD/MOV/CMP — same word-align-down as THUMB.6.
  (For BX R15 the same rule applies.)
- Timing: 1S for ADD/MOV/CMP, 2S+1N for ADD/MOV with Rd=R15 and for BX.
- **Delegates to:** the `dispatch_bx` path for BX Rs; a minimal ADD/MOV/CMP
  hi-register helper for the other three.

### THUMB.6 — PC-relative LDR (literal pool)

- Encoding: `01001 Rd3 imm8`
- Op: `LDR Rd, [PC, #imm8*4]` — 32-bit load only
- **PC value used:** `(instr_addr + 4) & ~2` (force-align to word).
- Effective address: `pc_aligned + (imm8 << 2)`. Always word-aligned, so no
  rotate-on-misalign.
- Timing: 1S+1N+1I.
- **Delegates to:** existing `bus.read32` path via `arm7_load_store`-style
  helper. No rotate needed.

### THUMB.7 — load/store with register offset (word/byte)

- Encoding: `0101 op2 0 Ro3 Rb3 Rd3`
- Ops (11..10): 0=STR, 1=STRB, 2=LDR, 3=LDRB
- Address: `Rb + Ro`
- **LDR unaligned rotate:** yes — same `(addr & 3) * 8` rotate-right as ARM
  state for misaligned word reads. The slice 3b2 `rotate_read_word` helper
  handles this.
- Timing: LDR 1S+1N+1I, STR 2N.
- **Delegates to:** existing ARM-state load/store path.

### THUMB.8 — load/store sign-extended byte/halfword

- Encoding: `0101 op2 1 Ro3 Rb3 Rd3`
- Ops (11..10): 0=STRH, 1=LDSB, 2=LDRH, 3=LDSH
- Address: `Rb + Ro`
- **Alignment rules (match ARM-state halfword ops from slice 3b3):**
  - LDRH: if `addr & 1`, reads from `addr & ~1`, then rotates right by 8.
    melonDS just masks `addr[0]` and zero-extends — we match.
  - LDSH: if `addr & 1`, sign-extends the byte at `addr & ~1` + 1 (i.e.
    behaves like LDSB with shifted base). Slice 3b3 standardized on masking
    `addr[0]` and sign-extending the halfword — we match.
  - STRH: masks `addr[0]` before the write.
- Timing: LDR variants 1S+1N+1I, STR 2N.
- **Delegates to:** ARM-state halfword helpers from slice 3b3.

### THUMB.9 — load/store with immediate offset

- Encoding: `011 op2 imm5 Rb3 Rd3`
- Ops (12..11): 0=STR, 1=LDR, 2=STRB, 3=LDRB
- Offset scaling:
  - Word forms (STR/LDR): `imm5 * 4` → 0..124
  - Byte forms (STRB/LDRB): `imm5 * 1` → 0..31
- Same unaligned-LDR rotate as THUMB.7.
- **Delegates to:** ARM-state load/store path.

### THUMB.10 — load/store halfword (immediate offset)

- Encoding: `1000 op1 imm5 Rb3 Rd3`
- Ops: 0=STRH, 1=LDRH
- Offset: `imm5 * 2` → 0..62
- Alignment: same as THUMB.8 (mask bit 0; LDRH rotates; STRH masks).
- **Delegates to:** ARM-state halfword helpers.

### THUMB.11 — load/store SP-relative

- Encoding: `1001 op1 Rd3 imm8`
- Ops: 0=STR, 1=LDR
- Address: `SP + (imm8 << 2)`. Always 32-bit.
- Rotate-on-misalign still applies (game code should always align SP but the
  CPU doesn't enforce it).
- **Delegates to:** ARM-state word load/store.

### THUMB.12 — get relative address (ADD PC/SP)

- Encoding: `1010 op1 Rd3 imm8`
- Ops: 0=ADD Rd, PC, #imm8*4 — `Rd = ((instr_addr + 4) & ~2) + (imm8 << 2)`
- Ops: 1=ADD Rd, SP, #imm8*4 — `Rd = SP + (imm8 << 2)`
- No flags.
- Timing: 1S.
- **New helper:** `exec_thumb_rel_addr(imm8, use_sp)` — tiny, no delegation
  needed.

### THUMB.13 — add offset to stack pointer

- Encoding: `10110000 S imm7`
- S: 0 = `SP += imm7*4`, 1 = `SP -= imm7*4`
- Offset: 0..508 in steps of 4.
- No flags.
- Timing: 1S.
- **New helper:** `exec_thumb_add_sp(imm7, sign)`. Two-line.

### THUMB.14 — push/pop with optional LR/PC

- Encoding: `1011 op1 10 R Rlist8`
- op: 0=PUSH, 1=POP
- R bit:
  - PUSH: R=1 means "include LR in the push" (LR pushed last = highest addr).
  - POP: R=1 means "include PC in the pop" (PC popped last = highest addr).
- Rlist = R0..R7 only (8-bit list). LR/PC is enabled separately by R.
- Addressing:
  - PUSH = STMDB SP! — full-descending store. Equivalent ARM-state: `STMFD
    SP!, {Rlist, LR?}`.
  - POP = LDMIA SP! — equivalent ARM-state: `LDMFD SP!, {Rlist, PC?}`.
- **POP PC behavior on ARM7 (ARMv4):** ignores bit 0 of loaded value — CPU
  stays in Thumb state regardless. (ARMv5/ARM9 uses bit 0 to set T flag; not
  our concern in this slice.)
- **Delegates to:** the slice-3b4 LDM/STM core. Build a synthetic `reg_list`
  (Rlist with LR bit 14 or PC bit 15 added), set P/U/W/L according to
  PUSH/POP semantics (U=0/P=1/W=1 for PUSH; U=1/P=0/W=1 for POP), and call
  into the existing block-transfer walker.

### THUMB.15 — multiple load/store (LDMIA/STMIA without LR/PC)

- Encoding: `1100 op1 Rb3 Rlist8`
- op: 0=STMIA Rb!, 1=LDMIA Rb!
- Base always writes back (no separate W bit — it's implicit).
- Lowest register in list → lowest address.
- **Invalid Rlist edge cases (ARMv4):**
  - **Empty Rlist:** R15 is loaded/stored (transferred as if bit 15 were set),
    and Rb is incremented by 0x40. This matches the ARM-state quirk and we
    route it through the existing empty-list path with a forced R15 flag.
  - **Writeback with Rb in Rlist:**
    - STM: if Rb is the FIRST (lowest) entry, store OLD base. Otherwise store
      NEW base. (Matches ARM-state STM rule from slice 3b4.)
    - LDM: **no writeback** when Rb is in the list. Note that this is the
      same as the ARMv4 ARM-state Thumb convention — matches what slice 3b4
      implemented.
- **Delegates to:** slice-3b4 block-transfer core with P=0, U=1, W=1, S=0.

### THUMB.16 — conditional branch

- Encoding: `1101 cond4 imm8`
- cond: standard ARM condition codes 0x0..0xD, plus:
  - 0xE: undefined (should not be used — slice treats as UNDEF warn stub)
  - 0xF: SWI (→ THUMB.17, same top bits)
- Offset: `(int8_t)imm8 * 2`, applied to `instr_addr + 4`.
- Timing: 2S+1N if taken, 1S if not.
- **Delegates to:** existing condition-evaluation helper; branch target is
  `instr_addr + 4 + (sext(imm8) << 1)`.

### THUMB.17 — software interrupt and breakpoint

- Encoding SWI: `11011111 imm8`
- Encoding BKPT: `10111110 imm8` (**ARMv5 only** — ARM9's slice, not 3c)
- SWI semantics: enter SVC mode, ARM state, IRQs disabled, PC = 0x00000008
  (vector). `R14_svc = instr_addr + 2`, `SPSR_svc = CPSR`. CPSR.T cleared
  (ARM state entry).
- Timing: 2S+1N.
- **Out of scope for slice 3c.** Slice 3d (exceptions) owns SWI and the
  mode-switch path. For slice 3c we decode it to a warn stub or a
  `dispatch_undef`-style handler that logs and returns — actual SWI dispatch
  lands in 3d.

### THUMB.18 — unconditional branch

- Encoding: `11100 imm11`
- Offset: `sext(imm11) << 1`, range ±2 KiB around PC+4.
- Timing: 2S+1N.
- Plain branch, same helper path as THUMB.16 minus the condition check.

### THUMB.19 — long branch with link (BL, two-halfword encoding)

- **Occupies 32 bits = two 16-bit Thumb opcodes.** Decoded as two separate
  instructions. Exceptions *may* occur between the two halves (GBATEK says
  "implementation defined" — we treat it as atomic from the CPU's pipeline
  perspective; no IRQ interleaving).
- **First halfword:** `11110 imm11_hi`
  - `LR = (instr_addr + 4) + (sext(imm11_hi) << 12)`
  - PC advances by 2 and the second halfword is fetched.
- **Second halfword:**
  - `11111 imm11_lo` → **BL**: `PC = LR + (imm11_lo << 1); LR = next_instr | 1`
  - `11101 imm11_lo` → **BLX** (ARMv5 only — ARM9)
- Target range: PC ± 4 MiB.
- On ARM7 the second halfword (BL, top 5 bits = 11111) is the only valid
  form. `11101` is UNDEF on ARMv4.
- **Pseudo usage:** games can issue only the second halfword as `BL LR+imm`
  (e.g. `F800h` = "BL LR+0"). Decoder must not assume the first halfword has
  always been seen — treat the second halfword as self-contained given the
  current LR.

---

## ARMv4 vs ARMv5 deltas (explicitly NOT in slice 3c)

- Thumb **BLX label** (second halfword bits 15..11 = `11101`) — ARM9 only.
- Thumb **BLX register** (THUMB.5 op=3 with MSBd=1) — ARM9 only.
- Thumb **BKPT** (THUMB.17 `10111110b` encoding) — ARM9 only.
- **POP {PC}** copying bit 0 into CPSR.T — ARM9 only. ARM7 stays in Thumb.
- **MUL in THUMB.4**: ARMv4 destroys C, takes 1S+mI (m=1..4). ARMv5 preserves
  C, takes 1S+3I. Slice 3c uses the ARMv4 behavior.

---

## Shared edge cases and pipeline notes

- **PC in Thumb state is `instr_addr + 4`** (not +8 like ARM state). Every
  place that reads R15 in slice 3c must use this offset.
- **LDR/ADD with PC literal pool** force-aligns PC: `(instr_addr + 4) & ~2`.
  This matters when the instruction itself sits at an address where bit 1
  is set (i.e. PC % 4 == 2).
- **Condition code field:** Thumb only has conditional execution in B{cond}
  (THUMB.16). Every other instruction is unconditional. No `Cond` prefix in
  the opcode.
- **Branch offsets are signed and pre-shifted**: THUMB.16 offset = `sext8
  << 1`, THUMB.18 offset = `sext11 << 1`, THUMB.19 first = `sext11 << 12`,
  THUMB.19 second (lo) = `imm11 << 1`.
- **Empty Rlist quirk applies to both THUMB.14 PUSH/POP and THUMB.15
  LDMIA/STMIA** — same ARMv4 convention (load/store R15, Rb += 0x40).

---

## Delegation map — what Thumb can reuse from slice 3b1–3b4

| Thumb format | Delegates to |
|---|---|
| 1 move-shifted | ARM-state shifter |
| 2 add/sub      | ARM-state add/sub helpers |
| 3 mov/cmp/add/sub imm | ARM-state dp helpers (with NZ-only variant for MOV) |
| 4 ALU reg/reg  | ARM-state dp helpers (Rn=Rd) |
| 5 Hi reg / BX  | `dispatch_bx` path; minimal ADD/MOV/CMP hi helper |
| 6 LDR PC rel   | ARM-state word load (pre-aligned address) |
| 7 LDR/STR reg offset  | ARM-state word/byte load/store |
| 8 LDRH/LDSB/LDRH/LDSH | slice 3b3 halfword helpers |
| 9 LDR/STR imm offset  | ARM-state word/byte load/store |
| 10 LDRH imm offset    | slice 3b3 halfword helpers |
| 11 LDR/STR SP rel     | ARM-state word load/store |
| 12 ADD PC/SP imm | tiny new helper |
| 13 ADD SP imm    | tiny new helper |
| 14 PUSH/POP      | slice 3b4 LDM/STM core |
| 15 LDMIA/STMIA   | slice 3b4 LDM/STM core |
| 16 Bcond         | existing condition + new branch path |
| 17 SWI           | stub → slice 3d |
| 18 B             | branch path (shared with 16) |
| 19 BL long       | new helper (stateful two-halfword) |

Total new code: roughly two new `.cpp` files (`arm7_thumb_decode.cpp` +
`arm7_thumb_ops.cpp`, splittable if needed) plus updates to `arm7.cpp`'s
dispatch and the test suite. Heavy reuse — most formats are 3–10 lines of
glue around existing helpers.
