// arm7_halfword.cpp
//
// ARM7TDMI halfword and signed single-data-transfer instructions:
//   LDRH  — unsigned halfword load, zero-extended to 32
//   STRH  — halfword store (low 16 bits of Rd)
//   LDRSB — signed byte load, sign-extended to 32
//   LDRSH — signed halfword load, sign-extended to 32
//
// Encoding: bits[27:25] == 000, bit[4] == 1, bit[7] == 1. Lives in the
// ARMv4T "halfword extension" corner of the data-processing encoding
// space. Dispatched from dispatch_dp() after the BX/PSR/multiply
// recognizers.
//
// LDRD/STRD are ARMv5TE+ only and not valid on ARM7 (GBATEK verbatim:
// "STRD/LDRD supported on ARMv5TE and up only, not ARMv5, not
// ARMv5TExP"). Their encoding slots (L=0, SH=2/3) log a warn on ARM7.

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7_decode_internal.hpp"
#include "cpu/arm7/arm7_state.hpp"
#include "ds/common.hpp"

namespace ds {

namespace {

// Decoded halfword transfer address with writeback metadata. Returned by
// compute_halfword_address() and consumed by the opcode handlers in
// tasks 4-7. `rn` is kept so write_rd_and_writeback() can suppress the
// writeback when Rn == Rd on a load (see §5.5 of the slice 3b3 design).
struct HalfwordAddress {
    u32  address;       // the address to access for the load/store
    u32  writeback_rn;  // the value Rn would receive if writeback is on
    u32  rn;            // the base register index (kept for Rn==Rd check)
    bool writeback;     // whether Rn actually updates
};

// Encode the P/U/W/I addressing math plus every GBATEK-documented
// malformed-encoding warn path. All four halfword opcodes share this;
// the variant-specific behavior lives in the transfer helpers below.
HalfwordAddress compute_halfword_address(const Arm7State& state, u32 instr);

// Transfer helpers — bodies land in tasks 4-7. Declared here so the
// follow-up tasks only need to fill in the bodies, not touch the
// anonymous-namespace declarations.
u32               load_halfword_unsigned(Arm7Bus& bus, u32 address);       // task 4
u32               load_halfword_signed(Arm7Bus& bus, u32 address);         // task 7
u32               load_byte_signed(Arm7Bus& bus, u32 address);             // task 6
void store_halfword(Arm7Bus& bus, u32 address, u32 value);                    // task 5

// Rn==Rd-safe Rd write with optional writeback. On a load where Rn==Rd,
// the loaded value must win — so we skip the writeback entirely when
// the two registers alias. Also handles the Rd==R15 UNPREDICTABLE warn
// path by logging and masking PC to word alignment (matching the
// existing write_rd() helper convention — ARMv4 leaves CPSR.T unchanged
// on PC writes, per GBATEK).
void write_rd_and_writeback(Arm7State& state, u32 rd, u32 value,
                            const HalfwordAddress& addr);

HalfwordAddress compute_halfword_address(const Arm7State& state, u32 instr) {
    const u32  rn = (instr >> 16) & 0xFu;
    const bool p  = (instr & (1u << 24)) != 0;  // pre/post-index
    const bool u  = (instr & (1u << 23)) != 0;  // up/down
    const bool i  = (instr & (1u << 22)) != 0;  // imm (1) vs reg (0) offset
    const bool w  = (instr & (1u << 21)) != 0;  // writeback (pre-index only)

    // Rn read follows the same convention as dispatch_single_data_transfer
    // in arm7_loadstore.cpp: state.r[15] already holds instr_addr+8 because
    // Arm7::step_arm() bakes the pipeline offset in before dispatch. No
    // extra +4/+8 needed — reading state.r[rn] directly yields PC+8 for
    // rn==15.
    const u32 rn_value = state.r[rn];

    u32 offset;
    if (i) {
        // Immediate offset: upper 4 bits at [11:8], lower 4 bits at [3:0].
        offset = ((instr >> 4) & 0xF0u) | (instr & 0xFu);
    } else {
        // Register offset: bits [11:8] must be zero per GBATEK.
        if (((instr >> 8) & 0xFu) != 0) {
            DS_LOG_WARN("arm7: halfword register-offset with non-zero bits[11:8] at 0x%08X",
                        state.pc);
        }
        const u32 rm = instr & 0xFu;
        if (rm == 15) {
            // GBATEK: "R0-R14, not including R15". UNPREDICTABLE on hardware;
            // we read r[15] for determinism.
            DS_LOG_WARN("arm7: halfword register offset Rm == 15 (unpredictable) at 0x%08X",
                        state.pc);
        }
        offset = state.r[rm];
    }

    const u32 signed_offset = u ? (rn_value + offset) : (rn_value - offset);

    HalfwordAddress out{};
    out.rn           = rn;
    out.writeback_rn = signed_offset;
    if (p) {
        out.address   = signed_offset;  // pre-index: access at new address
        out.writeback = w;              // only writes back if W==1
    } else {
        out.address   = rn_value;       // post-index: access at original Rn
        out.writeback = true;           // post-index always writes back
        // GBATEK: "Bit 21 not used, must be zero" when P=0. Warn and
        // proceed — the W=1 bit is effectively ignored because post-index
        // already writes back.
        if (w) {
            DS_LOG_WARN("arm7: halfword post-index with W=1 (malformed) at 0x%08X",
                        state.pc);
        }
    }
    return out;
}

u32 load_halfword_unsigned(Arm7Bus& bus, u32 address) {
    // Aligned path only — Task 9 adds the address[0]==1 rotate-by-8 quirk.
    // We defensively mask the low bit so an odd address still returns a
    // well-defined value; Task 9 will supersede this line with the real
    // rotate behavior cross-referenced against melonDS.
    return static_cast<u32>(bus.read16(address & ~1u));
}

u32 load_halfword_signed(Arm7Bus& bus, u32 address) {
    // Aligned path only — Task 10 adds the address[0]==1 quirk, which
    // on ARM7TDMI degenerates to an LDRSB of the single odd byte.
    // We defensively mask the low bit so an odd address still returns
    // a well-defined value until Task 10 replaces this.
    const u16 raw = bus.read16(address & ~1u);
    return static_cast<u32>(static_cast<i32>(static_cast<i16>(raw)));
}

u32 load_byte_signed(Arm7Bus& bus, u32 address) {
    // Byte loads have no alignment concerns — there's only one byte at
    // any address. Read the byte, then sign-extend bit 7 through bits
    // 8..31 using the standard u8 -> i8 -> i32 -> u32 promotion chain.
    const u8 raw = bus.read8(address);
    return static_cast<u32>(static_cast<i32>(static_cast<i8>(raw)));
}

void store_halfword(Arm7Bus& bus, u32 address, u32 value) {
    // Aligned path only — Task 11 adds the address[0]==1 mask behavior.
    // We already mask defensively so an odd address still writes to the
    // aligned halfword without crashing; Task 11 makes this explicit.
    bus.write16(address & ~1u, static_cast<u16>(value & 0xFFFFu));
}

void write_rd_and_writeback(Arm7State& state, u32 rd, u32 value,
                            const HalfwordAddress& addr) {
    // Writeback first, but suppress entirely when Rn == Rd. Per §5.5 of
    // the slice design: on a load with writeback where Rn == Rd, the
    // loaded value wins — Rd receives the loaded value and Rn is not
    // observably updated.
    if (addr.writeback && addr.rn != rd) {
        state.r[addr.rn] = addr.writeback_rn;
    }
    state.r[rd] = value;
    if (rd == 15) {
        // Rd==R15 as a halfword load destination is UNPREDICTABLE on
        // ARMv4T. Match the house write_rd() helper: mask to word
        // alignment, leave CPSR.T unchanged (GBATEK: "LDR PC,<op> on
        // ARMv4 leaves CPSR.T unchanged").
        DS_LOG_WARN("arm7: halfword load with Rd == 15 (unpredictable) at 0x%08X",
                    state.pc);
        state.pc = value & ~0x3u;
    }
}

}  // namespace

u32 dispatch_halfword(Arm7State& state, Arm7Bus& bus, u32 instr, u32 instr_addr) {
    const bool l  = (instr & (1u << 20)) != 0;
    const u32  sh = (instr >> 5) & 0x3u;

    if (!l) {
        // Store-side SH slots.
        switch (sh) {
            case 0:
                DS_LOG_WARN("arm7: SWP encoding encountered, not yet supported at 0x%08X",
                            state.pc);
                break;
            case 1: {
                const HalfwordAddress addr = compute_halfword_address(state, instr);
                const u32 rd = (instr >> 12) & 0xFu;
                // Rd == R15 in a halfword store reads as PC+12, matching the
                // word-STR pipeline convention (see arm7_loadstore.cpp). No
                // Rn==Rd suppression here — that rule is load-only per §5.5
                // of the slice 3b3 design spec; stores always update Rn if
                // writeback is enabled.
                const u32 rd_value = (rd == 15) ? (instr_addr + 12u) : state.r[rd];
                store_halfword(bus, addr.address, rd_value);
                if (addr.writeback) {
                    state.r[addr.rn] = addr.writeback_rn;
                }
                break;
            }
            case 2:
            case 3:
                DS_LOG_WARN("arm7: LDRD/STRD (ARMv5TE) on ARM7 at 0x%08X", state.pc);
                break;
        }
    } else {
        // Load-side SH slots.
        switch (sh) {
            case 0:
                DS_LOG_WARN("arm7: halfword load with SH=0 (reserved) at 0x%08X", state.pc);
                break;
            case 1: {
                const HalfwordAddress addr = compute_halfword_address(state, instr);
                const u32 rd    = (instr >> 12) & 0xFu;
                const u32 value = load_halfword_unsigned(bus, addr.address);
                write_rd_and_writeback(state, rd, value, addr);
                break;
            }
            case 2: {
                const HalfwordAddress addr = compute_halfword_address(state, instr);
                const u32 rd    = (instr >> 12) & 0xFu;
                const u32 value = load_byte_signed(bus, addr.address);
                write_rd_and_writeback(state, rd, value, addr);
                break;
            }
            case 3: {
                const HalfwordAddress addr = compute_halfword_address(state, instr);
                const u32 rd    = (instr >> 12) & 0xFu;
                const u32 value = load_halfword_signed(bus, addr.address);
                write_rd_and_writeback(state, rd, value, addr);
                break;
            }
        }
    }
    return 1;  // TODO(cycles): 1S+1N+1I for loads, 2N for STRH per GBATEK.
}

}  // namespace ds
