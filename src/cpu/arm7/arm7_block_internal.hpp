#pragma once

// Private header exposing LDM/STM addressing helpers implemented in
// arm7_block.cpp. Shared only with the arm7_block_test unit test so it
// can exercise the normalization math directly, without standing up a
// full NDS harness. Not part of any public include path.
//
// See docs/specs/2026-04-15-arm7-core-phase1-slice3b4-design.md §4.3
// and §5.4 for the derivation of the formulas below.

#include "ds/common.hpp"

namespace ds::arm7_block_detail {

// Forward-walking normalization of an LDM/STM transfer. `start_addr` is
// the lowest-address word touched by the transfer loop; `wb_value` is
// the value that gets written back to Rn when W=1 (and the Rn-in-list
// rules permit it). See §4.3 of the slice 3b4 spec.
struct BlockAddressing {
    u32 start_addr; // address of the first word transferred (low-to-high walk)
    u32 wb_value;   // value to write back to Rn if W=1
};

// popcount of the low 16 bits of an LDM/STM register list. Bits above
// bit 15 are ignored because the encoded register list is a 16-bit
// field; callers still pass the whole instruction word on occasion.
u32 reg_list_count(u32 reg_list);

// Normalize any {P, U} combination into a forward-walking start address
// and precomputed writeback value:
//
//   if U == 1:  start = Rn + (P ? 4 : 0);   wb = Rn + 4 * n
//   else:       start = Rn - 4 * n + (P ? 0 : 4); wb = Rn - 4 * n
//
// All arithmetic is 32-bit wrapping. The empty-list case (n == 0)
// produces wb == Rn and start == Rn (or Rn + 4 for IB), which is
// correct for the no-op path documented in spec §5.8.
BlockAddressing compute_block_addressing(u32 rn_value, u32 reg_list, bool p_bit, bool u_bit);

} // namespace ds::arm7_block_detail
