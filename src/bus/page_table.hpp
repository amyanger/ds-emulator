#pragma once

#include "ds/common.hpp"

#include <array>

namespace ds {

// A single 16-MB-region entry in a 256-entry page table. When `ptr` is
// non-null, `addr & mask` is the byte offset into the region's backing
// store. When `ptr` is null, the access falls through to the owning bus's
// slow-path dispatcher (I/O, open bus, sub-region split).
struct PageEntry {
    u8* ptr  = nullptr;
    u32 mask = 0;
};

// 256 entries keyed on the top 8 bits of a 32-bit physical address. One
// entry covers 16 MB of address space. Regions smaller than 16 MB are
// handled by `mask` (e.g., 4 MB main RAM mirrored four times across 16 MB
// uses mask = 0x003F'FFFF).
//
// Per design spec §3.4 the final implementation will split the table by
// access size (read8/16/32, write8/16/32) to support 8-bit-write promotion
// on palette / OAM / VRAM. This slice uses a single shared table because
// those regions don't exist yet. See task 7 for the spec amendment.
struct PageTable {
    std::array<PageEntry, 256> read{};
    std::array<PageEntry, 256> write{};

    void clear() {
        read.fill({});
        write.fill({});
    }
};

}  // namespace ds
