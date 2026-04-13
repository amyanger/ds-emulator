#pragma once

#include "bus/page_table.hpp"
#include "ds/common.hpp"

namespace ds {

class NDS;
class WramControl;

// ARM9's view of memory. Owns a 256-entry page table keyed on the top 8
// bits of the address. Fast-path reads/writes do a single lookup + memcpy.
// Unmapped addresses and I/O regions fall through to the owning NDS's I/O
// dispatcher. Only WRAMCNT (0x0400'0247) is wired in this slice; all other
// I/O addresses read as 0 and ignore writes until later slices.
class Arm9Bus {
public:
    Arm9Bus(NDS& nds, u8* main_ram, u8* shared_wram, const WramControl& wram_ctl);

    void reset();

    u32 read32(u32 addr);
    u16 read16(u32 addr);
    u8  read8 (u32 addr);

    void write32(u32 addr, u32 value);
    void write16(u32 addr, u16 value);
    void write8 (u32 addr, u8  value);

    // Called by NDS after a WRAMCNT write to rebuild only the 0x03 entry.
    void rebuild_shared_wram();

private:
    u32  slow_read32 (u32 addr);
    u16  slow_read16 (u32 addr);
    u8   slow_read8  (u32 addr);
    void slow_write32(u32 addr, u32 value);
    void slow_write16(u32 addr, u16 value);
    void slow_write8 (u32 addr, u8  value);

    NDS*               nds_         = nullptr;
    u8*                main_ram_    = nullptr;
    u8*                shared_wram_ = nullptr;
    const WramControl* wram_ctl_    = nullptr;
    PageTable          table_{};
};

}  // namespace ds
