#pragma once

#include "bus/page_table.hpp"
#include "ds/common.hpp"

namespace ds {

class NDS;
class WramControl;

// ARM7's view of memory. Shape mirrors Arm9Bus except:
//   - No write access to ARM9-only I/O (WRAMCNT is ignored here).
//   - Region 0x03 is handled entirely in the slow path because it holds
//     two distinct sub-regions: 0x0300'0000 (shared WRAM, WRAMCNT-gated)
//     and 0x0380'0000 (ARM7 WRAM, always). A single 16 MB page entry
//     can't distinguish them.
class Arm7Bus {
public:
    Arm7Bus(NDS& nds, u8* main_ram, u8* shared_wram, u8* arm7_wram,
            const WramControl& wram_ctl);

    void reset();

    u32 read32(u32 addr);
    u16 read16(u32 addr);
    u8  read8 (u32 addr);

    void write32(u32 addr, u32 value);
    void write16(u32 addr, u16 value);
    void write8 (u32 addr, u8  value);

    // Called by NDS after a WRAMCNT write. ARM7's page table itself does
    // not change (entry 0x03 is always nullptr), but the slow-path lookup
    // reads from WramControl so there is nothing to rebuild. This method
    // exists to keep the bus interface symmetric with Arm9Bus.
    void rebuild_shared_wram() {}

private:
    u32  slow_read32 (u32 addr);
    u16  slow_read16 (u32 addr);
    u8   slow_read8  (u32 addr);
    void slow_write32(u32 addr, u32 value);
    void slow_write16(u32 addr, u16 value);
    void slow_write8 (u32 addr, u8  value);

    // Resolve a 0x03xx'xxxx address to the backing pointer + mask for the
    // current WRAMCNT state. Always returns a non-null ptr — mode 0 gives
    // ARM7 its own WRAM mirror at 0x0300'0000, not unmapped.
    PageEntry resolve_region3(u32 addr) const;

    NDS*               nds_         = nullptr;
    u8*                main_ram_    = nullptr;
    u8*                shared_wram_ = nullptr;
    u8*                arm7_wram_   = nullptr;
    const WramControl* wram_ctl_    = nullptr;
    PageTable          table_{};
};

}  // namespace ds
