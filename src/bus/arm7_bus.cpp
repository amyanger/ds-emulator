#include "bus/arm7_bus.hpp"

#include "bus/wram_control.hpp"
#include "nds.hpp"

#include <cstring>

namespace ds {

namespace {
constexpr u8  kRegionMainRam = 0x02;
constexpr u32 kMainRamMask   = 0x003F'FFFFu;
constexpr u32 kArm7WramMask  = 0x0000'FFFFu;  // 64 KB mirror
}  // namespace

Arm7Bus::Arm7Bus(NDS& nds, u8* main_ram, u8* shared_wram, u8* arm7_wram,
                 const WramControl& wram_ctl)
    : nds_(&nds),
      main_ram_(main_ram),
      shared_wram_(shared_wram),
      arm7_wram_(arm7_wram),
      wram_ctl_(&wram_ctl) {}

void Arm7Bus::reset() {
    table_.clear();
    table_.read [kRegionMainRam] = PageEntry{main_ram_, kMainRamMask};
    table_.write[kRegionMainRam] = PageEntry{main_ram_, kMainRamMask};
    // Entry 0x03 stays nullptr — slow path handles shared/ARM7 WRAM split.
}

// Fast-path readers/writers are identical in shape to Arm9Bus. Duplicated
// on purpose so each bus is readable top-to-bottom without chasing a
// template.
u32 Arm7Bus::read32(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        u32 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~3u), sizeof(u32));
        return value;
    }
    return slow_read32(addr);
}

u16 Arm7Bus::read16(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        u16 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~1u), sizeof(u16));
        return value;
    }
    return slow_read16(addr);
}

u8 Arm7Bus::read8(u32 addr) {
    const PageEntry& e = table_.read[addr >> 24];
    if (e.ptr != nullptr) {
        return e.ptr[addr & e.mask];
    }
    return slow_read8(addr);
}

void Arm7Bus::write32(u32 addr, u32 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        std::memcpy(e.ptr + ((addr & e.mask) & ~3u), &value, sizeof(u32));
        return;
    }
    slow_write32(addr, value);
}

void Arm7Bus::write16(u32 addr, u16 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        std::memcpy(e.ptr + ((addr & e.mask) & ~1u), &value, sizeof(u16));
        return;
    }
    slow_write16(addr, value);
}

void Arm7Bus::write8(u32 addr, u8 value) {
    const PageEntry& e = table_.write[addr >> 24];
    if (e.ptr != nullptr) {
        e.ptr[addr & e.mask] = value;
        return;
    }
    slow_write8(addr, value);
}

// --- slow path ------------------------------------------------------------

PageEntry Arm7Bus::resolve_region3(u32 addr) const {
    // 0x0380'0000-0x03FF'FFFF is always ARM7 WRAM, 64 KB mirrored.
    if (addr & 0x0080'0000u) {
        return PageEntry{arm7_wram_, kArm7WramMask};
    }
    // 0x0300'0000-0x037F'FFFF depends on WRAMCNT. Verified against
    // melonDS src/NDS.cpp::MapSharedWRAM — ARM7's slice is the complement
    // of ARM9's.
    //   0: ARM7 sees ARM7 WRAM mirror (no shared-WRAM access)
    //   1: ARM7 sees shared WRAM first  16 KB (offset 0x0000)
    //   2: ARM7 sees shared WRAM second 16 KB (offset 0x4000)
    //   3: ARM7 sees full 32 KB shared WRAM (mirror)
    const u8 mode = wram_ctl_->value();
    switch (mode) {
        case 0: return PageEntry{arm7_wram_,              kArm7WramMask};
        case 1: return PageEntry{shared_wram_,            0x0000'3FFFu};
        case 2: return PageEntry{shared_wram_ + 0x4000u,  0x0000'3FFFu};
        case 3: return PageEntry{shared_wram_,            0x0000'7FFFu};
    }
    return PageEntry{};  // unreachable
}

u32 Arm7Bus::slow_read32(u32 addr) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        u32 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~3u), sizeof(u32));
        return value;
    }
    if ((addr >> 24) == 0x04) return nds_->arm7_io_read32(addr);
    return 0;
}

u16 Arm7Bus::slow_read16(u32 addr) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        u16 value;
        std::memcpy(&value, e.ptr + ((addr & e.mask) & ~1u), sizeof(u16));
        return value;
    }
    if ((addr >> 24) == 0x04) return nds_->arm7_io_read16(addr);
    return 0;
}

u8 Arm7Bus::slow_read8(u32 addr) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        return e.ptr[addr & e.mask];
    }
    if ((addr >> 24) == 0x04) return nds_->arm7_io_read8(addr);
    return 0;
}

void Arm7Bus::slow_write32(u32 addr, u32 value) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        std::memcpy(e.ptr + ((addr & e.mask) & ~3u), &value, sizeof(u32));
        return;
    }
    if ((addr >> 24) == 0x04) nds_->arm7_io_write32(addr, value);
}

void Arm7Bus::slow_write16(u32 addr, u16 value) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        std::memcpy(e.ptr + ((addr & e.mask) & ~1u), &value, sizeof(u16));
        return;
    }
    if ((addr >> 24) == 0x04) nds_->arm7_io_write16(addr, value);
}

void Arm7Bus::slow_write8(u32 addr, u8 value) {
    if ((addr >> 24) == 0x03) {
        const PageEntry e = resolve_region3(addr);
        e.ptr[addr & e.mask] = value;
        return;
    }
    if ((addr >> 24) == 0x04) nds_->arm7_io_write8(addr, value);
}

}  // namespace ds
