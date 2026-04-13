// Unit tests for PageTable / PageEntry and WramControl. Exercises the
// fast-path shape without any bus class wrapping it, so failures localize
// here before Arm9Bus / Arm7Bus add their own behavior on top.

#include "bus/page_table.hpp"
#include "bus/wram_control.hpp"
#include "require.hpp"

#include <array>
#include <cstdio>

using namespace ds;

static void default_constructed_is_all_unmapped() {
    PageTable table;
    for (const auto& e : table.read) {
        REQUIRE(e.ptr == nullptr);
        REQUIRE(e.mask == 0);
    }
    for (const auto& e : table.write) {
        REQUIRE(e.ptr == nullptr);
        REQUIRE(e.mask == 0);
    }
}

static void clear_resets_all_entries() {
    PageTable table;
    std::array<u8, 16> storage{};

    // Populate multiple slots with non-zero ptr AND non-zero mask so a
    // future clear() regression that drops one of the two fields fails.
    table.read [0x00] = PageEntry{storage.data(), 0x0000'000F};
    table.read [0x02] = PageEntry{storage.data(), 0x003F'FFFF};
    table.read [0xFF] = PageEntry{storage.data(), 0xFFFF'FFFF};
    table.write[0x00] = PageEntry{storage.data(), 0x0000'000F};
    table.write[0x02] = PageEntry{storage.data(), 0x003F'FFFF};
    table.write[0xFF] = PageEntry{storage.data(), 0xFFFF'FFFF};

    table.clear();

    for (const auto& e : table.read) {
        REQUIRE(e.ptr == nullptr);
        REQUIRE(e.mask == 0);
    }
    for (const auto& e : table.write) {
        REQUIRE(e.ptr == nullptr);
        REQUIRE(e.mask == 0);
    }
}

static void mask_mirrors_short_storage_across_the_region() {
    // 4-byte backing store, 16 MB page. Mask = 0x3 gives a 4-byte mirror.
    std::array<u8, 4> storage{0xAA, 0xBB, 0xCC, 0xDD};
    PageTable table;
    table.read[0x02] = PageEntry{storage.data(), 0x0000'0003};

    const PageEntry& e = table.read[0x02];

    // Mirror 0: 0x02000000 -> storage[0]
    REQUIRE(e.ptr[(0x0200'0000u) & e.mask] == 0xAA);
    // Mirror 1: 0x02000004 -> storage[0]
    REQUIRE(e.ptr[(0x0200'0004u) & e.mask] == 0xAA);
    // Mirror offset: 0x0200_0002 -> storage[2]
    REQUIRE(e.ptr[(0x0200'0002u) & e.mask] == 0xCC);
    // Far end: 0x02FF_FFFF -> storage[3]
    REQUIRE(e.ptr[(0x02FF'FFFFu) & e.mask] == 0xDD);
}

static void wram_control_reset_zeroes_value() {
    WramControl wc;
    wc.write(0x3);
    wc.reset();
    REQUIRE(wc.value() == 0);
}

static void wram_control_write_masks_reserved_bits() {
    WramControl wc;
    wc.write(0xFF);
    REQUIRE(wc.value() == 0x3);  // top 6 bits are reserved, read as zero
}

static void wram_control_write_passes_valid_modes_through() {
    WramControl wc;
    wc.write(0);       REQUIRE(wc.value() == 0);
    wc.write(0x1);     REQUIRE(wc.value() == 0x1);
    wc.write(0x2);     REQUIRE(wc.value() == 0x2);
    wc.write(0x3);     REQUIRE(wc.value() == 0x3);
}

int main() {
    default_constructed_is_all_unmapped();
    clear_resets_all_entries();
    mask_mirrors_short_storage_across_the_region();
    wram_control_reset_zeroes_value();
    wram_control_write_masks_reserved_bits();
    wram_control_write_passes_valid_modes_through();
    std::printf("page_table_test: all passed\n");
    return 0;
}
