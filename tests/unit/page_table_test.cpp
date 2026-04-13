// Unit tests for PageTable / PageEntry. Exercises the fast-path shape
// without any bus class wrapping it, so failures localize here before
// Arm9Bus / Arm7Bus add their own behavior on top.

#include "bus/page_table.hpp"
#include "require.hpp"

#include <array>

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
    table.read [0x02] = PageEntry{storage.data(), 0x0000'000F};
    table.write[0x02] = PageEntry{storage.data(), 0x0000'000F};

    table.clear();

    REQUIRE(table.read [0x02].ptr == nullptr);
    REQUIRE(table.write[0x02].ptr == nullptr);
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

int main() {
    default_constructed_is_all_unmapped();
    clear_resets_all_entries();
    mask_mirrors_short_storage_across_the_region();
    return 0;
}
