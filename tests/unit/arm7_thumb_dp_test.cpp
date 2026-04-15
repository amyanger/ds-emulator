// arm7_thumb_dp_test.cpp — THUMB.3 MOV/CMP/ADD/SUB imm8 tests.
// Verifies encoding decode, register writeback, and flag behavior,
// with particular attention to MOV's NZ-only flag rule (C/V preserved).

#include "bus/arm7_bus.hpp"
#include "cpu/arm7/arm7.hpp"
#include "nds.hpp"
#include "require.hpp"

using namespace ds;

// Encode THUMB.3: 001 op2 Rd3 imm8
static u16 thumb3(u32 op, u32 rd, u32 imm8) {
    return static_cast<u16>((0b001u << 13) | (op << 11) | (rd << 8) | imm8);
}

static constexpr u32 BASE = 0x0380'0000u;

static bool flag_n(u32 cpsr) {
    return (cpsr & (1u << 31)) != 0;
}
static bool flag_z(u32 cpsr) {
    return (cpsr & (1u << 30)) != 0;
}
static bool flag_c(u32 cpsr) {
    return (cpsr & (1u << 29)) != 0;
}
static bool flag_v(u32 cpsr) {
    return (cpsr & (1u << 28)) != 0;
}

static void setup_thumb(NDS& nds, u32 addr) {
    nds.cpu7().state().pc = addr;
    nds.cpu7().state().cpsr |= (1u << 5);
}

static void step_one(NDS& nds) {
    const u64 before = nds.cpu7().state().cycles;
    nds.cpu7().run_until((before + 1) * 2);
}

// ---- MOV ----

static void thumb3_mov_basic() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(0, 3, 42));
    setup_thumb(nds, BASE);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[3] == 42);
}

static void thumb3_mov_zero_sets_z_clears_n() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(0, 0, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().cpsr |= (1u << 31); // N set before
    nds.cpu7().state().r[0] = 0xDEAD'BEEFu;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0);
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
}

static void thumb3_mov_preserves_c_and_v_when_set() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(0, 1, 0xFF));
    setup_thumb(nds, BASE);
    nds.cpu7().state().cpsr |= (1u << 29) | (1u << 28); // C=1, V=1
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[1] == 0xFF);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // preserved
    REQUIRE(flag_v(nds.cpu7().state().cpsr)); // preserved
}

static void thumb3_mov_preserves_c_and_v_when_clear() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(0, 2, 5));
    setup_thumb(nds, BASE);
    nds.cpu7().state().cpsr &= ~((1u << 29) | (1u << 28)); // C=0, V=0
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[2] == 5);
    REQUIRE(!flag_c(nds.cpu7().state().cpsr));
    REQUIRE(!flag_v(nds.cpu7().state().cpsr));
}

static void thumb3_mov_max_imm() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(0, 7, 0xFF));
    setup_thumb(nds, BASE);
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[7] == 0xFF);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
}

// ---- CMP ----

static void thumb3_cmp_equal_sets_z_and_c() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(1, 0, 42));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 42;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 42); // no writeback
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // a >= b, no borrow
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_v(nds.cpu7().state().cpsr));
}

static void thumb3_cmp_less_sets_n_clears_c() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(1, 1, 100));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[1] = 50;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[1] == 50);
    REQUIRE(flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr)); // borrow
}

static void thumb3_cmp_greater() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(1, 2, 10));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[2] = 200;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[2] == 200);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr));
}

static void thumb3_cmp_clears_stale_v() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(1, 3, 5));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[3] = 10;
    nds.cpu7().state().cpsr |= (1u << 28); // V=1 before
    step_one(nds);
    // 10 - 5 = 5: no signed overflow, V must be cleared
    REQUIRE(nds.cpu7().state().r[3] == 10);
    REQUIRE(!flag_v(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr));
}

// ---- ADD ----

static void thumb3_add_basic() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(2, 3, 10));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[3] = 100;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[3] == 110);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr));
}

static void thumb3_add_wrap_to_zero() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(2, 0, 1));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0xFFFF'FFFFu;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 0);
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // carry out
}

static void thumb3_add_signed_overflow() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(2, 4, 1));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[4] = 0x7FFF'FFFFu;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[4] == 0x8000'0000u);
    REQUIRE(flag_n(nds.cpu7().state().cpsr));
    REQUIRE(flag_v(nds.cpu7().state().cpsr));
}

static void thumb3_add_zero_imm() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(2, 5, 0));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[5] = 42;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[5] == 42);
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr));
}

// ---- SUB ----

static void thumb3_sub_basic() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(3, 5, 20));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[5] = 100;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[5] == 80);
    REQUIRE(!flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // no borrow
}

static void thumb3_sub_to_zero() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(3, 6, 50));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[6] = 50;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[6] == 0);
    REQUIRE(flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr)); // equal means no borrow
}

static void thumb3_sub_underflow() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(3, 7, 1));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[7] = 0;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[7] == 0xFFFF'FFFFu);
    REQUIRE(flag_n(nds.cpu7().state().cpsr));
    REQUIRE(!flag_c(nds.cpu7().state().cpsr)); // borrow
}

static void thumb3_sub_max_imm() {
    NDS nds;
    nds.arm7_bus().write16(BASE, thumb3(3, 0, 0xFF));
    setup_thumb(nds, BASE);
    nds.cpu7().state().r[0] = 0x100;
    step_one(nds);
    REQUIRE(nds.cpu7().state().r[0] == 1);
    REQUIRE(!flag_z(nds.cpu7().state().cpsr));
    REQUIRE(flag_c(nds.cpu7().state().cpsr));
}

int main() {
    thumb3_mov_basic();
    thumb3_mov_zero_sets_z_clears_n();
    thumb3_mov_preserves_c_and_v_when_set();
    thumb3_mov_preserves_c_and_v_when_clear();
    thumb3_mov_max_imm();

    thumb3_cmp_equal_sets_z_and_c();
    thumb3_cmp_less_sets_n_clears_c();
    thumb3_cmp_greater();
    thumb3_cmp_clears_stale_v();

    thumb3_add_basic();
    thumb3_add_wrap_to_zero();
    thumb3_add_signed_overflow();
    thumb3_add_zero_imm();

    thumb3_sub_basic();
    thumb3_sub_to_zero();
    thumb3_sub_underflow();
    thumb3_sub_max_imm();

    return 0;
}
