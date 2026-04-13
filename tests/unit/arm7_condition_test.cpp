// 16-case condition code evaluator. Every case exercised against a
// CPSR with the relevant flag bit toggled.

#include "cpu/arm7/arm7_alu.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

// CPSR flag bit positions.
static constexpr u32 N_BIT = 31;
static constexpr u32 Z_BIT = 30;
static constexpr u32 C_BIT = 29;
static constexpr u32 V_BIT = 28;

static constexpr u32 flag(u32 pos) { return 1u << pos; }

static void al_always_true_nv_always_false() {
    REQUIRE(eval_condition(0xE, 0) == true);
    REQUIRE(eval_condition(0xE, 0xFFFF'FFFFu) == true);
    REQUIRE(eval_condition(0xF, 0) == false);
    REQUIRE(eval_condition(0xF, 0xFFFF'FFFFu) == false);
}

static void eq_ne_track_z() {
    REQUIRE(eval_condition(0x0, flag(Z_BIT)) == true);
    REQUIRE(eval_condition(0x0, 0) == false);
    REQUIRE(eval_condition(0x1, flag(Z_BIT)) == false);
    REQUIRE(eval_condition(0x1, 0) == true);
}

static void cs_cc_track_c() {
    REQUIRE(eval_condition(0x2, flag(C_BIT)) == true);
    REQUIRE(eval_condition(0x2, 0) == false);
    REQUIRE(eval_condition(0x3, flag(C_BIT)) == false);
    REQUIRE(eval_condition(0x3, 0) == true);
}

static void mi_pl_track_n() {
    REQUIRE(eval_condition(0x4, flag(N_BIT)) == true);
    REQUIRE(eval_condition(0x4, 0) == false);
    REQUIRE(eval_condition(0x5, flag(N_BIT)) == false);
    REQUIRE(eval_condition(0x5, 0) == true);
}

static void vs_vc_track_v() {
    REQUIRE(eval_condition(0x6, flag(V_BIT)) == true);
    REQUIRE(eval_condition(0x6, 0) == false);
    REQUIRE(eval_condition(0x7, flag(V_BIT)) == false);
    REQUIRE(eval_condition(0x7, 0) == true);
}

static void hi_ls_compound() {
    // HI = C && !Z
    REQUIRE(eval_condition(0x8, flag(C_BIT))                == true);
    REQUIRE(eval_condition(0x8, flag(C_BIT) | flag(Z_BIT))  == false);
    REQUIRE(eval_condition(0x8, 0)                          == false);
    // LS = !C || Z
    REQUIRE(eval_condition(0x9, flag(C_BIT))                == false);
    REQUIRE(eval_condition(0x9, flag(C_BIT) | flag(Z_BIT))  == true);
    REQUIRE(eval_condition(0x9, 0)                          == true);
}

static void ge_lt_gt_le_track_nv() {
    // N=V → GE true, LT false.
    REQUIRE(eval_condition(0xA, 0) == true);
    REQUIRE(eval_condition(0xA, flag(N_BIT) | flag(V_BIT)) == true);
    REQUIRE(eval_condition(0xA, flag(N_BIT)) == false);
    REQUIRE(eval_condition(0xB, flag(N_BIT)) == true);

    // GT = !Z && N==V
    REQUIRE(eval_condition(0xC, 0) == true);
    REQUIRE(eval_condition(0xC, flag(Z_BIT)) == false);
    REQUIRE(eval_condition(0xC, flag(N_BIT)) == false);
    REQUIRE(eval_condition(0xC, flag(N_BIT) | flag(V_BIT)) == true);
    // LE = Z || N!=V
    REQUIRE(eval_condition(0xD, flag(Z_BIT)) == true);
    REQUIRE(eval_condition(0xD, flag(N_BIT)) == true);
    REQUIRE(eval_condition(0xD, flag(N_BIT) | flag(V_BIT)) == false);
}

int main() {
    al_always_true_nv_always_false();
    eq_ne_track_z();
    cs_cc_track_c();
    mi_pl_track_n();
    vs_vc_track_v();
    hi_ls_compound();
    ge_lt_gt_le_track_nv();
    std::puts("arm7_condition_test OK");
    return 0;
}
