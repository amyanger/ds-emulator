// tests/unit/fixed_test.cpp
#include "ds/fixed.hpp"

#include <cassert>
#include <cstdio>

using namespace ds;

static void test_construct_and_to_int() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(5);
    assert(a.to_int() == 5);

    Fixed<20, 12> b = Fixed<20, 12>::from_raw(4096);  // 1.0 in 20.12
    assert(b.to_int() == 1);
}

static void test_addition() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(3);
    Fixed<20, 12> b = Fixed<20, 12>::from_int(4);
    Fixed<20, 12> c = a + b;
    assert(c.to_int() == 7);
}

static void test_subtraction() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(10);
    Fixed<20, 12> b = Fixed<20, 12>::from_int(3);
    Fixed<20, 12> c = a - b;
    assert(c.to_int() == 7);
}

static void test_multiplication() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(6);
    Fixed<20, 12> b = Fixed<20, 12>::from_int(7);
    Fixed<20, 12> c = a * b;
    assert(c.to_int() == 42);
}

static void test_multiplication_fractional() {
    // 1.5 * 2.0 = 3.0
    Fixed<20, 12> a = Fixed<20, 12>::from_raw(1 << 12 | 1 << 11);  // 1.5
    Fixed<20, 12> b = Fixed<20, 12>::from_int(2);
    Fixed<20, 12> c = a * b;
    assert(c.to_int() == 3);
}

static void test_negative() {
    Fixed<20, 12> a = Fixed<20, 12>::from_int(-5);
    assert(a.to_int() == -5);
    Fixed<20, 12> b = -a;
    assert(b.to_int() == 5);
}

static void test_raw_roundtrip() {
    const int32_t raw = 0x12345;
    Fixed<20, 12> a = Fixed<20, 12>::from_raw(raw);
    assert(a.raw() == raw);
}

int main() {
    test_construct_and_to_int();
    test_addition();
    test_subtraction();
    test_multiplication();
    test_multiplication_fractional();
    test_negative();
    test_raw_roundtrip();
    std::printf("fixed_test: all passed\n");
    return 0;
}
