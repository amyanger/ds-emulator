// tests/unit/ring_buffer_test.cpp
#include "ds/ring_buffer.hpp"
#include "require.hpp"

#include <cstdio>

using namespace ds;

static void test_empty_new() {
    CircularBuffer<int, 4> rb;
    REQUIRE(rb.size() == 0);
    REQUIRE(rb.empty());
    REQUIRE(!rb.full());
}

static void test_push_and_size() {
    CircularBuffer<int, 4> rb;
    rb.push(10);
    REQUIRE(rb.size() == 1);
    REQUIRE(!rb.empty());
    rb.push(20);
    rb.push(30);
    REQUIRE(rb.size() == 3);
}

static void test_pop_fifo_order() {
    CircularBuffer<int, 4> rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    REQUIRE(rb.pop() == 1);
    REQUIRE(rb.pop() == 2);
    REQUIRE(rb.pop() == 3);
    REQUIRE(rb.empty());
}

static void test_full() {
    CircularBuffer<int, 4> rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    rb.push(4);
    REQUIRE(rb.full());
    REQUIRE(rb.size() == 4);
}

static void test_wrap_around() {
    CircularBuffer<int, 4> rb;
    rb.push(1); rb.push(2); rb.push(3); rb.push(4);
    REQUIRE(rb.pop() == 1);
    REQUIRE(rb.pop() == 2);
    rb.push(5);
    rb.push(6);
    REQUIRE(rb.pop() == 3);
    REQUIRE(rb.pop() == 4);
    REQUIRE(rb.pop() == 5);
    REQUIRE(rb.pop() == 6);
    REQUIRE(rb.empty());
}

static void test_clear() {
    CircularBuffer<int, 4> rb;
    rb.push(1); rb.push(2);
    rb.clear();
    REQUIRE(rb.empty());
    REQUIRE(rb.size() == 0);
}

int main() {
    test_empty_new();
    test_push_and_size();
    test_pop_fifo_order();
    test_full();
    test_wrap_around();
    test_clear();
    std::printf("ring_buffer_test: all passed\n");
    return 0;
}
