// tests/unit/ring_buffer_test.cpp
#include "ds/ring_buffer.hpp"

#include <cassert>
#include <cstdio>

using namespace ds;

static void test_empty_new() {
    CircularBuffer<int, 4> rb;
    assert(rb.size() == 0);
    assert(rb.empty());
    assert(!rb.full());
}

static void test_push_and_size() {
    CircularBuffer<int, 4> rb;
    rb.push(10);
    assert(rb.size() == 1);
    assert(!rb.empty());
    rb.push(20);
    rb.push(30);
    assert(rb.size() == 3);
}

static void test_pop_fifo_order() {
    CircularBuffer<int, 4> rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    assert(rb.pop() == 1);
    assert(rb.pop() == 2);
    assert(rb.pop() == 3);
    assert(rb.empty());
}

static void test_full() {
    CircularBuffer<int, 4> rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);
    rb.push(4);
    assert(rb.full());
    assert(rb.size() == 4);
}

static void test_wrap_around() {
    CircularBuffer<int, 4> rb;
    rb.push(1); rb.push(2); rb.push(3); rb.push(4);
    assert(rb.pop() == 1);
    assert(rb.pop() == 2);
    rb.push(5);
    rb.push(6);
    assert(rb.pop() == 3);
    assert(rb.pop() == 4);
    assert(rb.pop() == 5);
    assert(rb.pop() == 6);
    assert(rb.empty());
}

static void test_clear() {
    CircularBuffer<int, 4> rb;
    rb.push(1); rb.push(2);
    rb.clear();
    assert(rb.empty());
    assert(rb.size() == 0);
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
