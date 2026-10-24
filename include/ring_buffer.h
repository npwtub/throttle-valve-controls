#pragma once

#include <stdint.h>

template <typename Record, uint16_t capacity>
class RingBuffer {
    static_assert(capacity > 0, "Ring buffer capacity must be positive");

public:
uint16_t size() const {
    return count;
}

bool empty() const {
    return count == 0;
}

bool full() const {
    return count == capacity;
}

uint32_t dropped() const {
    return droppedRecords;
}
bool push(const Record& record) {
    if (count == capacity) {
        ++droppedRecords;
        return false;
    }

    records[head] = record;

    ++head;
    if (head == capacity) {
        head = 0;
    }

    ++count;
    return true;
}
bool pop(Record& record) {
    if (count == 0) {
        return false;
    }

    record = records[tail];

    ++tail;
    if (tail == capacity) {
        tail = 0;
    }

    --count;
    return true;
}
private:
    Record records[capacity] = {};

    uint16_t head = 0;
    uint16_t tail = 0;
    uint16_t count = 0;

    uint32_t droppedRecords = 0;
};
