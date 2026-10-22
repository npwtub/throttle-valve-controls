#pragma once

#include <stdint.h>

template <typename Record, uint16_t Capacity>
class RingBuffer {
    static_assert(Capacity > 0, "Ring buffer capacity must be positive");

public:
uint16_t size() const {
    return count;
}

bool empty() const {
    return count == 0;
}

bool full() const {
    return count == Capacity;
}

uint32_t dropped() const {
    return droppedRecords;
}
bool push(const Record& record) {
    if (count == Capacity) {
        ++droppedRecords;
        return false;
    }

    records[head] = record;

    ++head;
    if (head == Capacity) {
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
    if (tail == Capacity) {
        tail = 0;
    }

    --count;
    return true;
}
private:
    Record records[Capacity] = {};

    uint16_t head = 0;
    uint16_t tail = 0;
    uint16_t count = 0;

    uint32_t droppedRecords = 0;
};
