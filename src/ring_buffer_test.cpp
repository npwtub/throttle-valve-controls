#include <stdio.h>
#include <assert.h>
#include "ring_buffer.h"

RingBuffer<uint16_t, 3> buffer;

int main() {

    // Starts empty.
    assert(buffer.empty());
    assert(buffer.size() == 0);

    uint16_t value = 99;
    assert(!buffer.pop(value));
    assert(value == 99);  // Failed pop leaves the output unchanged.

    // Fill all three slots.
    assert(buffer.push(10));
    assert(buffer.push(20));
    assert(buffer.push(30));
    assert(buffer.full());

    // A full buffer rejects the new record.
    assert(!buffer.push(40));
    assert(buffer.dropped() == 1);
    assert(buffer.size() == 3);

    // Remove the oldest record, freeing one slot.
    assert(buffer.pop(value));
    assert(value == 10);

    // Reuse that slot after wrapping around.
    assert(buffer.push(40));

    // Records must still come out in insertion order.
    assert(buffer.pop(value));
    assert(value == 20);

    assert(buffer.pop(value));
    assert(value == 30);

    assert(buffer.pop(value));
    assert(value == 40);

    assert(buffer.empty());
    assert(buffer.dropped() == 1);

    printf("Ring buffer tests passed\n");
return 0;
}

