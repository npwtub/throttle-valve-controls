#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

class LogQueue {
public:
    static constexpr size_t CAPACITY = 2048;

    // Add a complete record.
    // If it cannot fit, leave the queue unchanged.
    bool enqueue(const uint8_t* data, size_t length) {
        if (data == nullptr || length == 0 ||
            length > freeSpace()) {
            return false;
        }

        size_t firstPart = CAPACITY - writeIndex_;

        if (firstPart > length) {
            firstPart = length;
        }

        memcpy(storage_ + writeIndex_, data, firstPart);

        const size_t secondPart = length - firstPart;

        if (secondPart > 0) {
            memcpy(storage_, data + firstPart, secondPart);
        }

        writeIndex_ = (writeIndex_ + length) % CAPACITY;
        used_ += length;
        return true;
    }

    // Address of the oldest queued bytes.
    const uint8_t* front() const {
        return storage_ + readIndex_;
    }

    // Number of bytes available before the memory wraps,
    // limited to the requested maximum.
    size_t contiguousSize(size_t maximum = 512) const {
        size_t count = CAPACITY - readIndex_;

        if (count > used_) {
            count = used_;
        }

        if (count > maximum) {
            count = maximum;
        }

        return count;
    }

    // Remove only bytes the SD writer confirms it accepted.
    bool consume(size_t count) {
        if (count > used_) {
            return false;
        }

        readIndex_ = (readIndex_ + count) % CAPACITY;
        used_ -= count;
        return true;
    }

    size_t size() const {
        return used_;
    }

    size_t freeSpace() const {
        return CAPACITY - used_;
    }

    void reset() {
        readIndex_ = 0;
        writeIndex_ = 0;
        used_ = 0;
    }

private:
    uint8_t storage_[CAPACITY] = {};
    size_t readIndex_ = 0;
    size_t writeIndex_ = 0;
    size_t used_ = 0;
};