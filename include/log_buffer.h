#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

class LogRecordBuffer : public Print {
public:
    // Initial limit for one formatted CSV row.
    static constexpr size_t CAPACITY = 256;

    // Keep Print's other write() overloads available.
    using Print::write;

    size_t write(uint8_t value) override {
        if (overflowed_ || length_ >= CAPACITY) {
            overflowed_ = true;
            return 0;
        }

        data_[length_++] = value;
        return 1;
    }

    void reset() {
        length_ = 0;
        overflowed_ = false;
        clearWriteError();
    }

    const uint8_t* data() const {
        return data_;
    }

    size_t size() const {
        return length_;
    }

    bool overflowed() const {
        return overflowed_;
    }

private:
    uint8_t data_[CAPACITY] = {};
    size_t length_ = 0;
    bool overflowed_ = false;
};