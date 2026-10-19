#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#include "log_clock.h"

class LogTimeSync {
public:
    enum class Result {
        None,
        Synchronized,
        InvalidMessage,
        AlreadySynchronized
    };

    explicit LogTimeSync(LogClock& clock) : clock_(clock) {}

    // Feed this one character at a time from the serial input.
    // It never waits for more characters to arrive.
    Result feed(char character) {
        if (character == '\n') {
            Result result = Result::InvalidMessage;

            if (!overflow_) {
                // Accept either LF or CRLF line endings.
                if (length_ > 0 && buffer_[length_ - 1] == '\r') {
                    --length_;
                }

                buffer_[length_] = '\0';
                result = parseLine();
            }

            reset();
            return result;
        }

        if (!overflow_) {
            if (length_ < sizeof(buffer_) - 1) {
                buffer_[length_++] = character;
            } else {
                // Discard the rest of an oversized line.
                overflow_ = true;
            }
        }

        return Result::None;
    }

    void reset() {
        length_ = 0;
        overflow_ = false;
    }

private:
    Result parseLine() {
        // "TIME," is 5 characters, followed by 13 digits.
        if (length_ != 18 || strncmp(buffer_, "TIME,", 5) != 0) {
            return Result::InvalidMessage;
        }

        // Require an actual 13-digit positive value.
        if (buffer_[5] < '1' || buffer_[5] > '9') {
            return Result::InvalidMessage;
        }

        uint64_t utcMs = 0;

        for (uint8_t index = 5; index < 18; ++index) {
            const char digit = buffer_[index];

            if (digit < '0' || digit > '9') {
                return Result::InvalidMessage;
            }

            utcMs = utcMs * 10 + static_cast<uint8_t>(digit - '0');
        }

        if (clock_.isSynchronized()) {
            return Result::AlreadySynchronized;
        }

        return clock_.synchronize(utcMs)
            ? Result::Synchronized
            : Result::InvalidMessage;
    }

    LogClock& clock_;
    char buffer_[32] = {};
    uint8_t length_ = 0;
    bool overflow_ = false;
};