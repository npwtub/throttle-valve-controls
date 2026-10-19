#pragma once

#include <Arduino.h>
#include <stdint.h>

class LogClock {
public:
    // Call regularly, even before time synchronization.
    void update() {
        const uint32_t nowMs = millis();

        // Unsigned subtraction handles millis() rolling over,
        // provided update() runs at least once per rollover period.
        elapsedMs_ += static_cast<uint32_t>(nowMs - lastMillis_);
        lastMillis_ = nowMs;
    }

    // The computer supplies UTC as Unix epoch milliseconds.
    // Returns false if the timestamp is invalid or already set.
    bool synchronize(uint64_t utcMs) {
        if (synchronized_ || utcMs == 0) {
            return false;
        }

        update();
        utcAtSyncMs_ = utcMs;
        elapsedAtSyncMs_ = elapsedMs_;
        synchronized_ = true;
        return true;
    }

    bool isSynchronized() const {
        return synchronized_;
    }

    // Returns false instead of inventing a UTC timestamp
    // when no synchronization message has been received.
    bool getUtcMs(uint64_t& result) {
        update();

        if (!synchronized_) {
            return false;
        }

        result = utcAtSyncMs_ + (elapsedMs_ - elapsedAtSyncMs_);
        return true;
    }

private:
    uint32_t lastMillis_ = 0;
    uint64_t elapsedMs_ = 0;
    uint64_t utcAtSyncMs_ = 0;
    uint64_t elapsedAtSyncMs_ = 0;
    bool synchronized_ = false;
};