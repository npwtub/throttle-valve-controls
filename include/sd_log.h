#pragma once

#include <Arduino.h>
#include <SdFat.h>
#include "config.h"
#include "ring_buffer.h"

constexpr uint8_t LOG_FORMAT_VERSION = 2;
constexpr size_t LOG_RECORD_BYTES = 64;

struct LogRecord {
    uint8_t bytes[LOG_RECORD_BYTES];
};

// Blocking SD transport. Call service regularly; close before removing power.
// No automatic retries after an I/O error: a short write may leave a partial
// final record, and retrying the whole block would duplicate its prefix.
class SdLog {
public:
    bool begin() {
        if (active || failed || finalized) return false;
        pinMode(SS, OUTPUT);
        digitalWrite(SS, HIGH);
        pinMode(Config::CAN_CS_PIN, OUTPUT);
        digitalWrite(Config::CAN_CS_PIN, HIGH);
        pinMode(Config::SD_CS_PIN, OUTPUT);
        digitalWrite(Config::SD_CS_PIN, HIGH);
        if (!sd.begin(SdSpiConfig(Config::SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(4))))
            return fail();
        for (uint16_t i = 0; i < 10000; ++i) {
            snprintf(filename, sizeof(filename), "LOG%04u.BIN", i);
            if (sd.exists(filename)) continue;
            if (!file.open(filename, O_WRONLY | O_CREAT | O_EXCL)) return fail();
            if (!file.preAllocate(Config::SD_PREALLOCATE_BYTES) || !file.seekSet(0)) {
                file.close();
                return fail();
            }
            active = true;
            lastSyncMs = millis();
            return true;
        }
        return fail();
    }

    bool enqueue(const LogRecord& record) {
        if (!active || failed) return false;
        const bool accepted = queue.push(record);
        if (queue.size() > peakRecords) peakRecords = queue.size();
        return accepted;
    }

    bool hasFullBlock() const {
        return isActive() && queue.size() >= 8;
    }

    bool writeOneBlock() {
        if (!hasFullBlock()) return false;
        return writeRecords(8);
    }

    void writeUntil(uint32_t nextControlUs) {
        while (hasFullBlock()) {
            const int32_t remainingUs =
                static_cast<int32_t>(nextControlUs - micros());

            const int32_t requiredUs =
                Config::SD_WRITE_ESTIMATE_US +
                Config::SD_WRITE_MARGIN_US;

            if (remainingUs <= requiredUs) {
                break;
            }

            if (!writeOneBlock()) {
                break;
            }
        }
    }

    bool service(bool flush = false) {
        if (!active || failed) return false;
        const bool syncDue = flush || millis() - lastSyncMs >= 1000UL;
        while (hasFullBlock()) if (!writeOneBlock()) return false;
        // Partial writes only during final flush: preserve sector alignment
        // throughout normal operation, including the legacy dwell service path.
        if (flush && !queue.empty() && !writeRecords(queue.size())) return false;
        if (syncDue) {
            if (!file.sync()) return fail();
            lastSyncMs = millis();
        }
        return true;
    }

    bool close() {
        if (!active) return !failed;
        const uint32_t startUs = micros();
        const bool flushed = service(true);
        // Retain only complete records, even after a partial failed write.
        const bool trimmed = file.truncate(bytesWritten - bytesWritten % LOG_RECORD_BYTES);
        const bool closed = file.close();
        closeUs = micros() - startUs;
        active = false;
        if (!flushed || !trimmed || !closed) return fail();
        finalized = true;
        return true;
    }

    bool isActive() const { return active && !failed; }
    bool hasError() const { return failed; }
    bool isOpen() const { return active; }
    bool isFinalized() const { return finalized; }
    uint32_t dropped() const { return queue.dropped(); }
    uint16_t buffered() const { return queue.size(); }
    uint16_t peakBuffered() const { return peakRecords; }
    uint32_t writtenBytes() const { return bytesWritten; }
    uint32_t worstBlockUs() const { return maxBlockUs; }
    // Same scope as worstBlockUs: full-block attempts, including final drain.
    // Includes buffer copying; excludes partial writes and sync/close work.
    uint32_t blockWriteCount() const { return blockWrites; }
    // Nearest-rank percentile upper bound, in 512 us buckets. The last
    // bucket (>=65024 us) uses the observed maximum as a conservative bound.
    uint32_t blockPercentileUpperUs(uint8_t percent) const {
        if (!blockWrites || percent == 0 || percent > 100) return 0;
        const uint32_t rank = (blockWrites / 100) * percent +
            ((blockWrites % 100) * percent + 99) / 100;
        uint32_t cumulative = 0;
        for (uint8_t i = 0; i < 128; ++i) {
            cumulative += blockHistogram[i];
            if (cumulative >= rank)
                return i == 127 ? maxBlockUs : (uint32_t(i) + 1) * 512 - 1;
        }
        return maxBlockUs;
    }
    double averageBlockUs() const {
        return blockWrites ? double(totalBlockUs) / blockWrites : 0.0;
    }
    uint32_t finalizationUs() const { return closeUs; }
    const char* name() const { return filename; }

    // Sample the supplied Unix reference at the moment this function is called.
    void setUnixTime(uint64_t unixMs) {
        epochMs = unixMs;
        lastClockMs = millis();
        clockValid = true;
    }
    void updateClock(uint32_t nowMs) {
        if (clockValid) epochMs += uint32_t(nowMs - lastClockMs);
        lastClockMs = nowMs;
    }
    bool hasUnixTime() const { return clockValid; }
    uint64_t unixTime() const { return clockValid ? epochMs : 0; }

private:
    bool writeRecords(uint8_t count) {
        const uint16_t bytes = count * LOG_RECORD_BYTES;
        if (bytes > Config::SD_PREALLOCATE_BYTES - bytesWritten) return fail();
        const uint32_t startUs = micros();
        for (uint8_t i = 0; i < count; ++i) {
            LogRecord record;
            queue.pop(record);
            memcpy(block + i * LOG_RECORD_BYTES, record.bytes, LOG_RECORD_BYTES);
        }
        const size_t written = file.write(block, bytes);
        const uint32_t elapsedUs = micros() - startUs;
        if (count == 8) {
            totalBlockUs += elapsedUs;
            ++blockWrites;
            const uint32_t bucket = elapsedUs / 512;
            ++blockHistogram[bucket < 127 ? bucket : 127];
            if (elapsedUs > maxBlockUs) maxBlockUs = elapsedUs;
        }
        bytesWritten += written;
        if (written != bytes) return fail();
        return true;
    }
    bool fail() { failed = true; return false; }
    SdFat32 sd;
    File32 file;
    RingBuffer<LogRecord, 16> queue;
    uint8_t block[512] = {};
    char filename[13] = {};
    uint32_t lastSyncMs = 0;
    uint32_t lastClockMs = 0;
    uint64_t epochMs = 0;
    uint32_t bytesWritten = 0;
    uint32_t maxBlockUs = 0;
    uint64_t totalBlockUs = 0;
    uint32_t blockWrites = 0;
    uint32_t blockHistogram[128] = {};
    uint32_t closeUs = 0;
    uint16_t peakRecords = 0;
    bool clockValid = false;
    bool active = false;
    bool failed = false;
    bool finalized = false;
};
