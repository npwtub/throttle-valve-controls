#pragma once
#include <Arduino.h>
#include <SdFat.h>
#include <stdio.h>
#include <string.h>
#include "log_config.h"
#include "log_queue.h"

class SdLogWriter {
public:
    enum class ServiceResult { Idle, Busy, WroteData, Error };
    enum class Error : uint8_t {
        None, CardInit, Filename, Open, Reserve, Write, BusyTimeout, Finish
    };
    // nullptr selects the first unused RUN0001.CSV ... RUN9999.CSV.
    // Initialization and reservation can block: run only before motion.
    bool begin(uint8_t cs, const char* requestedName = nullptr) {
        if (file_.isOpen()) return false;
        if (!sd_.begin(SdSpiConfig(cs, SHARED_SPI, SD_SCK_MHZ(4))))
            return fail(Error::CardInit);
        if (requestedName) {
            if (!requestedName[0] || strlen(requestedName) >= sizeof(filename_))
                return fail(Error::Filename);
            strcpy(filename_, requestedName);
        } else {
            bool found = false;
            for (uint16_t number = 1; number <= 9999; ++number) {
                snprintf(filename_, sizeof(filename_), "RUN%04u.CSV", number);
                if (!sd_.exists(filename_)) { found = true; break; }
            }
            if (!found) return fail(Error::Filename);
        }
        if (!file_.open(filename_, O_WRONLY | O_CREAT | O_EXCL))
            return fail(Error::Open);
        if (!file_.preAllocate(LogConfig::reservedBytes) || !file_.sync()) {
            file_.truncate(0);
            file_.close();
            return fail(Error::Reserve);
        }
        return true;
    }
    ServiceResult service(LogQueue& queue, bool drain = false) {
        if (error_ != Error::None || !file_.isOpen()) return ServiceResult::Error;
        if (!drain && queue.size() < 512)
            return ServiceResult::Idle;
        if (sd_.card()->isBusy()) {
            if (!waiting_) { waiting_ = true; busySinceMs_ = millis(); }
            if (static_cast<uint32_t>(millis() - busySinceMs_) >= LogConfig::busyTimeoutMs) {
                fail(Error::BusyTimeout);
                return ServiceResult::Error;
            }
            return ServiceResult::Busy;
        }
        waiting_ = false;
        if (!queue.size()) return ServiceResult::Idle;
        const size_t count = queue.contiguousSize(512);
        if (count > LogConfig::reservedBytes - bytesWritten_) {
            fail(Error::Write);
            return ServiceResult::Error;
        }
        // Preallocation removes cluster growth, but this call can STILL block.
        const uint32_t started = micros();
        const size_t written = file_.write(queue.front(), count);
        const uint32_t duration = static_cast<uint32_t>(micros() - started);
        if (duration > maxWriteUs_) maxWriteUs_ = duration;
        if (written <= count) { queue.consume(written); bytesWritten_ += written; }
        if (written != count || file_.getWriteError()) {
            fail(Error::Write);
            return ServiceResult::Error;
        }
        return ServiceResult::WroteData;
    }
    bool finish(const LogQueue& queue) {
        if (error_ != Error::None || !file_.isOpen() || queue.size()) return false;
        const uint32_t started = micros();
        const bool truncated = file_.truncate();
        const bool closed = file_.close(); // includes sync; can still block
        closeUs_ = static_cast<uint32_t>(micros() - started);
        if (!truncated || !closed) return fail(Error::Finish);
        return true;
    }
    const char* filename() const { return filename_; }
    uint32_t bytesWritten() const { return bytesWritten_; }
    uint32_t maxWriteUs() const { return maxWriteUs_; }
    uint32_t closeUs() const { return closeUs_; }
    Error error() const { return error_; }
private:
    bool fail(Error error) { error_ = error; return false; }
    SdFat32 sd_;
    File32 file_;
    char filename_[13] = {};
    Error error_ = Error::None;
    bool waiting_ = false;
    uint32_t busySinceMs_ = 0;
    uint32_t bytesWritten_ = 0;
    uint32_t maxWriteUs_ = 0;
    uint32_t closeUs_ = 0;
};
