#pragma once

#include <Arduino.h>
#include <avr/pgmspace.h>

class LogStatus {
public:
    enum class Code : uint8_t {
        Boot,
        TimeOK,
        TimeInvalid,
        TimeAlreadySet,
        NeedTime,
        InvalidSdPin,
        SdInitFailed,
        TestStarted,
        SdWriteFailed,
        LogStopRequested,
        LogClosed
    };

    bool push(Code code) {
        if (count_ == CAPACITY) {
            ++dropped_;
            return false;
        }

        queue_[writeIndex_] = code;
        writeIndex_ = (writeIndex_ + 1) % CAPACITY;
        ++count_;
        return true;
    }

    // Send at most one complete message per call.
    // Wait until another loop pass if serial space is insufficient.
    void service() {
        if (count_ == 0) {
            return;
        }

        const __FlashStringHelper* message =
            textFor(queue_[readIndex_]);

        PGM_P text = reinterpret_cast<PGM_P>(message);
        const size_t length = strlen_P(text);

        if (Serial.availableForWrite() < static_cast<int>(length)) {
            return;
        }

        for (size_t index = 0; index < length; ++index) {
            Serial.write(
                static_cast<uint8_t>(pgm_read_byte(text + index))
            );
        }

        readIndex_ = (readIndex_ + 1) % CAPACITY;
        --count_;
    }

    uint32_t droppedMessages() const {
        return dropped_;
    }

private:
    static const __FlashStringHelper* textFor(Code code) {
        switch (code) {
            case Code::Boot:
                return F("# BOOT_WAITING_FOR_TIME\n");
            case Code::TimeOK:
                return F("# TIME_SYNC_OK\n");
            case Code::TimeInvalid:
                return F("# TIME_SYNC_INVALID\n");
            case Code::TimeAlreadySet:
                return F("# TIME_ALREADY_SET\n");
            case Code::NeedTime:
                return F("# START_BLOCKED_NO_TIME\n");
            case Code::InvalidSdPin:
                return F("# START_BLOCKED_SD_PIN\n");
            case Code::SdInitFailed:
                return F("# SD_INIT_FAILED_RESET_REQUIRED\n");
            case Code::TestStarted:
                return F("# TEST_STARTED\n");
            case Code::SdWriteFailed:
                return F("# SD_WRITE_FAILED\n");
            case Code::LogStopRequested:
                return F("# LOG_STOP_REQUESTED\n");
            case Code::LogClosed:
                return F("# LOG_CLOSED\n");
        }

        return F("# UNKNOWN_STATUS\n");
    }

    static constexpr uint8_t CAPACITY = 8;

    Code queue_[CAPACITY] = {};
    uint8_t readIndex_ = 0;
    uint8_t writeIndex_ = 0;
    uint8_t count_ = 0;
    uint32_t dropped_ = 0;
};