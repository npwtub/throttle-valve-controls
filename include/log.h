/**
 * Shared CSV formatting.
 * - Defaults to serial output.
 * - Can write to another Arduino Print destination.
 * - Keep the same inputs enabled throughout each log file.
 */
#pragma once

#include <Arduino.h>

#include "motor_state.h"
#include "pressure_transducer.h"
#include "config.h"
#include "log_clock.h"
#include "log_time_sync.h"
#include "log_buffer.h"
#include "log_queue.h"
#include "sd_log_writer.h"

template <typename StateText>
static void logRow(
    const MotorState* motorState,
    const PressureTransducer* pt1,
    const PressureTransducer* pt2,
    const unsigned long nowMs,
    const StateText& stateName,
    Print& output = Serial
) {
    output.print(nowMs);
    output.print(',');

    if (pt1 != nullptr) {
        output.print(pt1->getPressure(false), 2);
        output.print(',');
    }

    if (pt2 != nullptr) {
        output.print(pt2->getPressure(false), 2);
        output.print(',');
    }

    if (motorState != nullptr) {
        output.print(
            motorState->getPosition() * 360.0 / Config::GEAR_RATIO,
            3
        );
        output.print(',');

        output.print(motorState->getTemperature(), 1);
        output.print(',');

        output.print(motorState->getAppliedOutputPercent(), 2);
        output.print(',');

        output.print(motorState->getRPM(), 2);
        output.print(',');

        output.print(motorState->getVoltageRaw() / 136.5);
        output.print(',');

        output.print(motorState->getCurrentRaw() / 27.3);
        output.print(',');

        output.print(motorState->hasStatus0());
        output.print(',');

        output.print(motorState->hasStatus2());
        output.print(',');

        output.print(motorState->hasTelemetry());
        output.print(',');

        output.print(
            motorState->isFresh(Config::TELEMETRY_TIMEOUT_MS)
        );
        output.print(',');

        output.print(motorState->getLastStatus0Time());
        output.print(',');

        output.print(motorState->getLastStatus2Time());
        output.print(',');
    }

    output.println(stateName);
}

static void printLogHeader(
    const MotorState* motorState,
    const PressureTransducer* pt1,
    const PressureTransducer* pt2,
    Print& output = Serial
) {
    output.print(F("timestamp_ms,"));

    if (pt1 != nullptr) {
        output.print(F("pt1_psi,"));
    }

    if (pt2 != nullptr) {
        output.print(F("pt2_psi,"));
    }

    if (motorState != nullptr) {
        output.print(F(
            "position_deg,motor_temp_c,output_percent,rpm,"
            "voltage,current,has_status_0,has_status_2,"
            "has_telemetry,is_fresh,"
            "last_status_0_time,last_status_2_time,"
        ));
    }

    output.println(F("state"));
}
class SdCsvLogger {
public:
    enum class Status {
        NotStarted,
        Recording,
        Stopping,
        Closed,
        Failed
    };

    explicit SdCsvLogger(LogClock& clock) : clock_(clock) {}

    // Call after UTC synchronization and before starting the test.
    // Pass nullptr to choose an unused filename automatically, or provide
    // an explicit filename that does not already exist.
    bool begin(
        uint8_t chipSelectPin,
        const char* filename,
        const MotorState* motor,
        const PressureTransducer* pt1,
        const PressureTransducer* pt2
    ) {
        if (status_ != Status::NotStarted ||
            !clock_.isSynchronized()) {
            return false;
        }

        motor_ = motor;
        pt1_ = pt1;
        pt2_ = pt2;

        // Build the complete header in memory first.
        row_.reset();
        row_.print(F("timestamp_unix_ms,sequence,"));
        printLogHeader(motor_, pt1_, pt2_, row_);

        if (row_.overflowed()) {
            status_ = Status::Failed;
            return false;
        }

        if (!writer_.begin(chipSelectPin, filename)) {
            status_ = Status::Failed;
            return false;
        }

        if (!queue_.enqueue(row_.data(), row_.size())) {
            status_ = Status::Failed;
            return false;
        }

        acceptedBytes_ = row_.size();
        peakQueuedBytes_ = queue_.size();
        status_ = Status::Recording;
        return true;
    }

    // Capture one record when the test's scheduler requests it.
    // This formats into RAM; it does not write to the SD card.
    template <typename StateText>
    bool capture(const StateText& stateName) {
        if (status_ != Status::Recording) {
            return false;
        }

        const uint32_t sequence = nextSequence_++;

        uint64_t utcMs = 0;
        if (!clock_.getUtcMs(utcMs)) {
            status_ = Status::Failed;
            return false;
        }

        const unsigned long capturedMs = millis();

        row_.reset();
        printUint64(row_, utcMs);
        row_.print(',');
        row_.print(sequence);
        row_.print(',');

        logRow(
            motor_,
            pt1_,
            pt2_,
            capturedMs,
            stateName,
            row_
        );

        if (row_.overflowed()) {
            ++oversizedRecords_;
            return false;
        }

        if (row_.size() > LogConfig::reservedBytes - acceptedBytes_) {
            capacityReached_ = true;
            requestStop();
            return false;
        }
        if (!queue_.enqueue(row_.data(), row_.size())) {
            ++queueFullRecords_;
            return false;
        }

        acceptedBytes_ += row_.size();
        ++queuedRecords_;

        if (queue_.size() > peakQueuedBytes_) {
            peakQueuedBytes_ = queue_.size();
        }

        return true;
    }

    // Call from the main loop after control/communication work.
    // At most one queued write is attempted per call.
    void service() {
        clock_.update();

        if (status_ != Status::Recording &&
            status_ != Status::Stopping) {
            return;
        }

        const bool draining = status_ == Status::Stopping;
        const SdLogWriter::ServiceResult result =
            writer_.service(queue_, draining);

        if (result == SdLogWriter::ServiceResult::Error) {
            status_ = Status::Failed;
            return;
        }

        // Finish on a later pass, after the last write.
        if (draining &&
            result == SdLogWriter::ServiceResult::Idle &&
            queue_.size() == 0) {
            status_ = writer_.finish(queue_)
                ? Status::Closed
                : Status::Failed;
        }
    }

    // Stop accepting records. Continue calling service()
    // until status() becomes Closed or Failed.
    void requestStop() {
        if (status_ == Status::Recording) {
            status_ = Status::Stopping;
        }
    }

    Status status() const {
        return status_;
    }

    uint32_t queuedRecords() const {
        return queuedRecords_;
    }

    uint32_t oversizedRecords() const {
        return oversizedRecords_;
    }

    uint32_t queueFullRecords() const {
        return queueFullRecords_;
    }

    size_t queuedBytes() const {
        return queue_.size();
    }

    size_t peakQueuedBytes() const {
        return peakQueuedBytes_;
    }

    uint32_t maxWriteUs() const {
        return writer_.maxWriteUs();
    }

    const char* filename() const { return writer_.filename(); }
    uint32_t bytesWritten() const { return writer_.bytesWritten(); }
    uint32_t closeUs() const { return writer_.closeUs(); }
    uint8_t errorCode() const { return static_cast<uint8_t>(writer_.error()); }
    bool capacityReached() const { return capacityReached_; }

private:
    // AVR Arduino Print does not directly support uint64_t.
    static void printUint64(Print& output, uint64_t value) {
        char digits[20];
        uint8_t count = 0;

        do {
            digits[count++] =
                static_cast<char>('0' + value % 10);
            value /= 10;
        } while (value != 0);

        while (count > 0) {
            output.write(
                static_cast<uint8_t>(digits[--count])
            );
        }
    }

    LogClock& clock_;
    LogRecordBuffer row_;
    LogQueue queue_;
    SdLogWriter writer_;

    const MotorState* motor_ = nullptr;
    const PressureTransducer* pt1_ = nullptr;
    const PressureTransducer* pt2_ = nullptr;

    Status status_ = Status::NotStarted;
    uint32_t nextSequence_ = 0;
    uint32_t queuedRecords_ = 0;
    uint32_t oversizedRecords_ = 0;
    uint32_t queueFullRecords_ = 0;
    size_t peakQueuedBytes_ = 0;
    uint32_t acceptedBytes_ = 0;
    bool capacityReached_ = false;
};
