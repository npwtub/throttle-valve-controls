/**
 * Binary SD logger (blocking writes).
 * - Queues fixed-size telemetry records and writes them to an SD file.
 * - Stores encoder position as valve-equivalent degrees and the valve target
 *   before backlash compensation; the applied compensation has a separate flag.
 * - Skips sensors and motor data when their pointers are null.
 * - Serial is used only for logger status and errors.
 */
#pragma once

#include <Arduino.h>
#include "motor_state.h"
#include "pressure_transducer.h"
#include "pid.h"
#include "config.h"
#include "sd_log.h"
#include <math.h>

#include <stdint.h>
#include <string.h>

static inline void put_u32(uint8_t* dst, uint32_t value) {
    for (uint8_t i = 0; i < 4; ++i) {
        dst[i] = uint8_t(value >> (8 * i));
    }
}

static inline void put_u64(uint8_t* dst, uint64_t value) {
    for (uint8_t i = 0; i < 8; ++i) {
        dst[i] = uint8_t(value >> (8 * i));
    }
}

static inline void put_float(uint8_t* dst, float value) {
    static_assert(sizeof(float) == 4, "Requires 4-byte floats");
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put_u32(dst, bits);
}

// Round to whole units and saturate without integer wraparound. The lowest
// signed byte and highest unsigned byte are reserved for nonfinite samples.
static inline uint8_t log_signed_unit(float value, int8_t minimum, int8_t maximum) {
    if (!isfinite(value)) return 0x80;
    if (value <= minimum) return static_cast<uint8_t>(minimum);
    if (value >= maximum) return static_cast<uint8_t>(maximum);
    return static_cast<uint8_t>(static_cast<int8_t>(
        value + (value >= 0.0f ? 0.5f : -0.5f)));
}

static inline uint8_t log_current_amps(float value) {
    if (!isfinite(value)) return 0xff;
    if (value <= 0.0f) return 0;
    if (value >= 254.0f) return 254;
    return static_cast<uint8_t>(value + 0.5f);
}

struct LogControlData {
    // Optional exact sample already used by PID; avoids another ADC conversion.
    const float* pt3PressurePsi = nullptr;
    bool pidOutputClamped = false;
};

// commandedMotorRotations is the desired valve target expressed in motor
// rotations BEFORE adding backlash. backlashCompensated describes the offset
// applied to this cycle's motor command, not verified motion or CAN delivery.
static inline void make_log_record(
    uint8_t (&out)[logRecordBytes],
    const MotorState* motorState,
    const PressureTransducer* pt1,
    const PressureTransducer* pt2,
    const PressureTransducer* pt3,
    uint64_t unixTimeMs,
    uint8_t stateId,
    float commandedMotorRotations,
    uint32_t droppedRecords,
    bool timeValid = false,
    float commandedThrustLbf = 0.0f,
    bool backlashCompensated = false,
    const PidController* pidController = nullptr,
    const LogControlData* controlData = nullptr
) {
    memset(out, 0, sizeof(out));

    out[0] = logFormatVersion;
    out[1] = stateId;  // Numeric state instead of a string

    const bool hasPidOutput = pidController && pidController->has_last_output();
    // Presence flags: absent fields stay zero, but are marked absent.
    out[2] = (pt1 ? 1 : 0)
           | (pt2 ? 2 : 0)
           | (motorState ? 4 : 0)
           | (timeValid ? 8 : 0)
           | (backlashCompensated ? 16 : 0)
           | (pt3 ? 32 : 0)
           | (hasPidOutput ? 64 : 0)
           | (hasPidOutput && controlData && controlData->pidOutputClamped ? 128 : 0);

    put_u64(out + 4, unixTimeMs);

    if (pt1) put_float(out + 12, pt1->get_pressure(false));
    if (pt2) put_float(out + 16, pt2->get_pressure(false));
    if (pt3) put_float(out + 20,
        controlData && controlData->pt3PressurePsi
            ? *controlData->pt3PressurePsi : pt3->get_pressure(false));

    if (motorState) {
        out[3] = (motorState->has_status_0() ? 1 : 0)
               | (motorState->has_status_2() ? 2 : 0)
               | (motorState->has_telemetry() ? 4 : 0)
               | (motorState->is_fresh(
                      Config::kTelemetryTimeoutMs) ? 8 : 0);

        put_float(out + 24,
            motorState->get_position() * 360.0f / Config::kGearRatio);
        out[28] = log_signed_unit(motorState->get_temperature(), -127, 127);
        out[29] = log_signed_unit(motorState->get_applied_output_percent(), -100, 100);
        out[30] = log_current_amps(motorState->get_current_raw() / 27.3f);
        // Byte 31 stays free and zero.
        put_float(out + 32, motorState->get_rpm());
        put_float(out + 36, motorState->get_voltage_raw() / 136.5f);
    }

    put_float(out + 40,
        commandedMotorRotations * 360.0f / Config::kGearRatio);
    put_u32(out + 44, droppedRecords);

    put_float(out + 48, commandedThrustLbf);
    if (hasPidOutput) {
        put_float(out + 52, pidController->last_p_output() * 360.0f / Config::kGearRatio);
        put_float(out + 56, pidController->last_i_output() * 360.0f / Config::kGearRatio);
        put_float(out + 60, pidController->last_d_output() * 360.0f / Config::kGearRatio);
    }
}

// One logger per firmware translation unit (each environment builds one source).
static SdLog sdLog;

/**
 * Contains times related to control scheduling including current cycle start time,
 * next cycle start time, number of overruns, etc
 */
struct ControlSchedule {
    // Scheduled start time of the next cycle.
    uint32_t nextStartUs = 0;
    // cycles whose work and SD service run past the next scheduled start
    uint32_t overruns = 0;
    // total scheduled cycle starts skipped to catch up after overruns
    uint32_t missedSlots = 0;
    // actual start time of the most recent cycle
    uint32_t previousStartUs = 0;
    // largest gap between consecutive actual cycle starts
    uint32_t worstSpacingUs = 0;
    // whether a previous cycle start exists for measuring the next gap
    bool hasPreviousStart = false;
};

static ControlSchedule controlSchedule;

/**
 * Initializes ControlSchedule struct
 */
static inline void begin_control_schedule() {
    controlSchedule = ControlSchedule{};
    controlSchedule.nextStartUs = micros();
}

/**
 * Waits until scheduled control cycle start time,
 * then updates controlSchedule struct
 */
static inline void begin_control_cycle() {
    while (static_cast<int32_t>(
        micros() - controlSchedule.nextStartUs) < 0) {
    }

    controlSchedule.nextStartUs += Config::kControlPeriodUs;
    const uint32_t actualStartUs = micros();
    if (controlSchedule.hasPreviousStart) {
        const uint32_t spacing = actualStartUs - controlSchedule.previousStartUs;
        if (spacing > controlSchedule.worstSpacingUs)
            controlSchedule.worstSpacingUs = spacing;
    }
    controlSchedule.previousStartUs = actualStartUs;
    controlSchedule.hasPreviousStart = true;
}

/**
 * writes to SD card until cycle runs out of time and
 * tracks if cycle ran late
 */
static inline void finish_control_cycle() {
    sdLog.write_until(controlSchedule.nextStartUs);

    const int32_t lateUs = static_cast<int32_t>(
        micros() - controlSchedule.nextStartUs
    );

    if (lateUs > 0) {
        ++controlSchedule.overruns;

        const uint32_t skipped =
            (static_cast<uint32_t>(lateUs) +
             Config::kControlPeriodUs - 1) /
            Config::kControlPeriodUs;

        // skips a step if it's time slot has been breached by the previous cycle
        // this keeps the schedule from shifting
        controlSchedule.missedSlots += skipped;
        controlSchedule.nextStartUs +=
            skipped * Config::kControlPeriodUs;
    }
}

// Stable IDs shared by all tests, independent of their enum numbering.
static inline uint8_t log_state_id(const String& name) {
    if (name == F("waiting")) return 0;
    if (name == F("closed")) return 1;
    if (name == F("opening")) return 2;
    if (name == F("open")) return 3;
    if (name == F("closing")) return 4;
    if (name == F("motor froze")) return 5;
    if (name == F("initial_opening")) return 6;
    if (name == F("initial_open")) return 7;
    if (name == F("soaking")) return 8;
    if (name == F("cycle_opening")) return 9;
    if (name == F("cycle_open")) return 10;
    if (name == F("cycle_closing")) return 11;
    if (name == F("cycle_closed")) return 12;
    if (name == F("done")) return 13;
    if (name == F("timeout")) return 14;
    if (name == F("feedforward")) return 15;
    if (name == F("failed")) return 16;
    if (name == F("moving_up")) return 17;
    if (name == F("moving_down")) return 18;
    if (name == F("at_setpoint_up")) return 19;
    if (name == F("at_setpoint_down")) return 20;
    if (name == F("finished")) return 21;
    if (name == F("travel_timeout")) return 22;
    return 255;
}

static inline void report_sd_error() {
    static bool reported = false;
    if (sdLog.has_error() && !reported) {
        Serial.println(F("SD_ERROR: telemetry is not being saved."));
        reported = true;
    }
}

static inline bool begin_sd_log() {
    if (!sdLog.begin()) { report_sd_error(); return false; }
    Serial.print(F("SD log: "));
    Serial.println(sdLog.name());
    return true;
}

// Setup only: receive a 13-digit Unix timestamp in milliseconds, then newline.
// Control scheduling starts after this function returns.
static inline void wait_for_unix_time() {
    uint64_t timestamp = 0;
    uint8_t digits = 0;
    bool invalid = false;
    uint32_t lastPrompt = millis();

    Serial.println(F("READY_FOR_TIME"));

    while (!sdLog.has_unix_time()) {
        if (millis() - lastPrompt >= 10000UL) {
            Serial.println(F("READY_FOR_TIME"));
            lastPrompt = millis();
        }

        if (!Serial.available()) continue;

        const char c = Serial.read();
        if (c == '\r') continue;

        if (c == '\n') {
            if (!invalid && digits == 13 &&
                timestamp >= 1000000000000ULL &&
                timestamp < 4102444800000ULL) {
                sdLog.set_unix_time(timestamp);
                Serial.println(F("TIME_SET"));
            } else {
                Serial.println(F("INVALID_TIME"));
            }

            timestamp = 0;
            digits = 0;
            invalid = false;
        } else if (!invalid && c >= '0' && c <= '9' &&
                   digits < 13) {
            timestamp = timestamp * 10 + (c - '0');
            ++digits;
        } else {
            invalid = true;
        }
    }
}

// Use the same pre-backlash target and applied-offset flag as the motor command.
// PID getters copy the cached terms; this never advances the controller.
static inline bool log_row(const MotorState* motorState,
                   const PressureTransducer* pt1,
                   const PressureTransducer* pt2,
                   const PressureTransducer* pt3,
                   unsigned long nowMs, const String& stateName,
                   float commandedMotorRotations, bool finalRow = false,
                   bool backlashCompensated = false,
                   float commandedThrustLbf = 0.0f,
                   const PidController* pidController = nullptr,
                   const LogControlData* controlData = nullptr) {
    sdLog.update_clock(nowMs);
    if (!sdLog.is_active()) {
        if (finalRow && sdLog.is_open()) sdLog.close();
        return false;
    }
    // At test completion, make room so a full queue cannot drop the final state.
    // Finalization is deliberately blocking and has no time-budget check.
    if (finalRow && sdLog.has_full_block() && !sdLog.write_one_block()) {
        sdLog.close();
        report_sd_error();
        return false;
    }
    LogRecord record;
    make_log_record(record.bytes, motorState, pt1, pt2, pt3, sdLog.unix_time(),
                  log_state_id(stateName), commandedMotorRotations, sdLog.dropped(),
                  sdLog.has_unix_time(), commandedThrustLbf, backlashCompensated,
                  pidController, controlData);
    const bool queued = sdLog.enqueue(record);
    if (finalRow) {
        const bool closed = sdLog.close();
        report_sd_error();
        return queued && closed;
    }
    return queued;
}

static inline void service_sd_log() {
    sdLog.update_clock(millis());
    if (sdLog.is_active()) sdLog.service();
    report_sd_error();
}

static inline bool close_sd_log() {
    const bool ok = sdLog.close();
    report_sd_error();
    return ok;
}

// Used by the standalone SD bench test after sampling finishes.
static inline void report_log_status(Print& out) {
    out.print(F("SD: buffered=")); out.print(sdLog.buffered());
    out.print(F(" peak=")); out.print(sdLog.peak_buffered());
    out.print(F(" dropped=")); out.print(sdLog.dropped());
    out.print(F(" bytes=")); out.print(sdLog.written_bytes());
    out.print(F(" block_worst_us=")); out.print(sdLog.worst_block_us());
    out.print(F(" block_avg_us=")); out.print(sdLog.average_block_us(), 1);
    out.print(F(" block_writes=")); out.print(sdLog.block_write_count());
    out.print(F(" block_p95_upper_us=")); out.print(sdLog.block_percentile_upper_us(95));
    out.print(F(" block_p99_upper_us=")); out.print(sdLog.block_percentile_upper_us(99));
    out.print(F(" close_us=")); out.print(sdLog.finalization_us());
    out.print(F(" spacing_worst_us=")); out.print(controlSchedule.worstSpacingUs);
    out.print(F(" sd_error=")); out.print(sdLog.has_error() ? 1 : 0);
    out.print(F(" sd_closed=")); out.println(sdLog.is_finalized() ? 1 : 0);
}

static void printLogHeader(const MotorState* motorState, const PressureTransducer* pt1, const PressureTransducer* pt2){
    Serial.print("timestamp_ms,");

    if (pt1 != nullptr) {
        Serial.print("pt1_psi,");
    }
    if (pt2 != nullptr) {
        Serial.print("pt2_psi,");
    }

    if (motorState != nullptr) {
        Serial.print("position_deg,motor_temp_c,output_percent,rpm,voltage,current,has_status_0,has_status_2,has_telemetry,is_fresh,last_status_0_time,last_status_2_time");
    }
    Serial.println("state");
}
