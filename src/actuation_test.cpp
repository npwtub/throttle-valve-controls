/**
 * Repeated valve actuation test.
 * - Starts when a space arrives over Serial.
 * - Runs five opening and closing cycles.
 * - Holds open for five seconds per cycle.
 * - Commands closed after completion or a travel timeout.
 * - Calculates and submits control + heartbeat frames every 5 ms.
 * - Queues telemetry for SD; no periodic Serial diagnostics.
 */
#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <mcp2515.h>

#include "config.h"
#include "control_timing_tail.h"
#include "motor_state.h"
#include "log.h"

// MCP2515 chip select pin
MCP2515 mcp2515(Config::kCanCsPin, 4000000UL);

// Spark MAX CAN ID = 1
CanControl::SparkMax motor(mcp2515, Config::kLoxMotorId);

MotorState motorState(Config::kLoxMotorId);

const double delayTimeMs = 1000;
const int timeOpenMs = 5000;
const int actuationIntervalMs = 5000;
const int travelTimeoutMs = 1000;
const int numActuations = 5;
constexpr unsigned long timingWindowPeriodMs = 1000;

struct SendTiming {
    uint64_t totalUs = 0;
    unsigned long cycles = 0;
    unsigned long worstUs = 0;
    unsigned long lateSends = 0;
    unsigned long controlErrors = 0;
    unsigned long heartbeatErrors = 0;
    unsigned long missedSends = 0;
    unsigned long previousCommandUs = 0;
    unsigned long worstCommandSpacingUs = 0;
};

SendTiming sendTiming;
ControlTimingTail timingTail;

// Start-to-start timing for the commanded test only, excluding WAITING and
// final SD closure/reporting. Frequency uses intervals, not endpoint count.
struct RunTiming {
    bool started = false;
    bool reported = false;
    uint32_t cycles = 0;
    uint32_t firstUs = 0;
    uint32_t previousUs = 0;
    uint32_t worstSpacingUs = 0;
    uint32_t missedBaseline = 0;
    uint32_t droppedBaseline = 0;
    uint32_t controlOver5Ms = 0;
    uint32_t beforeSdOver5Ms = 0;
    uint32_t withSdOver5Ms = 0;
    uint32_t withoutSdOver5Ms = 0;
};
RunTiming runTiming;

enum TestState {
    WAITING,
    CLOSED,
    OPENING,
    OPEN,
    CLOSING,
    DONE,
    TIMEOUT
};

TestState testState = WAITING;

const __FlashStringHelper* state_name() {
    switch (testState) {
        case WAITING: return F("waiting");
        case CLOSED:  return F("closed");
        case OPENING: return F("opening");
        case OPEN:    return F("open");
        case CLOSING: return F("closing");
        case DONE:    return F("done");
        case TIMEOUT: return F("timeout");
    }
    return F("unknown");
}

void setup() {
    Serial.begin(Config::kBaudRateLong);

    // Initialize MCP2515
    pinMode(Config::kSdCsPin, OUTPUT);
    digitalWrite(Config::kSdCsPin, HIGH);
    mcp2515.reset();

    // Spark MAX CAN bus runs at 1 Mbps.
    // Change MCP_8MHZ if your MCP2515 module has a different crystal.
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);

    mcp2515.setNormalOneShotMode();

    delay(delayTimeMs);
    begin_sd_log();
    wait_for_unix_time();
    begin_control_schedule();
}

void loop() {
    begin_control_cycle();
    const unsigned long cycleStartUs = micros();
    static int actuationCount = 0;
    static unsigned long lastActuation = 0;
    static unsigned long lastTimingWindow = millis();

    // Bound receive work so continuous traffic cannot starve outgoing frames.
    struct can_frame frame;
    for (uint8_t count = 0; count < 8 &&
                            mcp2515.readMessage(&frame) == MCP2515::ERROR_OK;
         ++count) {
        motor.handle_received_frame(frame);
        motorState.process_frame(frame);
    }

    const unsigned long now = millis();

    switch (testState) {
        case WAITING:
            // Poll without blocking so the 5 ms control cycle continues.
            // A single space starts the test; no newline is required.
            if (Serial.available() > 0 && Serial.read() == ' ') {
                runTiming.started = true;
                runTiming.firstUs = cycleStartUs;
                runTiming.previousUs = cycleStartUs;
                runTiming.missedBaseline = controlSchedule.missedSlots;
                runTiming.droppedBaseline = sdLog.dropped();
                sendTiming = SendTiming{};
                lastActuation = now;  // Begin the existing initial closed interval.
                testState = CLOSED;
            }
            break;
        case CLOSED:
            // if actuation interval has passed, open the valve
            if ((now - lastActuation) > actuationIntervalMs) {
                actuationCount++;
                testState = OPENING;
                lastActuation = now;
            }
            break;
        case OPENING:
            // At the goal position, start the open hold interval.
            if (Config::kOpenRotations - motorState.get_position() < 0.15) {
                testState = OPEN;

                lastActuation = now;
            } else if (now - lastActuation > travelTimeoutMs) {
                testState = TIMEOUT;
            }
            break;
        case OPEN:
            // if actuation time has passed, close the valve
            if ((now - lastActuation) > timeOpenMs) {
                testState = CLOSING;
                lastActuation = now;
            }
            break;
        case CLOSING:
            // At the closed position, complete the test or start the next interval.
            if ((motorState.get_position() < Config::kGoalPositionTolerance)) {
                lastActuation = now;
                if (actuationCount >= numActuations) {
                    testState = DONE;  // test is done
                } else {
                    testState = CLOSED;  // test is not yet done and valve will open again
                }
            } else if (now - lastActuation > travelTimeoutMs) {
                testState = TIMEOUT;
            }
            break;
        case DONE:
        case TIMEOUT:
            break;
    }

    // Submit both frames immediately after calculations, exactly once per cycle.
    // Waiting, completed, and timed-out tests all command the closed position.
    const bool openingOrOpen = testState == OPENING || testState == OPEN;
    const float target = openingOrOpen ? Config::kOpenRotations : Config::kClosedRotations;
    const unsigned long commandStartUs = micros();
    if (sendTiming.cycles != 0) {
        const unsigned long spacingUs = commandStartUs - sendTiming.previousCommandUs;
        if (spacingUs > sendTiming.worstCommandSpacingUs)
            sendTiming.worstCommandSpacingUs = spacingUs;
    }
    sendTiming.previousCommandUs = commandStartUs;
    const MCP2515::ERROR controlResult = motor.set_position(target, 0, 0, 0);
    const MCP2515::ERROR heartbeatResult = CanControl::send_heartbeat(
        mcp2515, CanControl::default_heartbeat());

    // Time includes CAN reception, calculations, and BOTH send calls, even on
    // failures/overruns. The driver queues frames; this is not on-wire latency.
    const unsigned long sendTimeUs = micros() - cycleStartUs;
    timingTail.record(sendTimeUs);
    ++sendTiming.cycles;
    sendTiming.totalUs += sendTimeUs;
    if (sendTimeUs > sendTiming.worstUs) sendTiming.worstUs = sendTimeUs;
    const bool late = sendTimeUs > Config::kControlPeriodUs;
    const bool controlFailed = controlResult != MCP2515::ERROR_OK;
    const bool heartbeatFailed = heartbeatResult != MCP2515::ERROR_OK;
    if (late) ++sendTiming.lateSends;
    if (controlFailed) ++sendTiming.controlErrors;
    if (heartbeatFailed) ++sendTiming.heartbeatErrors;
    // A failed frame misses this cycle even when the driver returns promptly.
    if (late || controlFailed || heartbeatFailed) ++sendTiming.missedSends;
    const bool testFinished = testState == DONE || testState == TIMEOUT;
    const bool reportRun = runTiming.started && !runTiming.reported && testFinished;
    if (runTiming.started && !runTiming.reported) {
        const uint32_t spacingUs = cycleStartUs - runTiming.previousUs;
        if (spacingUs > runTiming.worstSpacingUs)
            runTiming.worstSpacingUs = spacingUs;
        runTiming.previousUs = cycleStartUs;
        ++runTiming.cycles;
    }
    // Snapshot before finalRow causes blocking shutdown work.
    const uint32_t runElapsedUs = cycleStartUs - runTiming.firstUs;
    const uint32_t runMissedSlots = controlSchedule.missedSlots - runTiming.missedBaseline;
    log_row(&motorState, nullptr, nullptr, millis(), state_name(), (target / 32.0017) * 360, testFinished);
    // Keep percentile windows bounded without formatting or sending reports.
    if (now - lastTimingWindow >= timingWindowPeriodMs) {
        lastTimingWindow = now;
        timingTail.start_next_interval(timingTail.percentile_bucket(95),
                                     timingTail.percentile_bucket(99));
    }

    // Use spare time for SD blocks, then account for any missed schedule slots.
    const uint32_t beforeSdUs = micros() - cycleStartUs;
    const uint32_t blocksBefore = sdLog.block_write_count();
    finish_control_cycle();
    const uint32_t fullCycleUs = micros() - cycleStartUs;
    // Mutually exclusive categories of active non-final cycles >5 ms.
    // These indicate where the threshold was crossed, not exclusive causation.
    if (runTiming.started && !runTiming.reported && !testFinished &&
        fullCycleUs > Config::kControlPeriodUs) {
        if (sendTimeUs > Config::kControlPeriodUs) ++runTiming.controlOver5Ms;
        else if (beforeSdUs > Config::kControlPeriodUs) ++runTiming.beforeSdOver5Ms;
        else if (sdLog.block_write_count() > blocksBefore) ++runTiming.withSdOver5Ms;
        else ++runTiming.withoutSdOver5Ms;
    }
    if (reportRun) {
        runTiming.reported = true;
        const double achievedHz = runElapsedUs && runTiming.cycles > 1
            ? double(runTiming.cycles - 1) * 1000000.0 / runElapsedUs : 0.0;
        Serial.print(F("RUN: state=")); Serial.print(state_name());
        Serial.print(F(" cycles=")); Serial.print(runTiming.cycles);
        Serial.print(F(" elapsed_ms=")); Serial.print(runElapsedUs / 1000UL);
        Serial.print(F(" control_hz=")); Serial.print(achievedHz, 2);
        Serial.print(F(" spacing_worst_us=")); Serial.print(runTiming.worstSpacingUs);
        Serial.print(F(" missed_slots=")); Serial.print(runMissedSlots);
        Serial.print(F(" dropped_records="));
        Serial.print(sdLog.dropped() - runTiming.droppedBaseline);
        Serial.print(F(" control_errors=")); Serial.print(sendTiming.controlErrors);
        Serial.print(F(" heartbeat_errors=")); Serial.print(sendTiming.heartbeatErrors);
        Serial.print(F(" write_allowance_us="));
        Serial.println(Config::kSdWriteEstimateUs + Config::kSdWriteMarginUs);
        report_log_status(Serial);
        Serial.print(F("OVER_5MS: control=")); Serial.print(runTiming.controlOver5Ms);
        Serial.print(F(" pre_sd_logging=")); Serial.print(runTiming.beforeSdOver5Ms);
        Serial.print(F(" service_with_write=")); Serial.print(runTiming.withSdOver5Ms);
        Serial.print(F(" service_without_write=")); Serial.println(runTiming.withoutSdOver5Ms);
    }
}
