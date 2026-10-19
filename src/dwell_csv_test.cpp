/**
 * Repeated valve dwell test with CSV logging.
 * - Starts when a space arrives over Serial.
 * - Opens every three minutes and holds open for 350 ms.
 * - Starts the dwell after fresh feedback confirms the open position.
 * - Logs pressure and motor telemetry at a requested 100 Hz; actual timing is measured.
 * - Stops cycling on a travel timeout and keeps commanding closed.
 */
#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <mcp2515.h>
#include "log_clock.h"
#include "log_time_sync.h"
#include "config.h"
#include "motor_state.h"
#include "pressure_transducer.h"
#include "log.h"
#include "log_status.h"
#include "log_diagnostics.h"

namespace {
MCP2515 mcp2515(Config::CAN_CS_PIN, 4000000UL);
CanControl::SparkMax motor(mcp2515, Config::LOX_MOTOR_ID);
MotorState motorState(Config::LOX_MOTOR_ID);
PressureTransducer pt1 = PressureTransducer(A2, 1000, 0.47, 4.5);
PressureTransducer pt2 = PressureTransducer(A3, 1000, 0.47, 4.5);

// Preserve the existing 50 Hz motor communication schedule.
// Confirm the separate 200 Hz requirement with the lead before changing it.
constexpr unsigned long controlIntervalMs = 20;
constexpr unsigned long cycleIntervalMs = 180000;
constexpr unsigned long timeOpenMs = 350;
constexpr unsigned long travelTimeoutMs = 2000;
constexpr int SD_CS_PIN = LogConfig::chipSelectPin;
constexpr unsigned long logIntervalMs = LogConfig::intervalMs;

unsigned long missedLogSlots = 0;

// State machine. test state determines the next action of the motor
enum class State { Waiting,
                   Closed,
                   Opening,
                   Open,
                   Closing,
                   Frozen };

State state = State::Waiting;

LogClock logClock;
LogTimeSync timeSync(logClock);
SdCsvLogger logger(logClock);
LogStatus statusMessages;
LogTiming timing;
LogDiagnostics diagnostics;
bool reportRequested = false;

SdCsvLogger::Status lastReportedLogStatus =
    SdCsvLogger::Status::NotStarted;

bool receivingSerialLine = false;
unsigned long cycleStart = 0;
unsigned long stateStart = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastCommand = 0;
unsigned long lastLog = 0;
bool positionReceivedDuringTravel = false;

// sets position to motor
void commandPosition(double target, unsigned long now) {
    timing.command(micros());
    motor.set_position(target, 0, 0, 0);
    lastCommand = now;
}

// begins to  move motor to the next state setpoint
void beginTravel(State next, double target, unsigned long now) {
    state = next;
    stateStart = now;
    // Require a new encoder report before accepting the target as reached.
    positionReceivedDuringTravel = false;
    commandPosition(target, now);
}

const __FlashStringHelper* stateName() {
    switch (state) {
        case State::Waiting:
            return F("waiting");
        case State::Closed:
            return F("closed");
        case State::Opening:
            return F("opening");
        case State::Open:
            return F("open");
        case State::Closing:
            return F("closing");
        case State::Frozen:
            return F("motor froze");
    }
    return F("unknown");
}

void captureRecord() {
    const uint32_t started = micros();
    logger.capture(stateName());
    LogTiming::maximum(timing.maxCaptureUs, static_cast<uint32_t>(micros() - started));
}

void freeze(unsigned long now) {
    state = State::Frozen;
    commandPosition(Config::CLOSED_ROTATIONS, now);

    // Attempt to retain the final fault-state record.
    if (logger.status() == SdCsvLogger::Status::Recording) {
        captureRecord();
        logger.requestStop();
    }
}

bool validSdPin() {
    return SD_CS_PIN >= 2 && SD_CS_PIN < NUM_DIGITAL_PINS &&
           SD_CS_PIN != Config::CAN_CS_PIN && SD_CS_PIN != MISO &&
           SD_CS_PIN != MOSI && SD_CS_PIN != SCK &&
           SD_CS_PIN != A2 && SD_CS_PIN != A3;
}
} // namespace
void setup() {
    Serial.begin(Config::BAUD_RATE_LONG);
    // Deselect both SPI devices before any transactions.
    pinMode(Config::CAN_CS_PIN, OUTPUT);
    digitalWrite(Config::CAN_CS_PIN, HIGH);
    if (validSdPin()) {
        pinMode(SD_CS_PIN, OUTPUT);
        digitalWrite(SD_CS_PIN, HIGH);
    }
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalOneShotMode();
    lastLog = millis();
    statusMessages.push(LogStatus::Code::Boot);
}

void loop() {
    timing.loop(micros());
    struct can_frame frame;
    for (uint8_t count = 0; count < 8 &&
            mcp2515.readMessage(&frame) == MCP2515::ERROR_OK; ++count) {
        motor.handle_received_frame(frame);
        if (motorState.processFrame(frame) &&
            (frame.can_id & CAN_EFF_MASK &
             ~CanControl::LowLevel::SparkMax::SPARK_DEVICE_ID_MASK) ==
                CanControl::LowLevel::SparkMax::SPARK_ARB_STATUS_2) {
            positionReceivedDuringTravel = true;
        }
    }
    const unsigned long now = millis();
    if (now - lastHeartbeat >= controlIntervalMs) {
        timing.heartbeat(micros());
        CanControl::send_heartbeat(mcp2515, CanControl::default_heartbeat());
        lastHeartbeat = now;
    }
    logClock.update();
    for (uint8_t count = 0; count < 16 && Serial.available() > 0; ++count) {
        const char character = static_cast<char>(Serial.read());
        if (!receivingSerialLine && character == '?') {
            reportRequested = true;
            continue;
        }
        if (!receivingSerialLine && character == '!') {
            // Logging only: no valve command or test-state change.
            if (logger.status() == SdCsvLogger::Status::Recording) {
                logger.requestStop();
                statusMessages.push(LogStatus::Code::LogStopRequested);
            } else if (logger.status() == SdCsvLogger::Status::Closed) {
                statusMessages.push(LogStatus::Code::LogClosed);
            }
            continue;
        }
        if (state != State::Waiting) continue;
        if (!receivingSerialLine) {
            if (character == '\r' || character == '\n') continue;
            if (character == ' ') {
                if (!logClock.isSynchronized()) {
                    statusMessages.push(LogStatus::Code::NeedTime);
                    continue;
                }
                if (!validSdPin()) {
                    statusMessages.push(LogStatus::Code::InvalidSdPin);
                    continue;
                }
                const uint32_t initStarted = micros();
                const bool ready = logger.begin(static_cast<uint8_t>(SD_CS_PIN),
                                                nullptr, &motorState, &pt1, &pt2);
                timing.initUs = static_cast<uint32_t>(micros() - initStarted);
                lastReportedLogStatus = logger.status();
                if (!ready) {
                    statusMessages.push(LogStatus::Code::SdInitFailed);
                    reportRequested = true;
                    // Refresh timestamps on the next pass after blocking setup.
                    return;
                }
                const unsigned long startedMs = millis();
                timing.start(micros());
                timing.heartbeat(micros());
                CanControl::send_heartbeat(mcp2515, CanControl::default_heartbeat());
                lastHeartbeat = startedMs;
                cycleStart = lastLog = startedMs;
                missedLogSlots = 0;
                beginTravel(State::Opening, Config::OPEN_ROTATIONS, startedMs);
                captureRecord();
                statusMessages.push(LogStatus::Code::TestStarted);
                reportRequested = true;
                return;
            }
            receivingSerialLine = true;
        }
        switch (timeSync.feed(character)) {
            case LogTimeSync::Result::Synchronized:
                statusMessages.push(LogStatus::Code::TimeOK); break;
            case LogTimeSync::Result::InvalidMessage:
                statusMessages.push(LogStatus::Code::TimeInvalid); break;
            case LogTimeSync::Result::AlreadySynchronized:
                statusMessages.push(LogStatus::Code::TimeAlreadySet); break;
            case LogTimeSync::Result::None: break;
        }
        if (character == '\n') receivingSerialLine = false;
    }

    switch (state) {
        case State::Waiting:
            break;
        case State::Closed:
            // The three-minute period runs from one opening command to the next.
            if (now - cycleStart >= cycleIntervalMs) {
                cycleStart = now;
                beginTravel(State::Opening, Config::OPEN_ROTATIONS, now);
            }
            break;
        case State::Opening:
            if (now - stateStart > travelTimeoutMs) {
                freeze(now);
            } else if (positionReceivedDuringTravel &&
                       motorState.atPosition(Config::OPEN_ROTATIONS, now,
                                             Config::GOAL_POSITION_TOLERANCE,
                                             Config::TELEMETRY_TIMEOUT_MS)) {
                // Start the full 350 ms dwell only after reaching open.
                state = State::Open;
                stateStart = now;
            } else if (now - stateStart >= travelTimeoutMs) {
                freeze(now);
            }
            break;
        case State::Open:
            if (now - stateStart >= timeOpenMs) {
                beginTravel(State::Closing, Config::CLOSED_ROTATIONS, now);
            }
            break;
        case State::Closing:
            if (positionReceivedDuringTravel &&
                motorState.atPosition(Config::CLOSED_ROTATIONS, now,
                                      Config::GOAL_POSITION_TOLERANCE,
                                      Config::TELEMETRY_TIMEOUT_MS)) {
                state = State::Closed;
            } else if (now - stateStart >= travelTimeoutMs) {
                freeze(now);  // Also stop subsequent cycles if closing fails.
            }
            break;
        case State::Frozen:
            break;
    }

    // Preserve the original position-command behavior in every test state.
    if (state != State::Waiting && now - lastCommand >= controlIntervalMs) {
        const bool openingOrOpen = state == State::Opening || state == State::Open;
        commandPosition(openingOrOpen ? Config::OPEN_ROTATIONS : Config::CLOSED_ROTATIONS, now);
    }
    const unsigned long sampleNowMs = millis();
    if (state != State::Waiting && state != State::Frozen &&
        logger.status() == SdCsvLogger::Status::Recording) {
        const unsigned long elapsedMs = sampleNowMs - lastLog;
        if (elapsedMs >= logIntervalMs) {
            const unsigned long elapsedSlots = elapsedMs / logIntervalMs;
            missedLogSlots += elapsedSlots - 1;
            lastLog += elapsedSlots * logIntervalMs;
            captureRecord();
        }
    }
    // Snapshot before storage work so a closure request from the previous
    // pass includes its measured delay and the following loop gap.
    if (reportRequested && !diagnostics.busy()) {
        diagnostics.begin(logger, timing, missedLogSlots,
                          statusMessages.droppedMessages(), controlIntervalMs);
        reportRequested = false;
    }
    const uint32_t storageStarted = micros();
    logger.service();
    LogTiming::maximum(timing.maxServiceUs,
                       static_cast<uint32_t>(micros() - storageStarted));
    const SdCsvLogger::Status current = logger.status();
    if (current != lastReportedLogStatus) {
        if (current == SdCsvLogger::Status::Failed) {
            statusMessages.push(LogStatus::Code::SdWriteFailed);
            reportRequested = true;
        } else if (current == SdCsvLogger::Status::Closed) {
            statusMessages.push(LogStatus::Code::LogClosed);
            reportRequested = true;
        }
        lastReportedLogStatus = current;
    }
    statusMessages.service();
    diagnostics.service();
}
