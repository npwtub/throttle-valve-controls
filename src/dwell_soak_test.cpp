/**
 * Valve soak test with binary SD logging.
 * - Starts when a space arrives over Serial.
 * - Holds open for two seconds, then closes for a one-hour soak.
 * - Repeats two seconds open and one second closed after the soak.
 * - Uses fresh position feedback to confirm each movement.
 * - Logs pressure and motor telemetry at 25 Hz until frozen.
 * - Stops cycling on a travel timeout and keeps commanding closed.
 */
#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <mcp2515.h>

#include "config.h"
#include "log.h"
#include "motor_state.h"
#include "pressure_transducer.h"

namespace {
MCP2515 mcp2515(Config::CAN_CS_PIN, 4000000UL);
CanControl::SparkMax motor(mcp2515, Config::LOX_MOTOR_ID);
MotorState motorState(Config::LOX_MOTOR_ID);
PressureTransducer pt1 = PressureTransducer(A2, 1000, 0.47, 4.5);
PressureTransducer pt2 = PressureTransducer(A3, 1000, 0.47, 4.5);

// Change this constant to set the closed wait after the initial actuation.
// constexpr unsigned long soakTimeMs = 60UL * 60UL * 1000UL;
constexpr unsigned long soakTimeMs = 1000UL;
constexpr unsigned long openTimeMs = 7000;
constexpr unsigned long cycleClosedMs = 1000;
constexpr unsigned long controlIntervalMs = 20;
constexpr unsigned long travelTimeoutMs = 3000;

enum class State {
    Waiting,
    InitialOpening,
    InitialOpen,
    Closing,
    Soaking,
    CycleOpening,
    CycleOpen,
    CycleClosing,
    CycleClosed,
    Frozen
};
State state = State::Waiting;
unsigned long stateStart = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastCommand = 0;
unsigned long lastLog = 0;
bool positionReceivedDuringTravel = false;
double targetPosition = Config::CLOSED_ROTATIONS;

void commandPosition(unsigned long now) {
    motor.set_position(targetPosition, 0, 0, 0);
    lastCommand = now;
}

void enterState(State next, unsigned long now) {
    state = next;
    stateStart = now;
    // Start each state's logging cadence from its transition time.
    lastLog = now;
}

void beginTravel(State next, double target, unsigned long now) {
    enterState(next, now);
    targetPosition = target;
    // Cached position from before the command cannot complete this travel.
    positionReceivedDuringTravel = false;
    commandPosition(now);
}

const __FlashStringHelper* stateName() {
    switch (state) {
        case State::Waiting:
            return F("waiting");
        case State::InitialOpening:
            return F("initial_opening");
        case State::InitialOpen:
            return F("initial_open");
        case State::Closing:
            return F("closing");
        case State::Soaking:
            return F("soaking");
        case State::CycleOpening:
            return F("cycle_opening");
        case State::CycleOpen:
            return F("cycle_open");
        case State::CycleClosing:
            return F("cycle_closing");
        case State::CycleClosed:
            return F("cycle_closed");
        case State::Frozen:
            return F("motor froze");
    }
    return F("unknown");
}

void freeze(unsigned long now) {
    // Stop cycling and keep commanding closed after any travel failure.
    targetPosition = Config::CLOSED_ROTATIONS;
    enterState(State::Frozen, now);
    commandPosition(now);
    logRow(&motorState, &pt1, &pt2, now, stateName(), targetPosition);
    closeSdLog();
    // Final fault record is flushed and the file is closed.
}
}  // namespace

void setup() {
    Serial.begin(Config::BAUD_RATE_LONG);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalOneShotMode();
    beginSdLog();
    lastLog = millis();
}

void loop() {
    // Bound receive processing so a busy CAN bus cannot monopolize the loop.
    struct can_frame frame;
    for (uint8_t count = 0; count < 8 &&
                            mcp2515.readMessage(&frame) == MCP2515::ERROR_OK;
         ++count) {
        motor.handle_received_frame(frame);
        if (motorState.processFrame(frame) &&
            (frame.can_id & CAN_EFF_MASK &
             ~CanControl::LowLevel::SparkMax::SPARK_DEVICE_ID_MASK) ==
                CanControl::LowLevel::SparkMax::SPARK_ARB_STATUS_2) {
            positionReceivedDuringTravel = true;
        }
    }
    const unsigned long now = millis();
    // Heartbeats run in every state, including before start and after a fault.
    if (now - lastHeartbeat >= controlIntervalMs) {
        CanControl::send_heartbeat(mcp2515, CanControl::default_heartbeat());
        lastHeartbeat = now;
    }
    for (uint8_t count = 0; count < 16 && Serial.available() > 0; ++count) {
        // A space starts once; no prompt or newline is required.
        if (Serial.read() == ' ' && state == State::Waiting) {
            beginTravel(State::InitialOpening, Config::OPEN_ROTATIONS, now);
        }
    }

    switch (state) {
        case State::Waiting:
            break;
        case State::InitialOpening:
        case State::Closing:
        case State::CycleOpening:
        case State::CycleClosing:
            // Reject late arrival as well as missing or stale position feedback.
            if (now - stateStart > travelTimeoutMs) {
                freeze(now);
            } else if (positionReceivedDuringTravel &&
                       motorState.atPosition(targetPosition, now, Config::GOAL_POSITION_TOLERANCE,
                                             Config::TELEMETRY_TIMEOUT_MS)) {
                State next = State::CycleOpen;
                if (state == State::InitialOpening) {
                    next = State::InitialOpen;
                } else if (state == State::Closing) {
                    next = State::Soaking;
                } else if (state == State::CycleClosing) {
                    next = State::CycleClosed;
                }
                enterState(next, now);
                // Capture the arrival, then continue at this state's log interval.
                logRow(&motorState, &pt1, &pt2, now, stateName(), targetPosition);
            } else if (now - stateStart >= travelTimeoutMs) {
                freeze(now);
            }
            break;
        case State::InitialOpen:
            // Hold for two full seconds AFTER the valve reaches open.
            if (now - stateStart >= openTimeMs) {
                beginTravel(State::Closing, Config::CLOSED_ROTATIONS, now);
            }
            break;
        case State::Soaking:
            // The one-hour soak begins only once closed position is confirmed.
            if (now - stateStart >= soakTimeMs) {
                beginTravel(State::CycleOpening, Config::OPEN_ROTATIONS, now);
            }
            break;
        case State::CycleOpen:
            // Each dwell starts after fresh feedback confirms arrival.
            if (now - stateStart >= openTimeMs) {
                beginTravel(State::CycleClosing, Config::CLOSED_ROTATIONS, now);
            }
            break;
        case State::CycleClosed:
            if (now - stateStart >= cycleClosedMs) {
                beginTravel(State::CycleOpening, Config::OPEN_ROTATIONS, now);
            }
            break;
        case State::Frozen:
            break;
    }

    // Keep commanding the current target, including closed after a fault.
    // Waiting sends no position command; it only receives telemetry and heartbeats.
    if (state != State::Waiting && now - lastCommand >= controlIntervalMs) {
        commandPosition(now);
    }

    if (state != State::Frozen && now - lastLog >= Config::LOG_FREQUENCY_25_HZ) {
        lastLog += Config::LOG_FREQUENCY_25_HZ;
        // Skip missed samples instead of sending a burst after a delay.
        if (now - lastLog >= Config::LOG_FREQUENCY_25_HZ) lastLog = now;
        logRow(&motorState, &pt1, &pt2, now, stateName(),
               state == State::Waiting ? NAN : targetPosition);
    }
    serviceSdLog();
}
