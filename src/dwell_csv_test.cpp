/**
 * Repeated valve dwell test with binary SD logging.
 * - Starts when a space arrives over Serial.
 * - Opens every three minutes and holds open for 350 ms.
 * - Starts the dwell after fresh feedback confirms the open position.
 * - Logs pressure and motor telemetry at 25 Hz.
 * - Stops cycling on a travel timeout and keeps commanding closed.
 */
#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <mcp2515.h>

#include "config.h"
#include "motor_state.h"
#include "pressure_transducer.h"
#include "log.h"

namespace {
MCP2515 mcp2515(Config::kCanCsPin, 4000000UL);
CanControl::SparkMax motor(mcp2515, Config::kLoxMotorId);
MotorState motorState(Config::kLoxMotorId);
PressureTransducer pt1 = PressureTransducer(A2, 1000, 0.47, 4.5);
PressureTransducer pt2 = PressureTransducer(A3, 1000, 0.47, 4.5);

 // Motor communication keeps its own 50 Hz schedule, independent of logging.
constexpr unsigned long controlIntervalMs = 20;
constexpr unsigned long cycleIntervalMs = 180000;
constexpr unsigned long timeOpenMs = 350;
constexpr unsigned long travelTimeoutMs = 2000;

// State machine. test state determines the next action of the motor
enum class State { Waiting,
                   Closed,
                   Opening,
                   Open,
                   Closing,
                   Frozen };

State state = State::Waiting;

unsigned long cycleStart = 0;
unsigned long stateStart = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastCommand = 0;
unsigned long lastLog = 0;
bool positionReceivedDuringTravel = false;
// No position command has been submitted while initially waiting.
float commandedPosition = NAN;

// sets position to motor
void command_position(double target, unsigned long now) {
    motor.set_position(target, 0, 0, 0);
    commandedPosition = target;
    lastCommand = now;
}

// begins to  move motor to the next state setpoint
void begin_travel(State next, double target, unsigned long now) {
    state = next;
    stateStart = now;
    // Require a new encoder report before accepting the target as reached.
    positionReceivedDuringTravel = false;
    command_position(target, now);
}

const __FlashStringHelper* state_name() {
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

void freeze(unsigned long now) {
    state = State::Frozen;
    command_position(Config::kClosedRotations, now);
    log_row(&motorState, &pt1, &pt2, now, state_name(), commandedPosition);
    close_sd_log();
    Serial.print("froze after: ");
    Serial.print(now / 1000);
    Serial.println(" seconds");
}
}  // namespace

void setup() {
    Serial.begin(Config::kBaudRateLong);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalOneShotMode();
    begin_sd_log();
    lastLog = millis();
}

void loop() {
    // Bound receive work so continuous CAN traffic cannot starve control timing.
    struct can_frame frame;
    for (uint8_t count = 0; count < 8 &&
                            mcp2515.readMessage(&frame) == MCP2515::ERROR_OK;
         ++count) {
        motor.handle_received_frame(frame);
        if (motorState.process_frame(frame) &&
            (frame.can_id & CAN_EFF_MASK &
             ~CanControl::LowLevel::SparkMax::SPARK_DEVICE_ID_MASK) ==
                CanControl::LowLevel::SparkMax::SPARK_ARB_STATUS_2) {
            positionReceivedDuringTravel = true;
        }
    }
    const unsigned long now = millis();
    if (now - lastHeartbeat >= controlIntervalMs) {
        CanControl::send_heartbeat(mcp2515, CanControl::default_heartbeat());
        lastHeartbeat = now;
    }

    // No newline required. Further input cannot restart a terminated test.
    for (uint8_t count = 0; count < 16 && Serial.available() > 0; ++count) {
        if (Serial.read() == ' ' && state == State::Waiting) {
            cycleStart = now;
            begin_travel(State::Opening, Config::kOpenRotations, now);
        }
    }

    /**
     * switch determines behavior based on test state. once a certain state has reached
     * its end state
     */
    switch (state) {
        case State::Waiting:
            break;
        case State::Closed:
            // The two-minute period runs from one opening command to the next.
            if (now - cycleStart >= cycleIntervalMs) {
                cycleStart = now;
                begin_travel(State::Opening, Config::kOpenRotations, now);
            }
            break;
        case State::Opening:
            if (now - stateStart > travelTimeoutMs) {
                freeze(now);
            } else if (positionReceivedDuringTravel &&
                       motorState.at_position(Config::kOpenRotations, now,
                                             Config::kGoalPositionTolerance,
                                             Config::kTelemetryTimeoutMs)) {
                // Start the full one-second dwell only after reaching open.
                state = State::Open;
                stateStart = now;
            } else if (now - stateStart >= travelTimeoutMs) {
                freeze(now);
            }
            break;
        case State::Open:
            if (now - stateStart >= timeOpenMs) {
                begin_travel(State::Closing, Config::kClosedRotations, now);
            }
            break;
        case State::Closing:
            if (positionReceivedDuringTravel &&
                motorState.at_position(Config::kClosedRotations, now,
                                      Config::kGoalPositionTolerance,
                                      Config::kTelemetryTimeoutMs)) {
                state = State::Closed;
            } else if (now - stateStart >= travelTimeoutMs) {
                freeze(now);  // Also stop subsequent cycles if closing fails.
            }
            break;
        case State::Frozen:
            break;
    }

    // Keep heartbeats and close commands running even after serial goes silent.
    if (state != State::Waiting && now - lastCommand >= controlIntervalMs) {
        const bool openingOrOpen = state == State::Opening || state == State::Open;
        command_position(openingOrOpen ? Config::kOpenRotations : Config::kClosedRotations, now);
    }
        
    // print at 40ms 25Hz when not frozen, closed, or waiting
    if (state != State::Frozen && now - lastLog >= Config::kLogFrequency25Hz) {
        lastLog += Config::kLogFrequency25Hz;
        if (now - lastLog >= Config::kLogFrequency25Hz) lastLog = now;  // No bursts after delays.
        log_row(&motorState, &pt1, &pt2, now, state_name(), commandedPosition);
    } 
    service_sd_log();
}
