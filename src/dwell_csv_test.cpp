/**
 * Repeated valve dwell test with CSV logging.
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
MCP2515 mcp2515(Config::CAN_CS_PIN, 4000000UL);
CanControl::SparkMax motor(mcp2515, Config::LOX_MOTOR_ID);
MotorState motorState(Config::LOX_MOTOR_ID);
PressureTransducer pt1(A0, 1000, 0.5, 4.5);
PressureTransducer pt2(A1, 1000, 0.5, 4.5);

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
unsigned long start_ms = 0;
bool positionReceivedDuringTravel = false;

// sets position to motor
void commandPosition(double target, unsigned long now) {
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

void freeze(unsigned long now) {
    state = State::Frozen;
    commandPosition(Config::CLOSED_ROTATIONS, now);
    logRow(&motorState, &pt1, &pt2, now, stateName());// One final CSV row; no further serial output until reset.
    Serial.print("froze after: ");
    Serial.print(now / 1000);
    Serial.println(" seconds");
}
}  // namespace

void setup() {
    Serial.begin(Config::BAUD_RATE_LONG);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalOneShotMode();
    Serial.println(F("pt1_psi,pt2_psi,timestamp_ms,position_deg,output,temp_c,output_percent,rpm,voltage_raw,current_raw,has_status0,has_status2,has_telemetry,is_fresh,last_status0_ms,last_status2_ms,state"));
    lastLog = millis();
}

void loop() {
    // Bound receive work so continuous CAN traffic cannot starve control timing.
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
    if (now - lastHeartbeat >= controlIntervalMs) {
        CanControl::send_heartbeat(mcp2515, CanControl::default_heartbeat());
        lastHeartbeat = now;
    }

    // No newline required. Further input cannot restart a terminated test.
    for (uint8_t count = 0; count < 16 && Serial.available() > 0; ++count) {
        if (Serial.read() == ' ' && state == State::Waiting) {
            cycleStart = now;
            beginTravel(State::Opening, Config::OPEN_ROTATIONS, now);
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
                // Start the full one-second dwell only after reaching open.
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

    // Keep heartbeats and close commands running even after serial goes silent.
    if (state != State::Waiting && now - lastCommand >= controlIntervalMs) {
        const bool openingOrOpen = state == State::Opening || state == State::Open;
        commandPosition(openingOrOpen ? Config::OPEN_ROTATIONS : Config::CLOSED_ROTATIONS, now);
    }
        
    // print at 40ms 25Hz when not frozen, closed, or waiting
    if (state != State::Frozen && now - lastLog >= Config::LOG_FREQUENCY_25_HZ) {
        lastLog += Config::LOG_FREQUENCY_25_HZ;
        if (now - lastLog >= Config::LOG_FREQUENCY_25_HZ) lastLog = now;  // No bursts after delays.
        logRow(&motorState, &pt1, &pt2, now, stateName());
    } 
}
