#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <math.h>
#include <mcp2515.h>

#include "config.h"
#include "motor_state.h"
#include "pressure_transducer.h"

namespace {
MCP2515 mcp2515(Config::CAN_CS_PIN, 4000000UL);
CanControl::SparkMax motor(mcp2515, Config::LOX_MOTOR_ID);
MotorState motorState(Config::LOX_MOTOR_ID);
pressure_transducer pt1(A0, 1000, 0.5, 4.5);
pressure_transducer pt2(A1, 1000, 0.5, 4.5);

// 25 Hz logging frequency
constexpr unsigned long logIntervalMs = 40;
constexpr unsigned long logIntervalClosedMs = 5000;
// Motor communication keeps its own 50 Hz schedule, independent of logging.
constexpr unsigned long controlIntervalMs = 20;
constexpr unsigned long cycleIntervalMs = 180000;
constexpr unsigned long timeOpenMs = 350;
constexpr unsigned long travelTimeoutMs = 2000;
constexpr double positionTolerance = 0.25;  // Motor rotations, as in dwell_test

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

void commandPosition(double target, unsigned long now) {
    motor.set_position(target, 0, 0, 0);
    lastCommand = now;
}

void beginTravel(State next, double target, unsigned long now) {
    state = next;
    stateStart = now;
    // Require a new encoder report before accepting the target as reached.
    positionReceivedDuringTravel = false;
    commandPosition(target, now);
}

bool atPosition(double target, unsigned long now) {
    // Missing, stale, or non-finite feedback must not complete an actuation.
    return positionReceivedDuringTravel && motorState.hasStatus2() &&
           now - motorState.getLastStatus2Time() <= Config::TELEMETRY_TIMEOUT_MS &&
           isfinite(motorState.getPosition()) &&
           fabs(motorState.getPosition() - target) <= positionTolerance;
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

void logRow(unsigned long now) {
    // Suppress the PT helper's voltage messages to keep every row valid CSV.
    Serial.print(pt1.getPressure(false), 2);
    Serial.print(',');
    Serial.print(pt2.getPressure(false), 2);
    Serial.print(',');
    Serial.print(now);
    Serial.print(',');  // Milliseconds since boot
    // Gear ratio is motor revolutions per valve revolution; report valve degrees.
    Serial.print(motorState.getPosition() * 360.0 / Config::GEAR_RATIO, 3);
    Serial.print(',');
    Serial.print(motorState.getAppliedOutput(), 4);
    Serial.print(',');
    Serial.print(motorState.getTemperature(), 1);
    Serial.print(',');
    Serial.print(motorState.getAppliedOutputPercent(), 2);
    Serial.print(',');
    Serial.print(motorState.getRPM(), 2);
    Serial.print(',');
    Serial.print(motorState.getVoltageRaw());
    Serial.print(',');
    Serial.print(motorState.getCurrentRaw());
    Serial.print(',');
    // These flags distinguish initial cached zeroes from received motor data.
    Serial.print(motorState.hasStatus0());
    Serial.print(',');
    Serial.print(motorState.hasStatus2());
    Serial.print(',');
    Serial.print(motorState.hasTelemetry());
    Serial.print(',');
    Serial.print(motorState.isFresh(Config::TELEMETRY_TIMEOUT_MS));
    Serial.print(',');
    Serial.print(motorState.getLastStatus0Time());
    Serial.print(',');
    Serial.print(motorState.getLastStatus2Time());
    Serial.print(',');
    Serial.println(stateName());
}

void freeze(unsigned long now) {
    state = State::Frozen;
    commandPosition(Config::CLOSED_ROTATIONS, now);
    logRow(now);  // One final CSV row; no further serial output until reset.
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
            } else if (atPosition(Config::OPEN_ROTATIONS, now)) {
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
            if (atPosition(Config::CLOSED_ROTATIONS, now)) {
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

    // Log every 5 seconds while idle; every 40 ms during actuation.
    const unsigned long interval =
        (state == State::Closed || state == State::Waiting)
            ? logIntervalClosedMs
            : logIntervalMs;
        
    // print at 40ms 25Hz when not frozen, closed, or waiting
    if (state != State::Frozen && now - lastLog >= interval) {
        lastLog += interval;
        if (now - lastLog >= interval) lastLog = now;  // No bursts after delays.
        logRow(now);
    } 
}
