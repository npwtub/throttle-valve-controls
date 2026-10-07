/**
 * this test runs the valve through a configurable set of angles, waits at each
 * one for a little bit, and logs the pressure and motor data at 25 hz before
 * moving on. the idea is to leave us with clean steady-ish data at known valve
 * positions so the later analysis can build a cda vs angle curve. that is the
 * same basic characterization idea used in the berkeley ereg paper, where they
 * first mapped valve opening to flow behavior and then used that map in the
 * model/feedforward side of the controller.
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
CanControl::SparkMax motor(mcp2515, Config::CDA_MOTOR_ID);
MotorState motorState(Config::CDA_MOTOR_ID);
PressureTransducer ptUp(A0,
                        Config::CDA_PT_MAX_PSI,
                        Config::CDA_PT_MIN_VOLTAGE,
                        Config::CDA_PT_MAX_VOLTAGE);
PressureTransducer ptDown(A1,
                          Config::CDA_PT_MAX_PSI,
                          Config::CDA_PT_MIN_VOLTAGE,
                          Config::CDA_PT_MAX_VOLTAGE);

enum class State {
    Waiting,
    StartDelay,
    Opening,
    AtSetpoint,
    Closing,
    Finished,
    Frozen
};

State state = State::Waiting;

unsigned long stateStart = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastCommand = 0;
unsigned long lastLog = 0;

bool positionReceivedDuringTravel = false;
double targetAngleDeg = Config::CDA_START_ANGLE_DEG;
double targetPositionRotations = Config::CLOSED_ROTATIONS;

constexpr double angleEpsilonDeg = 1e-6;

static_assert(Config::CDA_ANGLE_INCREMENT_DEG > 0.0,
              "CDA angle increment must be positive");
static_assert(Config::CDA_START_ANGLE_DEG >= Config::CLOSED_THETA,
              "CDA starting angle cannot be below the closed angle");
static_assert(Config::CDA_END_ANGLE_DEG <= Config::OPEN_THETA,
              "CDA ending angle cannot exceed the fully-open angle");
static_assert(Config::CDA_START_ANGLE_DEG <= Config::CDA_END_ANGLE_DEG,
              "CDA starting angle must not exceed the ending angle");

// turns a valve angle into the motor encoder position the spark max expects.
double angleToMotorRotations(double valveAngleDeg) {
    return (valveAngleDeg / 360.0) * Config::GEAR_RATIO;
}

void commandTarget(unsigned long now) {
    motor.set_position(targetPositionRotations, 0, 0, 0);
    lastCommand = now;
}

void enterState(State next, unsigned long now) {
    state = next;
    stateStart = now;
}

// starts a position move and makes sure we see fresh encoder data after the
// command before calling it done, so stale telemetry cannot fake a setpoint hit.
void beginTravel(State next, double valveAngleDeg, unsigned long now) {
    targetAngleDeg = valveAngleDeg;
    targetPositionRotations = angleToMotorRotations(valveAngleDeg);
    positionReceivedDuringTravel = false;
    enterState(next, now);
    commandTarget(now);
}

const __FlashStringHelper* stateName() {
    switch (state) {
        case State::Waiting:
            return F("waiting");
        case State::StartDelay:
            return F("start_delay");
        case State::Opening:
            return F("opening");
        case State::AtSetpoint:
            return F("at_setpoint");
        case State::Closing:
            return F("closing");
        case State::Finished:
            return F("finished");
        case State::Frozen:
            return F("motor_froze");
    }
    return F("unknown");
}

// only status 2 from the characterization motor counts here since that is the
// frame carrying the encoder position and velocity we care about.
bool isPositionFrame(const struct can_frame& frame) {
    namespace Spark = CanControl::LowLevel::SparkMax;

    if (frame.can_id & CAN_RTR_FLAG) return false;
    if (!(frame.can_id & CAN_EFF_FLAG)) return false;

    const uint32_t id = frame.can_id & CAN_EFF_MASK;
    const uint8_t receivedMotorID = id & Spark::SPARK_DEVICE_ID_MASK;
    if (receivedMotorID != Config::CDA_MOTOR_ID) return false;

    const uint32_t baseID = id & ~Spark::SPARK_DEVICE_ID_MASK;
    return baseID == Spark::SPARK_ARB_STATUS_2;
}

void freeze(unsigned long now) {
    // if a move times out, fail safe by keeping the target at closed.
    targetAngleDeg = Config::CLOSED_THETA;
    targetPositionRotations = Config::CLOSED_ROTATIONS;
    enterState(State::Frozen, now);
    commandTarget(now);

    // grab one last row with the failure state, then stop the normal logger.
    logRow(&motorState, &ptUp, &ptDown, now, stateName());
}

void advanceSetpointOrClose(unsigned long now) {
    if (targetAngleDeg < Config::CDA_END_ANGLE_DEG - angleEpsilonDeg) {
        double nextAngle = targetAngleDeg + Config::CDA_ANGLE_INCREMENT_DEG;
        if (nextAngle > Config::CDA_END_ANGLE_DEG) {
            nextAngle = Config::CDA_END_ANGLE_DEG;
        }
        beginTravel(State::Opening, nextAngle, now);
        return;
    }

    beginTravel(State::Closing, Config::CLOSED_THETA, now);
}
}  // namespace

void setup() {
    Serial.begin(Config::BAUD_RATE_LONG);

    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalOneShotMode();

    // this header matches the field order that logrow() writes below.
    Serial.println(F("timestamp_ms,pt_up_psi,pt_down_psi,position_deg,temp_c,output_percent,rpm,motor_voltage,motor_current,has_status0,has_status2,has_telemetry,is_fresh,last_status0_ms,last_status2_ms,state"));

    lastLog = millis();
}

void loop() {
    // only chew through a few can frames per loop so a busy bus cannot hold up
    // the heartbeat, state machine, motor command refresh, or logger.
    struct can_frame frame;
    for (uint8_t count = 0;
         count < 8 && mcp2515.readMessage(&frame) == MCP2515::ERROR_OK;
         ++count) {
        motor.handle_received_frame(frame);

        if (motorState.processFrame(frame) && isPositionFrame(frame)) {
            positionReceivedDuringTravel = true;
        }
    }

    const unsigned long now = millis();

    // keep the spark max heartbeat going so the controller stays enabled.
    if (now - lastHeartbeat >= Config::CDA_CONTROL_INTERVAL_MS) {
        CanControl::send_heartbeat(mcp2515, CanControl::default_heartbeat());
        lastHeartbeat = now;
    }

    // one space starts the test and everything else gets ignored. this keeps the
    // same simple operator-start idea as the old sketch without blocking the loop.
    for (uint8_t count = 0; count < 16 && Serial.available() > 0; ++count) {
        if (Serial.read() == ' ' && state == State::Waiting) {
            targetAngleDeg = Config::CLOSED_THETA;
            targetPositionRotations = Config::CLOSED_ROTATIONS;
            enterState(State::StartDelay, now);
            commandTarget(now);
        }
    }

    switch (state) {
        case State::Waiting:
            break;

        case State::StartDelay:
            if (now - stateStart >= Config::CDA_START_DELAY_MS) {
                beginTravel(State::Opening, Config::CDA_START_ANGLE_DEG, now);
            }
            break;

        case State::Opening:
            if (positionReceivedDuringTravel &&
                motorState.atPosition(targetPositionRotations,
                                      now,
                                      Config::CDA_POSITION_TOLERANCE_ROTATIONS,
                                      Config::TELEMETRY_TIMEOUT_MS)) {
                enterState(State::AtSetpoint, now);
            } else if (now - stateStart >= Config::CDA_TRAVEL_TIMEOUT_MS) {
                freeze(now);
            }
            break;

        case State::AtSetpoint:
            // start the dwell only after fresh feedback says we actually got there.
            if (now - stateStart >= Config::CDA_DWELL_TIME_MS) {
                advanceSetpointOrClose(now);
            }
            break;

        case State::Closing:
            if (positionReceivedDuringTravel &&
                motorState.atPosition(Config::CLOSED_ROTATIONS,
                                      now,
                                      Config::CDA_POSITION_TOLERANCE_ROTATIONS,
                                      Config::TELEMETRY_TIMEOUT_MS)) {
                targetAngleDeg = Config::CLOSED_THETA;
                targetPositionRotations = Config::CLOSED_ROTATIONS;
                enterState(State::Finished, now);
                commandTarget(now);
                logRow(&motorState, &ptUp, &ptDown, now, stateName());
            } else if (now - stateStart >= Config::CDA_TRAVEL_TIMEOUT_MS) {
                freeze(now);
            }
            break;

        case State::Finished:
        case State::Frozen:
            break;
    }

    // keep refreshing the position command after the test starts. once we finish
    // or freeze, the repeated command is intentionally just the closed position.
    if (state != State::Waiting &&
        now - lastCommand >= Config::CDA_CONTROL_INTERVAL_MS) {
        commandTarget(now);
    }

    /**
     * log the whole characterization at 25 hz with the shared logger. keeping one
     * logger and a fixed sample rate makes the later cda analysis a lot cleaner,
     * especially when we compare pressure data across valve angles. this follows
     * the same general workflow as the ereg paper: get the valve characterization
     * data first, then use that measured relationship in the control model later.
     */
    if (state != State::Finished && state != State::Frozen &&
        now - lastLog >= Config::LOG_FREQUENCY_25_HZ) {
        lastLog += Config::LOG_FREQUENCY_25_HZ;
        // if we miss a sample, skip it instead of dumping a burst of old timestamps.
        if (now - lastLog >= Config::LOG_FREQUENCY_25_HZ) {
            lastLog = now;
        }
        logRow(&motorState, &ptUp, &ptDown, now, stateName());
    }
}
