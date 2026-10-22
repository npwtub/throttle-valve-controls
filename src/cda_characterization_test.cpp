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
/**
 * this is the actual characterization sweep. start angle, end angle, step size,
 * and dwell time are all easy to tweak here depending on how much resolution
 * we want. the ereg paper did the same kind of angle-to-flow characterization
 * before building its feedforward model, while our later analysis will use the
 * logged test data to build the cda vs angle relationship for this valve.
 */
// constexpr double startAngleDeg = Config::CLOSED_THETA;
constexpr double startAngleDeg = 5.0;
constexpr double endAngleDeg = Config::OPEN_THETA;
constexpr double angleIncrementDeg = 10.0;
constexpr unsigned long dwellTimeMs = 3000;

// motor commands and heartbeats are refreshed every 20 ms, same basic timing
// used by the other tests.
constexpr unsigned long controlIntervalMs = 5;

// this is how long a commanded move can take before the state machine gives up
// and commands the valve closed.
constexpr unsigned long travelTimeoutMs = 3000;

MCP2515 mcp2515(Config::CAN_CS_PIN, 4000000UL);
CanControl::SparkMax motor(mcp2515, Config::LOX_MOTOR_ID);
MotorState motorState(Config::LOX_MOTOR_ID);
PressureTransducer ptUp(Config::PT1_INPUT_PIN, Config::PT1_MAX_PSI, Config::PT_MIN_VOLTAGE, Config::PT_MAX_VOLTAGE);
PressureTransducer ptDown(Config::PT2_INPUT_PIN, Config::PT2_MAX_PSI, Config::PT_MIN_VOLTAGE, Config::PT_MAX_VOLTAGE);

enum class State {
    Waiting,
    Moving,
    AtSetpoint,
    Finished,
    TravelTimeout
};

State state = State::Waiting;

unsigned long stateStart = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastCommand = 0;
unsigned long lastLog = 0;

bool positionReceivedDuringTravel = false;
bool descendingSweep = false;
double targetAngleDeg = startAngleDeg;
double targetPositionRotations = Config::CLOSED_ROTATIONS;

constexpr double angleEpsilonDeg = 1e-6;

static_assert(angleIncrementDeg > 0.0,
              "CDA angle increment must be positive");
static_assert(startAngleDeg >= Config::CLOSED_THETA,
              "CDA starting angle cannot be below the closed angle");
static_assert(endAngleDeg <= Config::OPEN_THETA,
              "CDA ending angle cannot exceed the fully-open angle");
static_assert(startAngleDeg <= endAngleDeg,
              "CDA starting angle must not exceed the ending angle");

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
    targetPositionRotations = (valveAngleDeg / 360.0) * Config::GEAR_RATIO;
    positionReceivedDuringTravel = false;
    enterState(next, now);
    commandTarget(now);
}

const __FlashStringHelper* stateName() {
    switch (state) {
        case State::Waiting:
            return F("waiting");

        case State::Moving:
            return descendingSweep ? F("moving_down") : F("moving_up");

        case State::AtSetpoint:
            return descendingSweep ? F("at_setpoint_down") : F("at_setpoint_up");

        case State::Finished:
            return F("finished");

        case State::TravelTimeout:
            return F("travel_timeout");
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
    if (receivedMotorID != Config::LOX_MOTOR_ID) return false;

    const uint32_t baseID = id & ~Spark::SPARK_DEVICE_ID_MASK;
    return baseID == Spark::SPARK_ARB_STATUS_2;
}

void travelTimeout(unsigned long now) {
    // this state means the move timed out, not necessarily that the motor itself
    // froze. keep the fail-safe behavior and command the valve back closed.
    targetAngleDeg = Config::CLOSED_THETA;
    targetPositionRotations = Config::CLOSED_ROTATIONS;
    enterState(State::TravelTimeout, now);
    commandTarget(now);

    // grab one last row with the timeout state, then stop the normal logger.
    logRow(&motorState, &ptUp, &ptDown, now, stateName());
}

void finishTest(unsigned long now) {
    targetAngleDeg = Config::CLOSED_THETA;
    targetPositionRotations = Config::CLOSED_ROTATIONS;

    enterState(State::Finished, now);
    commandTarget(now);

    logRow(&motorState, &ptUp, &ptDown, now, stateName());
}

void advanceSetpointOrFinish(unsigned long now) {
    if (!descendingSweep) {
        // keep stepping upward until we reach the fully open setpoint.
        if (targetAngleDeg <= endAngleDeg - angleIncrementDeg) {
            double nextAngle = targetAngleDeg + angleIncrementDeg;

            beginTravel(State::Moving, nextAngle, now);
            return;
        }

        /**
         * once the top setpoint finishes its dwell, reverse direction and walk
         * back through the same characterization angles. logging both directions
         * lets us compare the measured behavior at the same valve angle depending
         * on whether we approached it while opening or closing.
         */
        descendingSweep = true;

        // The top setpoint already had its dwell; move straight to the next lower angle.
        double nextAngle = targetAngleDeg - angleIncrementDeg;

        if (nextAngle < startAngleDeg) {
            nextAngle = startAngleDeg;
        }

        beginTravel(State::Moving, nextAngle, now);
        return;
    }

    // keep stepping downward until we return to the starting angle.
    if (targetAngleDeg > startAngleDeg + angleEpsilonDeg) {
        double nextAngle = targetAngleDeg - angleIncrementDeg;

        if (nextAngle < startAngleDeg) {
            nextAngle = startAngleDeg;
        }

        beginTravel(State::Moving, nextAngle, now);
        return;
    }

    // the starting angle already corresponds to closed, so after its final
    // three-second dwell the characterization is complete.
    finishTest(now);
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
    if (now - lastHeartbeat >= controlIntervalMs) {
        CanControl::send_heartbeat(mcp2515, CanControl::default_heartbeat());
        lastHeartbeat = now;
    }

    // one space starts the test and everything else gets ignored. this keeps the
    // same simple operator-start idea as the old sketch without blocking the loop.
    for (uint8_t count = 0; count < 16 && Serial.available() > 0; ++count) {
        if (Serial.read() == ' ' && state == State::Waiting) {
            beginTravel(State::Moving, startAngleDeg, now);
        }
    }

    switch (state) {
        case State::Waiting:
            break;

        case State::Moving:
            // use the same motor-position tolerance as the bench-tested tests. the
            // spark max can settle a little short of the exact requested angle, so
            // a tighter cda-only tolerance can call a healthy settled move a timeout.
            if (positionReceivedDuringTravel &&
                motorState.atPosition(targetPositionRotations,
                                      now,
                                      Config::GOAL_POSITION_TOLERANCE,
                                      Config::TELEMETRY_TIMEOUT_MS)) {
                enterState(State::AtSetpoint, now);
            } else if (now - stateStart >= travelTimeoutMs) {
                travelTimeout(now);
            }
            break;
            

        case State::AtSetpoint:
            // start the dwell only after fresh feedback says we actually got there.
            if (now - stateStart >= dwellTimeMs) {
                advanceSetpointOrFinish(now);
            }
            break;

        case State::Finished:
        case State::TravelTimeout:
            break;
    }

    // keep refreshing the position command after the test starts. once we finish
    // or time out, the repeated command is intentionally just the closed position.
    if (state != State::Waiting &&
        now - lastCommand >= controlIntervalMs) {
        commandTarget(now);
    }

    /**
     * log the whole characterization at 25 hz with the shared logger. keeping one
     * logger and a fixed sample rate makes the later cda analysis a lot cleaner,
     * especially when we compare pressure data across valve angles. this follows
     * the same general workflow as the ereg paper: get the valve characterization
     * data first, then use that measured relationship in the control model later.
     */
    if (state != State::Finished && state != State::TravelTimeout &&
        now - lastLog >= Config::LOG_FREQUENCY_25_HZ) {
        lastLog += Config::LOG_FREQUENCY_25_HZ;
        // if we miss a sample, skip it instead of dumping a burst of old timestamps.
        if (now - lastLog >= Config::LOG_FREQUENCY_25_HZ) {
            lastLog = now;
        }
        logRow(&motorState, &ptUp, &ptDown, now, stateName());
    }
}
