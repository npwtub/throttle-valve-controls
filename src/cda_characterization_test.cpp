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
// everything below here is specific to this characterization test, so keeping it
// local avoids filling the shared config with knobs that no other test uses.
constexpr uint8_t motorID = Config::LOX_MOTOR_ID;

/**
 * these pressure transducer numbers come straight from the old characterization
 * sketch, so this keeps the new test on the same sensor calibration for now.
 * the bigger reason we care about clean pressure measurements is the same one
 * shown in the ereg paper's water-flow characterization: pressure/flow data at
 * known valve positions is what lets us turn the hardware into a useful valve
 * model instead of just guessing how open the valve really is from angle alone.
 */
constexpr float ptMaxPsi = 600.0f;
constexpr double ptSenseResistanceOhms = 150.0;
constexpr double ptMinVoltage = 0.00425 * ptSenseResistanceOhms;
constexpr double ptMaxVoltage = 0.02000 * ptSenseResistanceOhms;

/**
 * this is the actual characterization sweep. start angle, end angle, step size,
 * and dwell time are all easy to tweak here depending on how much resolution
 * we want. the ereg paper did the same kind of angle-to-flow characterization
 * before building its feedforward model, while our later analysis will use the
 * logged test data to build the cda vs angle relationship for this valve.
 */
constexpr double startAngleDeg = Config::CLOSED_THETA;
constexpr double endAngleDeg = Config::OPEN_THETA;
constexpr double angleIncrementDeg = 5.0;
constexpr unsigned long dwellTimeMs = 1000;

// the old arduino test waited ten seconds after start, so keep that behavior
// here but leave it easy to change if the test procedure changes.
constexpr unsigned long startDelayMs = 10000;

// motor commands and heartbeats are refreshed every 20 ms, same basic timing
// used by the other tests.
constexpr unsigned long controlIntervalMs = 20;

// this is how long a commanded move can take before the state machine gives up
// and commands the valve closed.
constexpr unsigned long travelTimeoutMs = 3000;



MCP2515 mcp2515(Config::CAN_CS_PIN, 4000000UL);
CanControl::SparkMax motor(mcp2515, motorID);
MotorState motorState(motorID);
PressureTransducer ptUp(A0, ptMaxPsi, ptMinVoltage, ptMaxVoltage);
PressureTransducer ptDown(A1, ptMaxPsi, ptMinVoltage, ptMaxVoltage);

enum class State {
    Waiting,
    StartDelay,
    Opening,
    AtSetpoint,
    Closing,
    Finished,
    TravelTimeout
};

State state = State::Waiting;

unsigned long stateStart = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastCommand = 0;
unsigned long lastLog = 0;

bool positionReceivedDuringTravel = false;
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
    if (receivedMotorID != motorID) return false;

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

void advanceSetpointOrClose(unsigned long now) {
    if (targetAngleDeg < endAngleDeg - angleEpsilonDeg) {
        double nextAngle = targetAngleDeg + angleIncrementDeg;
        if (nextAngle > endAngleDeg) {
            nextAngle = endAngleDeg;
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
    if (now - lastHeartbeat >= controlIntervalMs) {
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
            if (now - stateStart >= startDelayMs) {
                beginTravel(State::Opening, startAngleDeg, now);
            }
            break;

        case State::Opening:
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
                advanceSetpointOrClose(now);
            }
            break;

        case State::Closing:
            if (positionReceivedDuringTravel &&
                motorState.atPosition(Config::CLOSED_ROTATIONS,
                                      now,
                                      Config::GOAL_POSITION_TOLERANCE,
                                      Config::TELEMETRY_TIMEOUT_MS)) {
                targetAngleDeg = Config::CLOSED_THETA;
                targetPositionRotations = Config::CLOSED_ROTATIONS;
                enterState(State::Finished, now);
                commandTarget(now);
                logRow(&motorState, &ptUp, &ptDown, now, stateName());
            } else if (now - stateStart >= travelTimeoutMs) {
                travelTimeout(now);
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
