/**
 * this test runs the valve through a configurable set of angles, waits at each
 * one for a little bit, and logs the pressure and motor data each 175 Hz cycle before
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
// constexpr double kStartAngleDeg = Config::kClosedTheta;
constexpr double kStartAngleDeg = 0;
constexpr double kEndAngleDeg = Config::kOpenTheta;
constexpr double kAngleIncrementDeg = 10.0;
constexpr unsigned long kDwellTimeMs = 3000;

// this is how long a commanded move can take before the state machine gives up
// and commands the valve closed.
constexpr unsigned long kTravelTimeoutMs = 3000;

MCP2515 mcp2515(Config::kCanCsPin, 4000000UL);
CanControl::SparkMax motor(mcp2515, Config::kLoxMotorId);
MotorState motorState(Config::kLoxMotorId);
PressureTransducer pt1(Config::kPt1InputPin, Config::kPt1MaxPsi, Config::kPtMinVoltage, Config::kPtMaxVoltage);
PressureTransducer pt2(Config::kPt2InputPin, Config::kPt2MaxPsi, Config::kPtMinVoltage, Config::kPtMaxVoltage);

enum class State {
    Waiting,
    Moving,
    AtSetpoint,
    Finished,
    TravelTimeout
};

State state = State::Waiting;

unsigned long stateStart = 0;
bool finalReportSent = false;

bool positionReceivedDuringTravel = false;
bool descendingSweep = false;
double targetAngleDeg = kStartAngleDeg;
double targetPositionRotations = Config::kClosedRotations;

constexpr double kAngleEpsilonDeg = 1e-6;

static_assert(kAngleIncrementDeg > 0.0,
              "CDA angle increment must be positive");
static_assert(kStartAngleDeg >= Config::kClosedTheta,
              "CDA starting angle cannot be below the closed angle");
static_assert(kEndAngleDeg <= Config::kOpenTheta,
              "CDA ending angle cannot exceed the fully-open angle");
static_assert(kStartAngleDeg <= kEndAngleDeg,
              "CDA starting angle must not exceed the ending angle");

void enter_state(State next, unsigned long now) {
    state = next;
    stateStart = now;
}

// starts a position move and makes sure we see fresh encoder data after the
// command before calling it done, so stale telemetry cannot fake a setpoint hit.
void begin_travel(State next, double valveAngleDeg, unsigned long now) {
    targetAngleDeg = valveAngleDeg;
    targetPositionRotations = (valveAngleDeg / 360.0) * Config::kGearRatio;
    positionReceivedDuringTravel = false;
    enter_state(next, now);
}

const __FlashStringHelper* state_name() {
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
bool is_position_frame(const struct can_frame& frame) {
    namespace Spark = CanControl::LowLevel::SparkMax;

    if (frame.can_id & CAN_RTR_FLAG) return false;
    if (!(frame.can_id & CAN_EFF_FLAG)) return false;

    const uint32_t id = frame.can_id & CAN_EFF_MASK;
    const uint8_t receivedMotorId = id & Spark::SPARK_DEVICE_ID_MASK;
    if (receivedMotorId != Config::kLoxMotorId) return false;

    const uint32_t baseId = id & ~Spark::SPARK_DEVICE_ID_MASK;
    return baseId == Spark::SPARK_ARB_STATUS_2;
}

void travel_timeout(unsigned long now) {
    // this state means the move timed out, not necessarily that the motor itself
    // froze. keep the fail-safe behavior and command the valve back closed.
    targetAngleDeg = Config::kClosedTheta;
    targetPositionRotations = Config::kClosedRotations;
    enter_state(State::TravelTimeout, now);
}

void finish_test(unsigned long now) {
    targetAngleDeg = Config::kClosedTheta;
    targetPositionRotations = Config::kClosedRotations;

    enter_state(State::Finished, now);
}

void advance_setpoint_or_finish(unsigned long now) {
    if (!descendingSweep) {
        // keep stepping upward until we reach the fully open setpoint.
        if (targetAngleDeg <= kEndAngleDeg - kAngleIncrementDeg) {
            double nextAngle = targetAngleDeg + kAngleIncrementDeg;

            begin_travel(State::Moving, nextAngle, now);
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
        double nextAngle = targetAngleDeg - kAngleIncrementDeg;

        if (nextAngle < kStartAngleDeg) {
            nextAngle = kStartAngleDeg;
        }

        begin_travel(State::Moving, nextAngle, now);
        return;
    }

    // keep stepping downward until we return to the starting angle.
    if (targetAngleDeg > kStartAngleDeg + kAngleEpsilonDeg) {
        double nextAngle = targetAngleDeg - kAngleIncrementDeg;

        if (nextAngle < kStartAngleDeg) {
            nextAngle = kStartAngleDeg;
        }

        begin_travel(State::Moving, nextAngle, now);
        return;
    }

    // After the starting angle's final dwell, finish and command closed.
    finish_test(now);
}
}  // namespace

void setup() {
    Serial.begin(Config::kBaudRateLong);

    
    //check PT values before starting. PT1 should be near tank pressure, PT2 ambient
    Serial.print("PT 1: ");
    Serial.print(pt1.get_pressure(false));
    Serial.print(" | PT 2: ");
    Serial.print(pt2.get_pressure(false));

    pinMode(Config::kSdCsPin, OUTPUT);
    digitalWrite(Config::kSdCsPin, HIGH);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalOneShotMode();

    begin_sd_log();
    wait_for_unix_time();
    begin_control_schedule();
}

void loop() {
    begin_control_cycle();
    // only chew through a few can frames per loop so a busy bus cannot hold up
    // the heartbeat, state machine, motor command refresh, or logger.
    struct can_frame frame;
    for (uint8_t count = 0;
         count < 8 && mcp2515.readMessage(&frame) == MCP2515::ERROR_OK;
         ++count) {
        motor.handle_received_frame(frame);

        if (motorState.process_frame(frame) && is_position_frame(frame)) {
            positionReceivedDuringTravel = true;
        }
    }

    const unsigned long now = millis();

    // one space starts the test and everything else gets ignored. this keeps the
    // same simple operator-start idea as the old sketch without blocking the loop.
    for (uint8_t count = 0; count < 16 && Serial.available() > 0; ++count) {
        if (Serial.read() == ' ' && state == State::Waiting) {
            begin_travel(State::Moving, kStartAngleDeg, now);
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
                motorState.at_position(targetPositionRotations,
                                      now,
                                      Config::kGoalPositionTolerance,
                                      Config::kTelemetryTimeoutMs)) {
                enter_state(State::AtSetpoint, now);
            } else if (now - stateStart >= kTravelTimeoutMs) {
                travel_timeout(now);
            }
            break;
            

        case State::AtSetpoint:
            // start the dwell only after fresh feedback says we actually got there.
            if (now - stateStart >= kDwellTimeMs) {
                advance_setpoint_or_finish(now);
            }
            break;

        case State::Finished:
        case State::TravelTimeout:
            break;
    }

    // Send once per cycle after calculating the target. Waiting sends no position
    // command; finished and timed-out tests keep commanding closed.
    if (state != State::Waiting) {
        motor.set_position(targetPositionRotations, 0, 0, 0);
    }
    CanControl::send_heartbeat(mcp2515, CanControl::default_heartbeat());

    // Queue one row per cycle and flush/close exactly once at completion.
    const bool testFinished = state == State::Finished || state == State::TravelTimeout;
    log_row(&motorState, &pt1, &pt2, millis(), state_name(),
            state == State::Waiting ? NAN : targetPositionRotations,
            testFinished && !finalReportSent);
    finish_control_cycle();

    if (testFinished && !finalReportSent) {
        finalReportSent = true;
        report_log_status(Serial);
    }
}
