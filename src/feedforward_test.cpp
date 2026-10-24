/**
 * Runs a thrust profile using the liquid-valve feedforward model.
 * - PT1 measures pressure upstream of the throttle; PT2 is logged only.
 * - Each thrust setpoint selects LOX flow and valve pressure drop from Engine.
 * - The downstream-pressure target is live PT1 minus that pressure drop.
 * - The feedforward helper converts those targets into a valve angle.
 * - Commands closed after the profile finishes or the model rejects a request.
 * - Starts after the Unix-time handshake, as in the original test.
 */
#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <mcp2515.h>

#include "config.h"
#include "feedforward.h"
#include "log.h"
#include "motor_state.h"

namespace {
// test will follow thrust profile setpoints, splitting each one equally
// along to total length of the test
constexpr unsigned long kTestLengthMs = 15000;
constexpr double kThrustProfileLbf[] = {430};
constexpr size_t kThrustProfileLength =
    sizeof(kThrustProfileLbf) / sizeof(kThrustProfileLbf[0]);
constexpr unsigned long kSetpointDurationMs = kTestLengthMs / kThrustProfileLength;
constexpr unsigned long kCloseTimeoutMs = 3000;

// water density at ~75 F
constexpr double kFluidDensityLbmPerFt3 = 62.26;

MCP2515 mcp2515(Config::kCanCsPin, 4000000UL);
CanControl::SparkMax motor(mcp2515, Config::kLoxMotorId);
MotorState motorState(Config::kLoxMotorId);
PressureTransducer pt1(A2, 1000, 0.0135, 5.0);
PressureTransducer pt2(A3, 1000, 0.0135, 5.0);

enum class TestState {
    InProgress,
    Closing,
    Done,
    Failed
};

TestState testState = TestState::InProgress;
size_t thrustSetpointIndex = 0;
unsigned long setpointStartTimeMs = 0;
unsigned long closingStartTimeMs = 0;
unsigned long closingStatusTimeMs = 0;
unsigned long lastPtPrint = 0;
uint32_t controlErrors = 0;
uint32_t heartbeatErrors = 0;
bool finalReportSent = false;
const __FlashStringHelper* failureReason = nullptr;

const __FlashStringHelper* state_name() {
    switch (testState) {
        case TestState::InProgress: return F("feedforward");
        case TestState::Closing: return F("closing");
        case TestState::Done: return F("done");
        case TestState::Failed: return F("failed");
    }
    return F("unknown");
}

void fail_test(const __FlashStringHelper* reason) {
    testState = TestState::Failed;
    if (!failureReason) failureReason = reason;
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

    if (begin_sd_log()) {
        wait_for_unix_time();
    } else {
        fail_test(F("SD initialization failed"));
    }
    // checks the whole thrust profile before starting test
    const double upstreamPressurePsi = pt1.get_pressure(false);
    for (size_t index = 0; index < kThrustProfileLength; ++index) {
        if (!Feedforward::set_thrust_to_mdot(
                kThrustProfileLbf[index], upstreamPressurePsi).valid) {
            fail_test(F("Invalid engine table, thrust range, or upstream pressure"));
            break;
        }
    }

    begin_control_schedule();
    setpointStartTimeMs = millis();
}

void loop() {
    begin_control_cycle();
    struct can_frame frame;
    
    // read incoming CAN frames. up to 8 per cycle
    for (uint8_t count = 0; count < 8 &&
            mcp2515.readMessage(&frame) == MCP2515::ERROR_OK; ++count) {
        motor.handle_received_frame(frame);
        motorState.process_frame(frame);
    }

    const unsigned long now = millis();
    double outputAngleDeg = Config::kClosedTheta;
    double commandedThrustLbf = 0.0;
    if (!finalReportSent && sdLog.has_error()) fail_test(F("SD write failed"));

    switch (testState) {
        case TestState::InProgress: {
            // Advance by milliseconds and skip elapsed setpoints after a stall.
            while (now - setpointStartTimeMs >= kSetpointDurationMs) {
                setpointStartTimeMs += kSetpointDurationMs;
                // ends test if previous setpoint was the last. goal angle is left at default 0
                if (++thrustSetpointIndex >= kThrustProfileLength) {
                    testState = TestState::Closing;
                    closingStartTimeMs = now;
                    closingStatusTimeMs = motorState.get_last_status_2_time();
                    break;
                }
            }
            if (testState != TestState::InProgress) break;

            // calculate target mdot and dp with target thrust and upstream pressure
            commandedThrustLbf = kThrustProfileLbf[thrustSetpointIndex];
            const double upstreamPressurePsi = pt1.get_pressure(false);
            const Feedforward::ThrustTargets targets = Feedforward::set_thrust_to_mdot(
                commandedThrustLbf, upstreamPressurePsi);
            if (!targets.valid) {
                fail_test(F("Invalid engine target or upstream pressure"));
                break;
            }

            // get feedforward output
            const double angleDeg = Feedforward::calculate_feedforward(
                targets.massFlowLbmPerSec, upstreamPressurePsi,
                targets.downstreamPressurePsi, kFluidDensityLbmPerFt3);
            if (!isfinite(angleDeg) || angleDeg < Config::kClosedTheta ||
                angleDeg > Config::kOpenTheta) {
                fail_test(F("Pressure drop or required Cv is outside the model range"));
                break;
            }
            outputAngleDeg = angleDeg;
            break;
        }
        case TestState::Closing:
            // Require feedback received after entering Closing, not cached data.
            if (now - closingStartTimeMs >= kCloseTimeoutMs) {
                fail_test(F("Timed out waiting for closed position"));
            } else if (motorState.get_last_status_2_time() != closingStatusTimeMs &&
                       motorState.at_position(Config::kClosedRotations, now,
                           Config::kGoalPositionTolerance, Config::kTelemetryTimeoutMs)) {
                testState = TestState::Done;
            }
            break;
        case TestState::Done:
        case TestState::Failed:
            break;
    }

    // Both the motor API and the logger take MOTOR ROTATIONS, not valve degrees.
    const float targetMotorRotations = outputAngleDeg * Config::kGearRatio / 360.0;
    if (motor.set_position(targetMotorRotations) != MCP2515::ERROR_OK) ++controlErrors;
    if (CanControl::send_heartbeat(mcp2515, CanControl::default_heartbeat()) !=
            MCP2515::ERROR_OK) ++heartbeatErrors;

    const bool testFinished = (testState == TestState::Done) || (testState == TestState::Failed);
    log_row(&motorState, &pt1, &pt2, millis(), state_name(), targetMotorRotations,
            testFinished && !finalReportSent, commandedThrustLbf);
    finish_control_cycle();

    if (testFinished && !finalReportSent) {
        finalReportSent = true;
        Serial.print(F("FEEDFORWARD: state=")); Serial.print(state_name());
        Serial.print(F(" control_errors=")); Serial.print(controlErrors);
        Serial.print(F(" heartbeat_errors=")); Serial.println(heartbeatErrors);
        if (failureReason) {
            Serial.print(F("FEEDFORWARD_ERROR: "));
            Serial.println(failureReason);
        }
        report_log_status(Serial);
    }
}

