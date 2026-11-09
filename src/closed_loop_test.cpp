/**
 * Runs the feedforward thrust-profile test with chamber-pressure feedback.
 * Thrust selects simulated chamber pressure; PT2 supplies measured pressure.
 * PID adds a motor-position correction to the liquid model's valve angle.
 * Gains and I limits are configured in PID.
 */
#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <mcp2515.h>

#include "config.h"
#include "feedforward.h"
#include "log.h"
#include "motor_state.h"
#include "pid.h"
#include "thrust_profiles.h"
#include "utils.h"

namespace {
// Select the profile and its total duration together, as in feedforward_test.
// constexpr unsigned long kTestLengthMs = 15500;
// constexpr auto& kThrustProfileLbf = ThrustProfile::kRampUpThrust;
constexpr unsigned long kTestLengthMs = 29700;
constexpr auto& kThrustProfileLbf = ThrustProfile::kFlightProfile;
constexpr size_t kThrustProfileLength =
    sizeof(kThrustProfileLbf) / sizeof(kThrustProfileLbf[0]);
constexpr unsigned long kSetpointDurationMs = kTestLengthMs / kThrustProfileLength;
constexpr unsigned long kCloseTimeoutMs = 3000;

// Water density at ~75 F.
constexpr double kFluidDensityLbmPerFt3 = 62.26;

MCP2515 mcp2515(Config::kCanCsPin, 4000000UL);
CanControl::SparkMax motor(mcp2515, Config::kLoxMotorId);
MotorState motorState(Config::kLoxMotorId);
PressureTransducer pt1(Config::kPt1InputPin, Config::kPt1MaxPsi, Config::kPtMinVoltage, Config::kPtMaxVoltage);
PressureTransducer pt2(Config::kPt2InputPin, Config::kPt2MaxPsi, Config::kPtMinVoltage, Config::kPtMaxVoltage);
PidController chamberPressurePid(PID::kKp, PID::kKi, PID::kKd,
                                 PID::kMinIContribution, PID::kMaxIContribution);

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
uint32_t previousControlStartUs = 0;
bool hasPreviousControlStart = false;
uint32_t controlErrors = 0;
uint32_t heartbeatErrors = 0;
bool finalReportSent = false;
const __FlashStringHelper* failureReason = nullptr;

const __FlashStringHelper* state_name() {
    switch (testState) {
        // Retain the model-running state ID so the logger and decoder stay unchanged.
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
    const double upstreamPressurePsi = pt1.get_pressure(false);
    for (size_t index = 0; index < kThrustProfileLength; ++index) {
        if (!Feedforward::set_thrust_to_mdot(
                kThrustProfileLbf[index], upstreamPressurePsi).valid ||
            !isfinite(utils::chamber_pressure_for_thrust(kThrustProfileLbf[index]))) {
            fail_test(F("Invalid engine table, thrust range, or upstream pressure"));
            break;
        }
    }

    chamberPressurePid.reset();
    hasPreviousControlStart = false;
    begin_control_schedule();
    setpointStartTimeMs = millis();
}

void loop() {
    begin_control_cycle();
    // Actual cycle spacing includes stalls; unsigned subtraction handles micros()
    // rollover. The first cycle has no elapsed interval and contributes no PID.
    const uint32_t controlStartUs = controlSchedule.previousStartUs;
    const double dtSeconds = hasPreviousControlStart ?
        static_cast<uint32_t>(controlStartUs - previousControlStartUs) * 1e-6 : 0.0;
    previousControlStartUs = controlStartUs;
    hasPreviousControlStart = true;

    // reads incoming CAN frames. up to 8 per cycle to not overunn time budget
    struct can_frame frame;
    for (uint8_t count = 0; count < 8 &&
            mcp2515.readMessage(&frame) == MCP2515::ERROR_OK; ++count) {
        motor.handle_received_frame(frame);
        motorState.process_frame(frame);
    }

    const unsigned long now = millis();
    double targetMotorRotations = Config::kClosedRotations;
    double commandedThrustLbf = 0.0;
    if (!finalReportSent && sdLog.has_error()) fail_test(F("SD write failed"));

    switch (testState) {
        case TestState::InProgress: {
            while (now - setpointStartTimeMs >= kSetpointDurationMs) {
                setpointStartTimeMs += kSetpointDurationMs;
                if (++thrustSetpointIndex >= kThrustProfileLength) {
                    testState = TestState::Closing;
                    closingStartTimeMs = now;
                    closingStatusTimeMs = motorState.get_last_status_2_time();
                    break;
                }
            }
            if (testState != TestState::InProgress) break;

            commandedThrustLbf = kThrustProfileLbf[thrustSetpointIndex];
            const double upstreamPressurePsi = pt1.get_pressure(false);
            const Feedforward::ThrustTargets targets = Feedforward::set_thrust_to_mdot(
                commandedThrustLbf, upstreamPressurePsi);
            const double goalChamberPressurePsi = utils::chamber_pressure_for_thrust(commandedThrustLbf);
            if (!targets.valid || !isfinite(goalChamberPressurePsi)) {
                fail_test(F("Invalid engine target or upstream pressure"));
                break;
            }

            // Match the LOX opening using water, keeping the same pressure targets.
            const double testMassFlowLbmPerSec = Feedforward::equivalent_mass_flow(
                targets.massFlowLbmPerSec, Engine::kLoxDensityLbmPerFt3,
                kFluidDensityLbmPerFt3);
            const double angleDeg = Feedforward::calculate_feedforward(
                testMassFlowLbmPerSec, upstreamPressurePsi,
                targets.downstreamPressurePsi, kFluidDensityLbmPerFt3);
            if (!isfinite(angleDeg) || angleDeg < Config::kClosedTheta ||
                angleDeg > Config::kOpenTheta) {
                fail_test(F("Pressure drop or required Cv is outside the model range"));
                break;
            }
            // Feedforward outlet pressure is distinct from the chamber pressure
            // target. Convert its valve angle to motor rotations before adding PID.
            const double feedforwardMotorRotations = angleDeg * Config::kGearRatio / 360.0;

            const double measuredChamberPressurePsi = pt2.get_pressure(false);
            
            // calculate PID angle correction
            const double correction = isfinite(measuredChamberPressurePsi) ?
                chamberPressurePid.update(goalChamberPressurePsi, measuredChamberPressurePsi, dtSeconds) : NAN;
            if (!isfinite(correction)) {
                fail_test(F("Invalid chamber pressure or correction"));
                break;
            }
            
            // add feedforward and PID output, clamp output target between 0 and 90 degrees
            targetMotorRotations = constrain(feedforwardMotorRotations + correction,
                Config::kClosedRotations, Config::kOpenRotations);
            break;
        }
        case TestState::Closing:
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

    if (testState != TestState::InProgress) {
        chamberPressurePid.reset();
    }

    //apply backlash compensation
    const double backlashOffset = utils::backlashCompensation(motorState, targetMotorRotations, false);
    const bool backlashCompensated = testState == TestState::InProgress && backlashOffset != 0.0;
    const float finalMotorCommand =
        targetMotorRotations + (backlashCompensated ? backlashOffset : 0.0);
    
    // send final position command to motor. feedforward + PID + backlash compensation
    if (motor.set_position(finalMotorCommand) != MCP2515::ERROR_OK) ++controlErrors;
    if (CanControl::send_heartbeat(mcp2515, CanControl::default_heartbeat()) !=
            MCP2515::ERROR_OK) ++heartbeatErrors;

    const bool testFinished = (testState == TestState::Done) || (testState == TestState::Failed);
    log_row(&motorState, &pt1, &pt2, millis(), state_name(), finalMotorCommand,
            testFinished && !finalReportSent, backlashCompensated, commandedThrustLbf);
    finish_control_cycle();

    if (testFinished && !finalReportSent) {
        finalReportSent = true;
        Serial.print(F("CLOSED_LOOP: state=")); Serial.print(state_name());
        Serial.print(F(" control_errors=")); Serial.print(controlErrors);
        Serial.print(F(" heartbeat_errors=")); Serial.println(heartbeatErrors);
        if (failureReason) {
            Serial.print(F("CLOSED_LOOP_ERROR: "));
            Serial.println(failureReason);
        }
        report_log_status(Serial);
    }
}
