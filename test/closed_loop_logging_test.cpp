// Build with the same include paths as log_record_test.cpp.
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../src/closed_loop_test.cpp"

static float record_float(unsigned offset) {
    float value;
    memcpy(&value, loggingTestDisk.data.data() + offset, sizeof(value));
    return value;
}

static void set_pressure(int pin, double pressure, double fullScale) {
    fakeAnalog[pin] = static_cast<int>(round((pressure *
        (Config::kPtMaxVoltage - Config::kPtMinVoltage) / fullScale +
        Config::kPtMinVoltage) * 1023.0 / 4.988));
}

static void prepare_case(double kp, double chamberPressure) {
    fakeUs = 1000000;
    loggingTestDisk = LoggingTestDisk{};
    sdLog = SdLog{};
    assert(sdLog.begin());
    begin_control_schedule();
    motorState = MotorState(Config::kLoxMotorId);
    chamberPressurePid = PidController(kp, 0.0, 0.0, -1.0, 1.0);
    pt3 = PressureTransducer(Config::kPt3InputPin, Config::kPt3MaxPsi,
                            Config::kPtMinVoltage, Config::kPtMaxVoltage);
    set_pressure(Config::kPt1InputPin, 500.0, Config::kPt1MaxPsi);
    // Deliberately different: PT2 must not be used for chamber feedback.
    set_pressure(Config::kPt2InputPin, 900.0, Config::kPt2MaxPsi);
    set_pressure(Config::kPt3InputPin, chamberPressure, Config::kPt3MaxPsi);
    testState = TestState::InProgress;
    thrustSetpointIndex = 0;
    setpointStartTimeMs = millis();
    closingStartTimeMs = millis();
    closingStatusTimeMs = motorState.get_last_status_2_time();
    finalReportSent = false;
    failureReason = nullptr;
    hasPreviousControlStart = true;
    previousControlStartUs = fakeUs - Config::kControlPeriodUs;
    commandsSent = heartbeatsSent = 0;
    memset(fakeAnalogReads, 0, sizeof(fakeAnalogReads));
}

static uint8_t run_and_read_flags() {
    loop();
    assert(sdLog.close());
    assert(loggingTestDisk.data.size() == 64);
    assert(commandsSent == 1 && heartbeatsSent == 1);
    assert(fakeAnalogReads[Config::kPt3InputPin] == 1);
    return loggingTestDisk.data[2];
}

int main() {
    const double goal = utils::chamber_pressure_for_thrust(kThrustProfileLbf[0]);
    prepare_case(0.01, goal - 10.0);
    const double measured = pt3.get_pressure(false);
    memset(fakeAnalogReads, 0, sizeof(fakeAnalogReads));
    uint8_t flags = run_and_read_flags();
    assert(testState == TestState::InProgress);
    assert(flags & 32);  // PT3 present.
    assert(flags & 64);  // Current PID snapshot.
    assert(!(flags & 128));
    assert(fabs(record_float(20) - measured) < 1e-5);
    const double expectedP = 0.01 * (goal - measured) * 360.0 / Config::kGearRatio;
    assert(fabs(record_float(52) - expectedP) < 1e-5);
    const double sent = record_float(40) * Config::kGearRatio / 360.0 +
        ((flags & 16) ? Config::kBacklashCompensation : 0.0);
    assert(fabs(sentMotorCommand - sent) < 2e-6);

    prepare_case(100.0, 0.0);
    flags = run_and_read_flags();
    assert((flags & (64 | 128)) == (64 | 128));
    assert(fabs(record_float(40) - Config::kOpenTheta) < 1e-5);
    assert(record_float(52) > Config::kOpenTheta);  // Retain raw P when clipped.

    prepare_case(0.01, goal - 10.0);
    hasPreviousControlStart = false;
    flags = run_and_read_flags();
    assert(flags & 64);
    assert(record_float(52) == 0 && record_float(56) == 0 && record_float(60) == 0);

    prepare_case(1.0, 0.0);
    chamberPressurePid.update(goal, 0.0, 0.1);  // Old nonzero terms must disappear.
    testState = TestState::Closing;
    flags = run_and_read_flags();
    assert(!(flags & (64 | 128)));
    assert(record_float(52) == 0 && record_float(56) == 0 && record_float(60) == 0);

    prepare_case(1.0, 0.0);
    chamberPressurePid.update(goal, 0.0, 0.1);
    pt3 = PressureTransducer(Config::kPt3InputPin, NAN,
                            Config::kPtMinVoltage, Config::kPtMaxVoltage);
    flags = run_and_read_flags();
    assert(testState == TestState::Failed);
    assert(!(flags & (64 | 128)));
    assert(isnan(record_float(20)));
    assert(record_float(40) == Config::kClosedTheta);
    puts("Closed-loop PT3 feedback, sample reuse, PID snapshot, and clipping checks passed.");
}
