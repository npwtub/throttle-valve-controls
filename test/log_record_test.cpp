// Host check from a Developer Command Prompt (create .pio/log_v4_validation first):
// cl /nologo /EHsc /std:c++14 /Itest/fakes/logging /Iinclude test/log_record_test.cpp /Fe.pio/log_v4_validation/log_record_test.exe /Fo.pio/log_v4_validation/log_record_test.obj
// .pio/log_v4_validation/log_record_test.exe [optional-output.bin]
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "log.h"

static bool near(float actual, double expected) { return fabs(actual - expected) < 2e-5; }

static uint32_t read_u32(const uint8_t* bytes) {
    return uint32_t(bytes[0]) | uint32_t(bytes[1]) << 8 |
           uint32_t(bytes[2]) << 16 | uint32_t(bytes[3]) << 24;
}
static uint64_t read_u64(const uint8_t* bytes) {
    return uint64_t(read_u32(bytes)) | uint64_t(read_u32(bytes + 4)) << 32;
}
static float read_float(const uint8_t* bytes) {
    const uint32_t bits = read_u32(bytes);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void receive_motor_status(MotorState& state, uint8_t temperature = 43,
                                 int16_t output = -16384, uint16_t current = 887) {
    namespace Spark = CanControl::LowLevel::SparkMax;
    can_frame frame{};
    frame.can_id = CAN_EFF_FLAG | Spark::SPARK_ARB_STATUS_0 | Config::kLoxMotorId;
    frame.can_dlc = 8;
    const uint16_t voltage = 3276;
    memcpy(frame.data, &output, 2);
    memcpy(frame.data + 2, &voltage, 2);
    memcpy(frame.data + 4, &current, 2);
    frame.data[6] = temperature;
    assert(state.process_frame(frame));

    frame.can_id = CAN_EFF_FLAG | Spark::SPARK_ARB_STATUS_2 | Config::kLoxMotorId;
    const float position = static_cast<float>(Config::kGearRatio / 4.0);
    const float rpm = -1234.5f;
    memcpy(frame.data, &position, 4);
    memcpy(frame.data + 4, &rpm, 4);
    assert(state.process_frame(frame));
}

int main(int argc, char** argv) {
    static_assert(sizeof(LogRecord) == 64, "The wire record must remain 64 bytes");
    static_assert(logFormatVersion == 4, "Update this wire-layout test for new versions");
    MotorState motor(Config::kLoxMotorId);
    receive_motor_status(motor);
    PressureTransducer pt1(Config::kPt1InputPin, Config::kPt1MaxPsi,
                           Config::kPtMinVoltage, Config::kPtMaxVoltage);
    PressureTransducer pt2(Config::kPt2InputPin, Config::kPt2MaxPsi,
                           Config::kPtMinVoltage, Config::kPtMaxVoltage);
    PressureTransducer pt3(Config::kPt3InputPin, Config::kPt3MaxPsi,
                           Config::kPtMinVoltage, Config::kPtMaxVoltage);
    fakeAnalog[Config::kPt1InputPin] = 700;
    fakeAnalog[Config::kPt2InputPin] = 400;
    fakeAnalog[Config::kPt3InputPin] = 300;
    const float expectedPt1 = pt1.get_pressure(false);
    const float expectedPt2 = pt2.get_pressure(false);
    const float chamberSample = pt3.get_pressure(false);
    const unsigned chamberReads = fakeAnalogReads[Config::kPt3InputPin];
    // A later ADC value must not replace the exact pressure used for PID.
    fakeAnalog[Config::kPt3InputPin] = 900;

    PidController pid(0.2, 0.1, 0.05, -0.25, 0.25);
    pid.update(chamberSample + 10.0, chamberSample - 2.0, 0.25);
    const double output = pid.update(chamberSample + 10.0, chamberSample, 0.5);
    assert(near(static_cast<float>(output), 2.05)); // P=2, bounded I=.25, D=-.2.
    LogControlData control;
    control.pt3PressurePsi = &chamberSample;
    control.pidOutputClamped = true;
    LogRecord record;
    make_log_record(record.bytes, &motor, &pt1, &pt2, &pt3,
                    1792540800123ULL, 15, static_cast<float>(Config::kGearRatio / 4.0),
                    0x12345678, true, 400.0f, true, &pid, &control);
    assert(record.bytes[0] == 4 && record.bytes[1] == 15);
    assert(record.bytes[2] == 0xff && record.bytes[3] == 0x0f);
    assert(read_u64(record.bytes + 4) == 1792540800123ULL);
    assert(read_float(record.bytes + 12) == expectedPt1);
    assert(read_float(record.bytes + 16) == expectedPt2);
    assert(read_float(record.bytes + 20) == chamberSample);
    assert(fakeAnalogReads[Config::kPt3InputPin] == chamberReads);
    assert(near(read_float(record.bytes + 24), 90.0));
    assert(record.bytes[28] == 43);
    assert(static_cast<int8_t>(record.bytes[29]) == -50);
    assert(record.bytes[30] == 32 && record.bytes[31] == 0);
    assert(read_float(record.bytes + 32) == -1234.5f);
    assert(near(read_float(record.bytes + 36), 24.0));
    assert(near(read_float(record.bytes + 40), 90.0));
    assert(read_u32(record.bytes + 44) == 0x12345678);
    assert(read_float(record.bytes + 48) == 400.0f);
    const double degreesPerRotation = 360.0 / Config::kGearRatio;
    assert(near(read_float(record.bytes + 52), 2.0 * degreesPerRotation));
    assert(near(read_float(record.bytes + 56), 0.25 * degreesPerRotation));
    assert(near(read_float(record.bytes + 60), -0.2 * degreesPerRotation));
    assert(pid.last_output() == output); // Logging copies the last result.
    assert(pid.last_i_output() == 0.25);
    assert(pid.last_d_output() == -0.2);

    if (argc > 1) {
        FILE* file = fopen(argv[1], "wb");
        assert(file);
        assert(fwrite(record.bytes, 1, sizeof(record.bytes), file) == 64);
        assert(fclose(file) == 0);
    }

    // Omitting the context samples PT3 once; omitting PID records no PID flags.
    make_log_record(record.bytes, &motor, nullptr, nullptr, &pt3, 0, 0, NAN, 0);
    assert(record.bytes[2] == (4 | 32));
    assert(fakeAnalogReads[Config::kPt3InputPin] == chamberReads + 1);
    assert(read_float(record.bytes + 20) == pt3.get_pressure(false));
    assert(isnan(read_float(record.bytes + 40)));
    for (unsigned i = 52; i < 64; ++i) assert(record.bytes[i] == 0);

    // Rounded whole-unit values saturate instead of wrapping into the sentinel.
    assert(log_signed_unit(12.49f, -127, 127) == 12);
    assert(log_signed_unit(12.5f, -127, 127) == 13);
    assert(static_cast<int8_t>(log_signed_unit(-12.5f, -127, 127)) == -13);
    assert(static_cast<int8_t>(log_signed_unit(-500.0f, -127, 127)) == -127);
    assert(log_signed_unit(500.0f, -127, 127) == 127);
    assert(log_signed_unit(NAN, -127, 127) == 0x80);
    assert(log_signed_unit(INFINITY, -100, 100) == 0x80);
    assert(log_current_amps(32.49f) == 32 && log_current_amps(32.5f) == 33);
    assert(log_current_amps(-1.0f) == 0 && log_current_amps(1000.0f) == 254);
    assert(log_current_amps(NAN) == 255 && log_current_amps(INFINITY) == 255);
    receive_motor_status(motor, 200, -32768, 65535);
    make_log_record(record.bytes, &motor, nullptr, nullptr, nullptr, 0, 0, 0, 0);
    assert(record.bytes[28] == 127);
    assert(static_cast<int8_t>(record.bytes[29]) == -100);
    assert(record.bytes[30] == 254 && record.bytes[31] == 0);

    // Neither reset nor an invalid PID update may replay a previous snapshot.
    pid.reset();
    make_log_record(record.bytes, nullptr, nullptr, nullptr, nullptr,
                    0, 0, 0, 0, false, 0, false, &pid, &control);
    assert(record.bytes[2] == 0);
    for (unsigned i = 12; i < 64; ++i) assert(record.bytes[i] == 0);
    pid.update(NAN, chamberSample, 0.5);
    make_log_record(record.bytes, nullptr, nullptr, nullptr, nullptr,
                    0, 0, 0, 0, false, 0, false, &pid, &control);
    assert(record.bytes[2] == 0);
    for (unsigned i = 52; i < 64; ++i) assert(record.bytes[i] == 0);
    puts("V4 binary writer tests passed");
}
