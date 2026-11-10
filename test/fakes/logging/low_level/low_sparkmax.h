#pragma once
#include <stdint.h>
#include <string.h>

// A deterministic host frame codec for exercising the real MotorState cache.
// It is deliberately not a replacement for the production CAN protocol codec.
namespace CanControl { namespace LowLevel { namespace SparkMax {
constexpr uint32_t SPARK_DEVICE_ID_MASK = 0x3f;
constexpr uint32_t SPARK_ARB_STATUS_0 = 0x100;
constexpr uint32_t SPARK_ARB_STATUS_2 = 0x200;
struct Spark_STATUS_0_t {
    int16_t APPLIED_OUTPUT;
    uint16_t VOLTAGE, CURRENT;
    uint8_t MOTOR_TEMPERATURE;
};
struct Spark_STATUS_2_t { float PRIMARY_ENCODER_POSITION, PRIMARY_ENCODER_VELOCITY; };
inline bool spark_decode_STATUS_0(const uint8_t* data, uint8_t length, Spark_STATUS_0_t* out) {
    if (length != 8) return false;
    memcpy(&out->APPLIED_OUTPUT, data, 2);
    memcpy(&out->VOLTAGE, data + 2, 2);
    memcpy(&out->CURRENT, data + 4, 2);
    out->MOTOR_TEMPERATURE = data[6];
    return true;
}
inline bool spark_decode_STATUS_2(const uint8_t* data, uint8_t length, Spark_STATUS_2_t* out) {
    if (length != 8) return false;
    memcpy(&out->PRIMARY_ENCODER_POSITION, data, 4);
    memcpy(&out->PRIMARY_ENCODER_VELOCITY, data + 4, 4);
    return true;
}
}}}
