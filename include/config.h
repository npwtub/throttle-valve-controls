#include <Arduino.h>

namespace Config {
    constexpr unsigned long BAUD_RATE_SHORT = 115200;
    constexpr unsigned long BAUD_RATE_LONG = 57600;

    constexpr uint8_t CAN_CS_PIN = 10;

    constexpr uint8_t LOX_MOTOR_ID = 1;

    constexpr uint8_t FUEL_MOTOR_ID = 2;

    constexpr unsigned long TELEMETRY_INTERVAL_MS = 100;
    constexpr unsigned long TELEMETRY_TIMEOUT_MS = 250;

    constexpr double GEAR_RATIO = 32.0017;
    constexpr double OPEN_THETA = 90.0;
    constexpr double CLOSED_THETA = 0.0;
    constexpr double OPEN_ROTATIONS = (OPEN_THETA / 360.0) * GEAR_RATIO;
    constexpr double CLOSED_ROTATIONS = (CLOSED_THETA / 360.0) * GEAR_RATIO;

}  