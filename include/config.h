/**
 * Shared test settings.
 * - Defines serial speeds, the CAN chip select pin, and motor IDs.
 * - Sets telemetry timeout and logging interval.
 * - Converts valve angles to motor rotations.
 * - Sets the target position tolerance.
 */
#include <Arduino.h>

namespace Config {
    constexpr unsigned long BAUD_RATE_SHORT = 115200; // shorter cable
    constexpr unsigned long BAUD_RATE_LONG = 57600; // 50 ft cable

    // PWM output port on arduino that CAN shield connects too
    constexpr uint8_t CAN_CS_PIN = 10;

    constexpr uint8_t LOX_MOTOR_ID = 1;
    constexpr uint8_t FUEL_MOTOR_ID = 2;

    // determines when telemetry becomes stale
    constexpr unsigned long TELEMETRY_TIMEOUT_MS = 250;
    constexpr unsigned long LOG_FREQUENCY_25_HZ = 40; 

    constexpr double GEAR_RATIO = 32.0017;
    constexpr double OPEN_THETA = 90.0;
    constexpr double CLOSED_THETA = 0.0;
    constexpr double OPEN_ROTATIONS = (OPEN_THETA / 360.0) * GEAR_RATIO;
    constexpr double CLOSED_ROTATIONS = (CLOSED_THETA / 360.0) * GEAR_RATIO;

    constexpr double GOAL_POSITION_TOLERANCE = 0.20; // motor rotations away from goal position
}  
