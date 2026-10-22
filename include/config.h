/**
 * Shared test settings.
 * - Defines serial speeds, the CAN chip select pin, and motor IDs.
 * - Sets telemetry timeout and logging interval.
 * - Converts valve angles to motor rotations.
 * - Sets the target position tolerance.
 */

#pragma once

#include <Arduino.h>

namespace Config {
    constexpr unsigned long BAUD_RATE_SHORT = 115200; // shorter cable
    constexpr unsigned long BAUD_RATE_LONG = 57600; // 50 ft cable

    // PWM output port on arduino that CAN shield connects too
    constexpr uint8_t CAN_CS_PIN = 10;
    // Adafruit #254 SD breakout chip select; wire CS to Mega digital pin 4.
    constexpr uint8_t SD_CS_PIN = 9;

    // The control period is 175 hz.
    constexpr uint32_t CONTROL_PERIOD_US = 5714;

    // Temporary estimate for one SD block write.
    // Replace with measured timing later.
    constexpr uint32_t SD_WRITE_ESTIMATE_US = 2500;

    // Leave extra time before the next control cycle.
    constexpr uint32_t SD_WRITE_MARGIN_US = 500;

    // About 175 minutes at 200 records/s, including time waiting to start.
    constexpr uint32_t SD_PREALLOCATE_BYTES = 128UL * 1024UL * 1024UL;

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
