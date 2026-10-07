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

    // these are the cda test knobs. keeping them here means we can change the
    // test setup without digging through the state machine every time.
    constexpr uint8_t CDA_MOTOR_ID = LOX_MOTOR_ID;

    /**
     * these pressure transducer numbers come straight from the old characterization
     * sketch, so this keeps the new test on the same sensor calibration for now.
     * the bigger reason we care about clean pressure measurements is the same one
     * shown in the ereg paper's water-flow characterization: pressure/flow data at
     * known valve positions is what lets us turn the hardware into a useful valve
     * model instead of just guessing how open the valve really is from angle alone.
     */
    constexpr float CDA_PT_MAX_PSI = 600.0f;
    constexpr double CDA_PT_SENSE_RESISTANCE_OHMS = 150.0;
    constexpr double CDA_PT_MIN_VOLTAGE = 0.00425 * CDA_PT_SENSE_RESISTANCE_OHMS;
    constexpr double CDA_PT_MAX_VOLTAGE = 0.02000 * CDA_PT_SENSE_RESISTANCE_OHMS;

    /**
     * this is the actual characterization sweep. start angle, end angle, step size,
     * and dwell time are all easy to tweak here depending on how much resolution
     * we want. the ereg paper did the same kind of angle-to-flow characterization
     * before building its feedforward model, while our later analysis will use the
     * logged test data to build the cda vs angle relationship for this valve.
     */
    constexpr double CDA_START_ANGLE_DEG = 0.0;
    constexpr double CDA_END_ANGLE_DEG = OPEN_THETA;
    constexpr double CDA_ANGLE_INCREMENT_DEG = 5.0;
    constexpr unsigned long CDA_DWELL_TIME_MS = 1000;

    // the old arduino test waited ten seconds after start, so keep that behavior
    // here but leave it as a config value in case the test procedure changes.
    constexpr unsigned long CDA_START_DELAY_MS = 10000;
    constexpr unsigned long CDA_CONTROL_INTERVAL_MS = 20;
    constexpr unsigned long CDA_TRAVEL_TIMEOUT_MS = 3000;

    // keep the old sketch's 0.5 degree valve-angle tolerance for now.
    constexpr double CDA_ANGLE_TOLERANCE_DEG = 0.5;
    constexpr double CDA_POSITION_TOLERANCE_ROTATIONS =
        (CDA_ANGLE_TOLERANCE_DEG / 360.0) * GEAR_RATIO;
}  
