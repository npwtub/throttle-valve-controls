/**
 * Shared CSV logger.
 * - Prints time, pressure readings, motor telemetry, and state name.
 * - Converts motor position to valve degrees.
 * - Skips sensors and motor data when their pointers are null.
 * - Keep the same inputs enabled throughout each log file.
 */
#pragma once

#include <Arduino.h>
#include "motor_state.h"
#include "pressure_transducer.h"
#include "config.h"

static void logRow(const MotorState* motorState, const PressureTransducer* pt1, const PressureTransducer* pt2, const unsigned long nowMs, const String& stateName){
    Serial.print(nowMs);
    Serial.print(',');  // Milliseconds since boot


    // log PT data if not null
    if (pt1 != nullptr){
        Serial.print(pt1->getPressure(false), 2);
        Serial.print(',');
    }
    if (pt2 != nullptr){
        Serial.print(pt2 -> getPressure(false), 2);
        Serial.print(',');
    }
    // Gear ratio is motor revolutions per valve revolution; report valve degrees.
    if (motorState != nullptr){
        Serial.print(motorState -> getPosition() * 360.0 / Config::GEAR_RATIO, 3);
        Serial.print(',');
        Serial.print(motorState -> getTemperature(), 1);
        Serial.print(',');
        Serial.print(motorState -> getAppliedOutputPercent(), 2);
        Serial.print(',');
        Serial.print(motorState -> getRPM(), 2);
        Serial.print(',');
        Serial.print(motorState -> getVoltageRaw() / 136.5);
        Serial.print(',');
        Serial.print(motorState -> getCurrentRaw() / 27.3);
        Serial.print(',');
        // These flags distinguish initial cached zeroes from received motor data.
        Serial.print(motorState -> hasStatus0());
        Serial.print(',');
        Serial.print(motorState -> hasStatus2());
        Serial.print(',');
        Serial.print(motorState -> hasTelemetry());
        Serial.print(',');
        Serial.print(motorState -> isFresh(Config::TELEMETRY_TIMEOUT_MS));
        Serial.print(',');
        Serial.print(motorState -> getLastStatus0Time());
        Serial.print(',');
        Serial.print(motorState -> getLastStatus2Time());
        Serial.print(',');
    }
    Serial.println(stateName);
}
