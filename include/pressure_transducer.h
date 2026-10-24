/**
 * Analog pressure sensor reader.
 * - Reads one analog pin.
 * - Converts sensor voltage to PSI.
 * - Uses configured voltage limits and maximum pressure.
 * - Assumes a 5 V, 10-bit ADC. Does not clamp readings.
 * - Pass false to get_pressure() to hide voltage prints.
 */
#pragma once

#include <Arduino.h>

class PressureTransducer {
   public:
    PressureTransducer(int analogPin, float maxPressurePsi, double minVoltage, double maxVoltage) : analogPin_(analogPin), minVoltage_(minVoltage), maxVoltage_(maxVoltage), maxPressurePsi_(maxPressurePsi) {}

    float get_pressure(bool printVoltage = true) const {
        // 0 to 1023
        int rawValue = analogRead(analogPin_);

        double voltage = 4.988 * rawValue / 1023.0;
        if (printVoltage) {
            Serial.print("Voltage: ");
            Serial.print(voltage);
        }

        double pressurePsi = (voltage - minVoltage_) * (maxPressurePsi_ / (maxVoltage_ - minVoltage_));
        return pressurePsi;
    }

   private:
    int analogPin_;
    double minVoltage_;
    double maxVoltage_;
    float maxPressurePsi_;
};
