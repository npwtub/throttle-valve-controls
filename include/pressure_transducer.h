#pragma once

#include <Arduino.h>

class pressure_transducer {
   public:
    pressure_transducer(int analogPin, float maxPressurePSI, double minVoltage, double maxVoltage) : analogPin_(analogPin), maxPressurePSI_(maxPressurePSI), minVoltage_(minVoltage), maxVoltage_(maxVoltage) {}

    float getPressure(bool printVoltage = true) const {
        // 0 to 1023
        int rawValue = analogRead(analogPin_);

        double voltage = 5.0 * rawValue / 1023.0;
        if (printVoltage) {
            Serial.print("Voltage: ");
            Serial.print(voltage);
        }
        // range from 1-5 volts
        double pressurePSI = (voltage - minVoltage_) * (maxPressurePSI_ / (maxVoltage_ - minVoltage_));
        return pressurePSI;
    }

   private:
    int analogPin_;
    double minVoltage_;
    double maxVoltage_;
    float maxPressurePSI_;
};
