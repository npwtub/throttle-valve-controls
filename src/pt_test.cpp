/**
 * Pressure transducer test.
 * - Reads sensors on A0 and A1.
 * - Uses a 0.5-4.5 V range for 0-1000 PSI.
 * - Prints each sensor's voltage and pressure over Serial.
 */
#include <Arduino.h>
#include "pressure_transducer.h"
#include "config.h"

PressureTransducer pt1 = PressureTransducer(A2, 1000, 0.47, 4.5);
PressureTransducer pt2 = PressureTransducer(A3, 1000, 0.47, 4.5);

void setup () {
    Serial.begin(Config::kBaudRateLong);
}


void loop (){
    unsigned long lastPrint = 0;
    unsigned long now = millis();

    if (now - lastPrint > 500) {
        double pressure1 = pt1.get_pressure();
        lastPrint = now;
        Serial.print("  PSI 1: ");
        Serial.print(pressure1);
        Serial.print(" | ");

        double pressure2 = pt2.get_pressure();
        Serial.print("  PSI 2: ");
        Serial.println(pressure2);
    }
}


