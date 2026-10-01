/**
 * Pressure transducer test.
 * - Reads sensors on A0 and A1.
 * - Uses a 0.5-4.5 V range for 0-1000 PSI.
 * - Prints each sensor's voltage and pressure over Serial.
 */
#include <Arduino.h>
#include "pressure_transducer.h"
#include "config.h"

PressureTransducer pt1 = PressureTransducer(A0, 1000, 0.5, 4.5);
PressureTransducer pt2 = PressureTransducer(A1, 1000, 0.5, 4.5);

void setup () {
    Serial.begin(Config::BAUD_RATE_LONG);
}


void loop (){
    unsigned long lastPrint = 0;
    unsigned long now = millis();

    if (now - lastPrint > 500) {
        double pressure1 = pt1.getPressure();
        lastPrint = now;
        Serial.print("  PSI 1: ");
        Serial.print(pressure1);
        Serial.print(" | ");

        double pressure2 = pt2.getPressure();
        Serial.print("  PSI 2: ");
        Serial.println(pressure2);
    }
}


