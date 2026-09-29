#include <Arduino.h>
#include "pressure_transducer.h"
#include "config.h"

pressure_transducer pt1 = pressure_transducer(A0, 1000, 0.5, 4.5);
pressure_transducer pt2 = pressure_transducer(A1, 1000, 0.5, 4.5);

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


