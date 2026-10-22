/** SD-only bench test: no motor or CAN commands. Writes 10,000 samples at 200 Hz. */
#include "log.h"

// 1,250 full blocks, nominally 50 seconds plus overruns and finalization.
constexpr uint32_t sampleLimit = 10000;
uint32_t sampleCount = 0;
bool complete = false;

void setup() {
    Serial.begin(Config::BAUD_RATE_LONG);
    if (!beginSdLog()) {
        complete = true;
        return;
    }
    waitForUnixTime();
    beginControlSchedule();
}

void loop() {
    if (complete) return;
    beginControlCycle();
    const bool finalRow = ++sampleCount == sampleLimit;
    // No sensors: presence flags are clear. The fixed command checks decoding.
    logRow(nullptr, nullptr, nullptr, millis(), finalRow ? F("done") : F("waiting"),
           Config::OPEN_ROTATIONS, finalRow);
    finishControlCycle();
    if (finalRow || sdLog.hasError()) {
        if (sdLog.isOpen()) closeSdLog();
        reportLogStatus(Serial);
        complete = true;
    }
}
