/**
 * Repeated valve actuation test.
 * - Starts when a space arrives over Serial.
 * - Runs five opening and closing cycles.
 * - Holds open for five seconds per cycle.
 * - Commands closed after completion or a travel timeout.
 * - Calculates and submits control + heartbeat frames every 5 ms.
 * - Reports cumulative send timing and failures once per second.
 */
#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <mcp2515.h>

#include "config.h"
#include "motor_state.h"

// MCP2515 chip select pin
MCP2515 mcp2515(Config::CAN_CS_PIN, 4000000UL);

// Spark MAX CAN ID = 1
CanControl::SparkMax motor(mcp2515, Config::LOX_MOTOR_ID);

MotorState motorState(Config::LOX_MOTOR_ID);

const double delayTime_ms = 1000;
const int timeOpen_ms = 5000;
const int actuationInterval_ms = 5000;
const int travelTimeout_ms = 1000;
const int numActuations = 5;
constexpr unsigned long controlPeriodUs = 5000;
constexpr unsigned long timingReportPeriodMs = 1000;

// Buffer reports in RAM, then copy only as many bytes as the UART can accept.
// Long timing reports must not block the next control cycle at 57600 baud.
class BufferedSerialLog : public Print {
public:
    size_t write(uint8_t value) override {
        const uint16_t next = (head + 1) % sizeof(buffer);
        if (next == tail) return 0;
        buffer[head] = value;
        head = next;
        return 1;
    }

    void drain() {
        int available = Serial.availableForWrite();
        while (available-- > 0 && tail != head) {
            Serial.write(buffer[tail]);
            tail = (tail + 1) % sizeof(buffer);
        }
    }

private:
    uint8_t buffer[512] = {};
    uint16_t head = 0;
    uint16_t tail = 0;
};

BufferedSerialLog serialLog;

struct SendTiming {
    uint64_t totalUs = 0;
    unsigned long cycles = 0;
    unsigned long worstUs = 0;
    unsigned long lateSends = 0;
    unsigned long controlErrors = 0;
    unsigned long heartbeatErrors = 0;
    unsigned long missedSends = 0;
    unsigned long cycleOverruns = 0;
};

SendTiming sendTiming;

void reportSendTiming() {
    serialLog.print(F("CAN timing: cycles="));
    serialLog.print(sendTiming.cycles);
    serialLog.print(F(" avg_us="));
    serialLog.print(static_cast<unsigned long>(sendTiming.totalUs / sendTiming.cycles));
    serialLog.print(F(" worst_us="));
    serialLog.print(sendTiming.worstUs);
    serialLog.print(F(" late_5ms="));
    serialLog.print(sendTiming.lateSends);
    serialLog.print(F(" control_errors="));
    serialLog.print(sendTiming.controlErrors);
    serialLog.print(F(" heartbeat_errors="));
    serialLog.print(sendTiming.heartbeatErrors);
    serialLog.print(F(" missed_sends="));
    serialLog.print(sendTiming.missedSends);
    serialLog.print(F(" cycle_overruns="));
    serialLog.println(sendTiming.cycleOverruns);
}

enum TestState {
    WAITING,
    CLOSED,
    OPENING,
    OPEN,
    CLOSING,
    DONE,
    TIMEOUT
};

TestState testState = WAITING;

void setup() {
    Serial.begin(Config::BAUD_RATE_LONG);

    // Initialize MCP2515
    mcp2515.reset();

    // Spark MAX CAN bus runs at 1 Mbps.
    // Change MCP_8MHZ if your MCP2515 module has a different crystal.
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);

    mcp2515.setNormalOneShotMode();

    delay(delayTime_ms);
}

void loop() {
    const unsigned long cycleStartUs = micros();
    static int actuationCount = 0;
    static unsigned long lastActuation = 0;
    static unsigned long lastTimingReport = 0;

    // Bound receive work so continuous traffic cannot starve outgoing frames.
    struct can_frame frame;
    for (uint8_t count = 0; count < 8 &&
                            mcp2515.readMessage(&frame) == MCP2515::ERROR_OK;
         ++count) {
        motor.handle_received_frame(frame);
        motorState.processFrame(frame);
    }

    const unsigned long now = millis();

    switch (testState) {
        case WAITING:
            // Poll without blocking so the 5 ms control cycle continues.
            // A single space starts the test; no newline is required.
            if (Serial.available() > 0 && Serial.read() == ' ') {
                lastActuation = now;  // Begin the existing initial closed interval.
                testState = CLOSED;
            }
            break;
        case CLOSED:
            // if actuation interval has passed, open the valve
            if ((now - lastActuation) > actuationInterval_ms) {
                actuationCount++;
                testState = OPENING;
                lastActuation = now;
            }
            break;
        case OPENING:
            // At the goal position, start the open hold interval.
            if (Config::OPEN_ROTATIONS - motorState.getPosition() < 0.15) {
                testState = OPEN;

                lastActuation = now;
            } else if (now - lastActuation > travelTimeout_ms) {
                testState = TIMEOUT;
            }
            break;
        case OPEN:
            // if actuation time has passed, close the valve
            if ((now - lastActuation) > timeOpen_ms) {
                testState = CLOSING;
                lastActuation = now;
            }
            break;
        case CLOSING:
            // At the closed position, complete the test or start the next interval.
            if ((motorState.getPosition() < 0.15)) {
                lastActuation = now;
                if (actuationCount >= numActuations) {
                    testState = DONE;  // test is done
                } else {
                    testState = CLOSED;  // test is not yet done and valve will open again
                }
            } else if (now - lastActuation > travelTimeout_ms) {
                testState = TIMEOUT;
            }
            break;
        case DONE:
        case TIMEOUT:
            break;
    }

    // Submit both frames immediately after calculations, exactly once per cycle.
    // Waiting, completed, and timed-out tests all command the closed position.
    const bool openingOrOpen = testState == OPENING || testState == OPEN;
    const float target = openingOrOpen ? Config::OPEN_ROTATIONS : Config::CLOSED_ROTATIONS;
    const MCP2515::ERROR controlResult = motor.set_position(target, 0, 0, 0);
    const MCP2515::ERROR heartbeatResult = CanControl::send_heartbeat(
        mcp2515, CanControl::default_heartbeat());

    // Time includes CAN reception, calculations, and BOTH send calls, even on
    // failures/overruns. The driver queues frames; this is not on-wire latency.
    const unsigned long sendTimeUs = micros() - cycleStartUs;
    ++sendTiming.cycles;
    sendTiming.totalUs += sendTimeUs;
    if (sendTimeUs > sendTiming.worstUs) sendTiming.worstUs = sendTimeUs;
    const bool late = sendTimeUs > controlPeriodUs;
    const bool controlFailed = controlResult != MCP2515::ERROR_OK;
    const bool heartbeatFailed = heartbeatResult != MCP2515::ERROR_OK;
    if (late) ++sendTiming.lateSends;
    if (controlFailed) ++sendTiming.controlErrors;
    if (heartbeatFailed) ++sendTiming.heartbeatErrors;
    // A failed frame misses this cycle even when the driver returns promptly.
    if (late || controlFailed || heartbeatFailed) ++sendTiming.missedSends;

    // Queue timing output only AFTER both frames have been submitted.
    if (now - lastTimingReport >= timingReportPeriodMs) {
        lastTimingReport = now;
        reportSendTiming();
    }
    serialLog.drain();

    // Serial formatting/draining and bookkeeping also consume this cycle's
    // budget. A slow cycle starts the next one immediately, with no extra delay.
    const unsigned long elapsedUs = micros() - cycleStartUs;
    if (elapsedUs > controlPeriodUs) {
        ++sendTiming.cycleOverruns;
    } else if (elapsedUs < controlPeriodUs) {
        const unsigned long remainingUs = controlPeriodUs - elapsedUs;
        delay(remainingUs / 1000UL);
        // Finish the fractional millisecond against the original start time,
        // keeping interrupt handling and delay overhead inside the 5 ms budget.
        while (micros() - cycleStartUs < controlPeriodUs) {
        }
    }
}