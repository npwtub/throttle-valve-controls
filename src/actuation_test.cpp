#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <mcp2515.h>

#include "config.h"
#include "motor_state.h"
#include "pressure_transducer.h"

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

    Serial.println("Starting Spark MAX test...");
    // Initialize MCP2515
    mcp2515.reset();

    // Spark MAX CAN bus runs at 1 Mbps.
    // Change MCP_8MHZ if your MCP2515 module has a different crystal.
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);

    mcp2515.setNormalOneShotMode();

    delay(delayTime_ms);
}

void loop() {
    static int actuationCount = 0;

    // read incoming can frames to get motor status/measurements
    struct can_frame frame;
    while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        motor.handle_received_frame(frame);
        motorState.processFrame(frame);
    }

    static unsigned long lastActuation = 0;
    static unsigned long lastHeartbeat = 0;
    unsigned long now = millis();

    // Spark MAX needs heartbeat messages to stay enabled
    if (now - lastHeartbeat >= 20) {
        CanControl::send_heartbeat(
            mcp2515,
            CanControl::default_heartbeat());

        lastHeartbeat = now;
    }

    switch (testState) {
        case WAITING:
            // Poll without blocking so CAN reception and heartbeats continue.
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
                motor.set_position(Config::OPEN_ROTATIONS, 0, 0, 0);
            } else {
                // keep valve closed if interval has not passed
                motor.set_position(0, 0, 0, 0);
            }
            break;
        case OPENING:
            // if at goal position, change state, print time to actuate, and keep the valve open
            if (Config::OPEN_ROTATIONS - motorState.getPosition() < 0.15) {
                Serial.print("Time to Open: ");
                Serial.print(now - lastActuation);
                testState = OPEN;

                lastActuation = now;
            }
            if (now - lastActuation > travelTimeout_ms) {
                Serial.print("Timeout activated while opening");
                testState = TIMEOUT;
                break;
            }
            motor.set_position(Config::OPEN_ROTATIONS, 0, 0, 0);
            break;
        case OPEN:
            // if actuation time has passed, close the valve
            if ((now - lastActuation) > timeOpen_ms) {
                testState = CLOSING;
                lastActuation = now;
                motor.set_position(0, 0, 0, 0);
            } else {
                // keep the valve open if timeOpen hasnt passed yet
                motor.set_position(Config::OPEN_ROTATIONS, 0, 0, 0);
            }
            break;
        case CLOSING:
            // if valve is close to goal angle, print time to close
            if ((motorState.getPosition() < 0.15)) {
                Serial.print("  |  Time to Close: ");
                Serial.println(now - lastActuation);
                lastActuation = now;
                if (actuationCount >= numActuations) {
                    Serial.print("Actuation Test Complete");
                    testState = DONE;  // test is done
                } else {
                    testState = CLOSED;  // test is not yet done and valve will open again
                }
            }
            if (now - lastActuation > travelTimeout_ms) {
                Serial.print("Timeout activated while closing");
                testState = TIMEOUT;
                break;
            }
            motor.set_position(0, 0, 0, 0);
            break;
        case DONE:
            motor.set_position(0, 0, 0, 0);
            delay(20);
            break;
        case TIMEOUT:
            delay(20);
            motor.set_position(0, 0, 0, 0);
            Serial.print("Position: ");
            Serial.println(motorState.getPosition());
            break;
    }
}
