
#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <mcp2515.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "motor_state.h"
#include "pressure_transducer.h"

// CAN and motor initialization
// MCP2515 chip select pin
MCP2515 mcp2515(Config::CAN_CS_PIN, 4000000UL);

// Spark MAX motor
CanControl::SparkMax motor(mcp2515, Config::LOX_MOTOR_ID);

// Motor telemetry
MotorState motorState(Config::LOX_MOTOR_ID);

// Position tolerance in motor rotations
const double positionTolerance = 0.15;

// Timing in milliseconds
const unsigned long timeOpen_ms = 1000;
const unsigned long actuationInterval_ms = 120000;
const unsigned long travelTimeout_ms = 1500;

// CAN timing
const unsigned long heartbeatInterval_ms = 20;
const unsigned long commandInterval_ms = 20;


// ============================================================
// State Machine
// ============================================================
enum TestState {
    WAITING,
    CLOSED,
    OPENING,
    OPEN,
    CLOSING,
    TIMEOUT,
    STOPPED
};

TestState testState = WAITING;


// ============================================================
// Test Variables
// ============================================================
// Number of successfully completed cycles
unsigned long completedCycles = 0;

// Timing variables
unsigned long cycleStartTime = 0;
unsigned long stateStartTime = 0;

unsigned long lastHeartbeat = 0;
unsigned long lastCommand = 0;

// Opening and closing times
unsigned long openingTime = 0;
unsigned long closingTime = 0;


// ============================================================
// Motor Commands
// ============================================================
void commandPosition(double rotations, unsigned long now) {

    motor.set_position(rotations, 0, 0, 0);

    lastCommand = now;
}

// Periodically resend position commands without flooding CAN.

void holdPosition(double rotations, unsigned long now) {

    if (now - lastCommand >= commandInterval_ms) {

        commandPosition(rotations, now);

    }
}


// ============================================================
// Start New Actuation Cycle
// ============================================================

void startCycle(unsigned long now) {

    cycleStartTime = now;
    stateStartTime = now;

    testState = OPENING;

    openingTime = 0;
    closingTime = 0;

    Serial.println();
    Serial.println("================================");

    Serial.print("CYCLE ");
    Serial.println(completedCycles + 1);

    Serial.println("================================");
    Serial.println("Commanding valve OPEN");

    commandPosition(Config::OPEN_ROTATIONS, now);
}


// ============================================================
// Terminate Test
// ============================================================

void terminateTest(const char* reason, unsigned long now) {

    testState = TIMEOUT;

    Serial.println();
    Serial.println("================================");
    Serial.println("DWELL TEST TERMINATED");
    Serial.println("================================");

    Serial.print("Failure: ");
    Serial.println(reason);

    Serial.print("Failed cycle: ");
    Serial.println(completedCycles + 1);

    Serial.print("Successfully completed cycles: ");
    Serial.println(completedCycles);

    Serial.print("Motor position: ");
    Serial.println(motorState.getPosition(), 4);

    Serial.print("Travel time at failure: ");
    Serial.print(now - stateStartTime);
    Serial.println(" ms");

    Serial.println();
    Serial.println("Commanding valve CLOSED.");
    Serial.println("Test will not automatically restart.");

    Serial.println("================================");

    commandPosition(Config::OPEN_ROTATIONS, now);
}


// ============================================================
// Manual Stop
// ============================================================

void stopTest(unsigned long now) {

    testState = STOPPED;

    Serial.println();
    Serial.println("================================");
    Serial.println("MANUAL STOP");
    Serial.println("================================");

    Serial.print("Successfully completed cycles: ");
    Serial.println(completedCycles);

    Serial.println("Commanding valve CLOSED.");

    Serial.println("================================");

    commandPosition(Config::OPEN_ROTATIONS, now);
}


// ============================================================
// Serial Input
// ============================================================

// Valid commands:
//
// START - Begin dwell test
// STOP  - Terminate dwell test
//
// Commands are case-sensitive.

void processSerial(unsigned long now) {

    static char input[16];
    static uint8_t inputIndex = 0;

    while (Serial.available() > 0) {

        char c = Serial.read();

        // Accept either newline or carriage return.
        if (c == '\n' || c == '\r') {

            // Ignore empty lines.
            if (inputIndex == 0) {
                continue;
            }

            input[inputIndex] = '\0';
            inputIndex = 0;

            // --------------------------------------------
            // START command
            // --------------------------------------------

            if (strcmp(input, "START") == 0) {

                if (testState == WAITING) {

                    Serial.println();
                    Serial.println("START command received.");
                    Serial.println("Beginning dwell test.");

                    startCycle(now);

                } else {

                    Serial.println(
                        "Test already started or terminated."
                    );

                }

            }

            // --------------------------------------------
            // STOP command
            // --------------------------------------------

            else if (strcmp(input, "STOP") == 0) {

                if (testState != WAITING &&
                    testState != STOPPED &&
                    testState != TIMEOUT) {

                    stopTest(now);

                }

            }

            // --------------------------------------------
            // Invalid command
            // --------------------------------------------

            else {

                Serial.println(
                    "Invalid command. Enter START or STOP."
                );

            }

        }

        // Add character to input buffer.
        else if (inputIndex < sizeof(input) - 1) {

            input[inputIndex++] = c;

        }

        // Prevent buffer overflow.
        else {

            inputIndex = 0;

            Serial.println("Serial input too long.");

        }
    }
}


// ============================================================
// Setup
// ============================================================

void setup() {

    Serial.begin(Config::BAUD_RATE_LONG);

    // Initialize MCP2515
    mcp2515.reset();

    // Spark MAX CAN bus runs at 1 Mbps.
    // Change MCP_8MHZ if your module uses a different crystal.
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);

    mcp2515.setNormalOneShotMode();

    Serial.println();
    Serial.println("================================");
    Serial.println("CRYOGENIC VALVE DWELL TEST");
    Serial.println("================================");

    Serial.print("Open angle: ");
    Serial.println(Config::OPEN_THETA);

    Serial.print("Open duration: ");
    Serial.print(timeOpen_ms);
    Serial.println(" ms");

    Serial.print("Actuation interval: ");
    Serial.print(actuationInterval_ms);
    Serial.println(" ms");

    Serial.print("Travel timeout: ");
    Serial.print(travelTimeout_ms);
    Serial.println(" ms");

    Serial.println("Cycle limit: NONE");

    Serial.println();
    Serial.println("Waiting for user input...");
    Serial.println("Enter START to begin.");
    Serial.println("Enter STOP to terminate.");

    Serial.println("================================");
}


// ============================================================
// Main Loop
// ============================================================

void loop() {

    unsigned long now = millis();

    // process CAN frames from bus
    struct can_frame frame;

    while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {

        motor.handle_received_frame(frame);

        motorState.processFrame(frame);

    }

    // sends heartbeat
    // heatbest has to send even while waiting for START.

    if (now - lastHeartbeat >= heartbeatInterval_ms) {

        CanControl::send_heartbeat(
            mcp2515,
            CanControl::default_heartbeat()
        );

        lastHeartbeat = now;

    }


    // ========================================================
    // Read Serial Commands
    // ========================================================
    processSerial(now);


    // ========================================================
    // State Machine
    // ========================================================
    switch (testState) {


        // ====================================================
        // WAITING
        // ====================================================

        case WAITING:

            // Do not begin the test until START is received.
            // No position commands are sent here.

            break;

        // ====================================================
        // CLOSED
        // ====================================================
        case CLOSED:

            // Maintain closed position while waiting.

            holdPosition(Config::CLOSED_ROTATIONS, now);

            // Start next cycle 2 minutes after the previous
            // opening command.

            if (now - cycleStartTime >= actuationInterval_ms) {

                startCycle(now);

            }

            break;


        // ====================================================
        // OPENING
        // ====================================================
        case OPENING:

            // Check if target position has been reached.

            if (isfinite(motorState.getPosition()) &&
                fabs(motorState.getPosition() - Config::OPEN_ROTATIONS)
                    <= positionTolerance) {

                openingTime = now - stateStartTime;

                Serial.print("Time to open: ");
                Serial.print(openingTime);
                Serial.println(" ms");

                // Begin 1-second open dwell.

                testState = OPEN;
                stateStartTime = now;

                break;

            }

            // Check opening timeout.

            if (now - stateStartTime >= travelTimeout_ms) {

                terminateTest("OPENING TIMEOUT", now);

                break;

            }

            // Continue commanding full open.

            holdPosition(Config::OPEN_ROTATIONS, now);

            break;


        // ====================================================
        // OPEN
        // ====================================================
        case OPEN:

            // Hold valve open for 1 second after reaching
            // target position.

            if (now - stateStartTime >= timeOpen_ms) {

                Serial.println("Open dwell complete.");
                Serial.println("Commanding valve CLOSED.");

                testState = CLOSING;

                stateStartTime = now;

                commandPosition(Config::CLOSED_ROTATIONS, now);

            }

            else {

                holdPosition(Config::OPEN_ROTATIONS, now);

            }

            break;


        // ====================================================
        // CLOSING
        // ====================================================
        case CLOSING:

            // Check if valve has reached closed position.

            if (isfinite(motorState.getPosition()) &&
                fabs(motorState.getPosition() - Config::CLOSED_ROTATIONS
                    <= positionTolerance)) {

                closingTime = now - stateStartTime;

                Serial.print("Time to close: ");
                Serial.print(closingTime);
                Serial.println(" ms");

                // Successful cycle.

                completedCycles++;

                Serial.println();

                Serial.print("Cycle ");
                Serial.print(completedCycles);
                Serial.println(" COMPLETE");

                Serial.print("Total completed cycles: ");
                Serial.println(completedCycles);

                Serial.println();
                Serial.println("Waiting for next actuation...");

                // Return to closed state.

                testState = CLOSED;
                stateStartTime = now;

                break;

            }

            // Check closing timeout.

            if (now - stateStartTime >= travelTimeout_ms) {

                terminateTest("CLOSING TIMEOUT", now);

                break;

            }

            // Continue commanding closed.

            holdPosition(Config::CLOSED_ROTATIONS, now);

            break;


        // ====================================================
        // TIMEOUT
        // ====================================================
        case TIMEOUT:

            // Test is permanently terminated until reset.
            // Continue commanding the valve closed.

            holdPosition(Config::CLOSED_ROTATIONS, now);

            break;


        // ====================================================
        // STOPPED
        // ====================================================
        case STOPPED:

            // Manual termination.
            // Continue commanding the valve closed.

            holdPosition(Config::CLOSED_ROTATIONS, now);

            break;
    }
}