/**
 * Basic motor position test.
 * - Starts automatically.
 * - Commands four rotations for two seconds, then two rotations.
 * - Sends CAN heartbeats to keep the controller enabled.
 * - Prints motor output, speed, and position over Serial.
 */
#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <mcp2515.h>

#include "config.h"
#include "motor_state.h"

constexpr unsigned long PRINT_INTERVAL_MS = 100;

// MCP2515 chip select pin
MCP2515 mcp2515(Config::CAN_CS_PIN, 4000000UL);

// Spark MAX CAN ID = 1
CanControl::SparkMax motor(mcp2515, Config::LOX_MOTOR_ID);

MotorState motorState(Config::LOX_MOTOR_ID);

void setup() {
    Serial.begin(Config::BAUD_RATE_LONG);
    delay(500);

    Serial.println("Starting Spark MAX test...");
    // Initialize MCP2515
    mcp2515.reset();

    // Spark MAX CAN bus runs at 1 Mbps.
    // Change MCP_8MHZ if your MCP2515 module has a different crystal.
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);

    mcp2515.setNormalOneShotMode();
}

void loop() {
    static unsigned long startTime = millis();
    static unsigned long lastHeartbeat = 0;
    static unsigned long lastPrint = 0;

    unsigned long now = millis();

    // Spark MAX needs heartbeat messages to stay enabled
    if (now - lastHeartbeat >= 20) {
        CanControl::send_heartbeat(
            mcp2515,
            CanControl::default_heartbeat());

        lastHeartbeat = now;
    }

    if (now - startTime < 2000) {
        motor.set_position(4, 0, 0, 0);
        // motor.set_duty_cycle(0.1);
    } else {
        motor.set_position(2, 0, 0, 0);
        // motor.set_duty_cycle(0.0);
    }


    struct can_frame frame;
    while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK)
    {
    motor.handle_received_frame(frame);
    motorState.processFrame(frame);
    }

    if (now - lastPrint > 500) {
        lastPrint = now;

        Serial.print("Output: ");
        Serial.print(motorState.getAppliedOutputPercent());

        Serial.print("% | RPM: ");
        Serial.print(motorState.getRPM());

        Serial.print(" | Position: ");
        Serial.println(motorState.getPosition());

    }
    delay(5);
}


