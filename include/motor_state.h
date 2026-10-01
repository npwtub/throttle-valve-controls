/**
 * Motor telemetry cache.
 * - Tracks one SPARK MAX by CAN ID.
 * - Reads Status 0 and Status 2 frames.
 * - Stores output, voltage, current, temperature, position, and speed.
 * - Checks telemetry freshness and target position.
 * - Leaves commands and movement tracking to the caller.
 */
#pragma once

#include <Arduino.h>
#include <math.h>
#include <mcp2515.h>
#include <low_level/low_sparkmax.h>


class MotorState
{
public:

    explicit MotorState(uint8_t motorID)
        : motorID_(motorID)
    {
    }

    // Call this for every incoming CAN frame.
    // Returns true if a valid Status 0 or Status 2
    // frame was processed.
    bool processFrame(const struct can_frame& frame)
    {
        namespace Spark = CanControl::LowLevel::SparkMax;

        // Ignore remote request frames.
        if (frame.can_id & CAN_RTR_FLAG)
            return false;

        // Spark MAX uses extended 29-bit CAN IDs.
        if (!(frame.can_id & CAN_EFF_FLAG))
            return false;

        const uint32_t id = frame.can_id & CAN_EFF_MASK;

        // Ignore frames belonging to other motors.
        const uint8_t receivedMotorID =
            id & Spark::SPARK_DEVICE_ID_MASK;

        if (receivedMotorID != motorID_)
            return false;

        // Remove the device ID to identify frame type.
        const uint32_t baseID =
            id & ~Spark::SPARK_DEVICE_ID_MASK;

        // ----------------------------------------------
        // STATUS 0
        // Output %, voltage, current, temperature, limits
        // ----------------------------------------------
        if (baseID == Spark::SPARK_ARB_STATUS_0)
        {
            Spark::Spark_STATUS_0_t status{};

            if (!Spark::spark_decode_STATUS_0(
                    frame.data,
                    frame.can_dlc,
                    &status))
            {
                return false;
            }

            appliedOutput_ =
                static_cast<float>(status.APPLIED_OUTPUT)
                / 32767.0f;

            voltageRaw_ = status.VOLTAGE;
            currentRaw_ = status.CURRENT;

            temperature_ = status.MOTOR_TEMPERATURE;

            lastStatus0_ = millis();

            receivedStatus0_ = true;

            return true;
        }

        // ----------------------------------------------
        // STATUS 2
        // Encoder velocity and position
        // ----------------------------------------------
        if (baseID == Spark::SPARK_ARB_STATUS_2)
        {
            Spark::Spark_STATUS_2_t status{};

            if (!Spark::spark_decode_STATUS_2(
                    frame.data,
                    frame.can_dlc,
                    &status))
            {
                return false;
            }

            rpm_ = status.PRIMARY_ENCODER_VELOCITY;

            position_ = status.PRIMARY_ENCODER_POSITION;

            lastStatus2_ = millis();

            receivedStatus2_ = true;

            return true;
        }

        return false;
    }

    // --------------------------------------------------
    // GETTERS
    // --------------------------------------------------

    // Applied output: -1.0 to +1.0
    float getAppliedOutput() const
    {
        return appliedOutput_;
    }

    // Applied output as percentage: -100 to +100
    float getAppliedOutputPercent() const
    {
        return appliedOutput_ * 100.0f;
    }

    // Primary encoder velocity (default RPM)
    float getRPM() const
    {
        return rpm_;
    }

    // Primary encoder position (default rotations)
    float getPosition() const
    {
        return position_;
    }

    // Motor temperature in degrees Celsius
    float getTemperature() const
    {
        return static_cast<float>(temperature_);
    }

    // Raw decoded voltage field
    uint16_t getVoltageRaw() const
    {
        return voltageRaw_;
    }

    // Raw decoded current field
    uint16_t getCurrentRaw() const
    {
        return currentRaw_;
    }

    // --------------------------------------------------
    // TELEMETRY VALIDITY
    // --------------------------------------------------

    bool hasStatus0() const
    {
        return receivedStatus0_;
    }

    bool hasStatus2() const
    {
        return receivedStatus2_;
    }

    bool hasTelemetry() const
    {
        return receivedStatus0_ && receivedStatus2_;
    }

    // Target and tolerance use motor rotations. Only fresh, finite position
    // feedback can satisfy this check; callers track feedback since a command.
    bool atPosition(double target, unsigned long now, double tolerance,
                    unsigned long timeoutMs) const
    {
        return hasStatus2() &&
               now - lastStatus2_ <= timeoutMs &&
               isfinite(position_) &&
               fabs(position_ - target) <= tolerance;
    }

    // Check whether both status frames have arrived
    // within the specified timeout.
    bool isFresh(unsigned long timeoutMs = 250) const
    {
        if (!hasTelemetry())
            return false;

        const unsigned long now = millis();

        return
            (now - lastStatus0_ <= timeoutMs) &&
            (now - lastStatus2_ <= timeoutMs);
    }

    unsigned long getLastStatus0Time() const
    {
        return lastStatus0_;
    }

    unsigned long getLastStatus2Time() const
    {
        return lastStatus2_;
    }

private:

    uint8_t motorID_;

    // Status 0 cached values
    float appliedOutput_ = 0.0f;

    uint16_t voltageRaw_ = 0;
    uint16_t currentRaw_ = 0;

    uint8_t temperature_ = 0;

    bool forwardLimit_ = false;
    bool reverseLimit_ = false;

    // Status 2 cached values
    float rpm_ = 0.0f;
    float position_ = 0.0f;

    // Receipt tracking
    bool receivedStatus0_ = false;
    bool receivedStatus2_ = false;

    unsigned long lastStatus0_ = 0;
    unsigned long lastStatus2_ = 0;
};
