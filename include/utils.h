#pragma once
#include "config.h"
#include <CanControl.h>

namespace utils{
    inline void setMotorPositionWithBacklash(CanControl::SparkMax& motor, double targetPosition){
        int prevDir = Config::motorDir; // Store the previous direction of movement

        // Determine the new direction of movement
        if(targetPosition > Config::motorPrevTargetPosition){
            Config::motorDir = 1; // Increasing
        } else if(targetPosition < Config::motorPrevTargetPosition){
            Config::motorDir = -1; // Decreasing
        } else {
            Config::motorDir = 0; // No movement
        }

        // Apply backlash compensation based on the direction of movement
        if(Config::motorDir == 1 && prevDir != 1){ // Increasing
            targetPosition += Config::backlash;
        } else if(Config::motorDir == -1 && prevDir != -1){ // Decreasing
            targetPosition -= Config::backlash;
        }

        // Set the motor position
        motor.set_position(targetPosition);

        // Update the previous target position
        Config::motorPrevTargetPosition = targetPosition;
    }
}