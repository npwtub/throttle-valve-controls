#pragma once
#include "config.h"
#include "motor_state.h"
#include <CanControl.h>
namespace utils{
    inline void setMotorPositionWithBacklash(CanControl::SparkMax& motor, double targetPosition){
        MotorState motorState = MotorState(motor.get_device_id());
        int prevDir = motorState.get_target_direction(); // Store the previous direction of movement

        // Determine the new direction of movement
        if(targetPosition > motorState.get_previous_target_position()){
            motorState.set_target_direction(1); // Increasing
        } else if(targetPosition < motorState.get_previous_target_position()){
            motorState.set_target_direction(-1); // Decreasing
        }

        // Apply backlash compensation based on the direction of movement
        if(motorState.get_target_direction() == 1 && prevDir == -1){ // Increasing
            motorState.set_total_backlash_compensation(motorState.get_total_backlash_compensation() + Config::kBacklashCompensation);
            targetPosition += Config::kBacklashCompensation; // Adjust the target position for backlash
        } else if(motorState.get_target_direction() == -1 && prevDir == 1){ // Decreasing
            motorState.set_total_backlash_compensation(motorState.get_total_backlash_compensation() - Config::kBacklashCompensation);
            targetPosition -= Config::kBacklashCompensation; // Adjust the target position for backlash
        }

        // Set the motor position
        motor.set_position(targetPosition);

        // Update the previous target position
        motorState.set_previous_target_position(targetPosition);
    }
}