#pragma once
#include "config.h"
#include "motor_state.h"
#include <CanControl.h>
namespace utils{
    // Call for each position command so direction follows changes in target.
    // Add the offset only on an upward move after feedback confirms the setpoint.
    inline double backlashCompensatedTarget(MotorState& motorState,
                                             double targetPosition,
                                             bool atSetpoint){
        const bool movingUp = motorState.update_target_direction(targetPosition);
        if(movingUp){
            return Config::kBacklashCompensation;
        }
        return 0.0;
    }
}
