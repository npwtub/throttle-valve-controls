#pragma once
#include "config.h"
#include <CanControl.h>

namespace utils{
    inline double backlashCompensation(CanControl::SparkMax& motor, double target){
        const double position = motor.get_encoder_position();

        if(position < target){
            return target + Config::backlash;
        } else if(position > target){
            return target - Config::backlash;
        } else {
            return target;
        }
    }
}