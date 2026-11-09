#pragma once
#include "config.h"
#include "motor_state.h"
#include <CanControl.h>
#include <math.h>
namespace utils{
    static_assert(sizeof(Engine::kChamberPressurePsi) / sizeof(Engine::kChamberPressurePsi[0]) == Engine::kPointCount,
                  "Chamber pressure and thrust tables must have matching lengths");

    // Interpolate simulated chamber pressure [psi] from requested thrust [lbf].
    // Match feedforward bounds: zero maps to zero, excess thrust clamps to the last
    // point, and negative/nonfinite or positive below-table requests return NAN.
    inline double chamber_pressure_for_thrust(double goalThrustLbf) {
        if (!isfinite(goalThrustLbf) || goalThrustLbf < 0.0) return NAN;
        if (goalThrustLbf == 0.0) return 0.0;
        if (goalThrustLbf < Engine::kThrustLbf[0]) return NAN;
        if (goalThrustLbf >= Engine::kThrustLbf[Engine::kPointCount - 1]) {
            return Engine::kChamberPressurePsi[Engine::kPointCount - 1];
        }

        for (size_t high = 1; high < Engine::kPointCount; ++high) {
            if (goalThrustLbf > Engine::kThrustLbf[high]) continue;
            const size_t low = high - 1;
            const double fraction = (goalThrustLbf - Engine::kThrustLbf[low]) /
                                    (Engine::kThrustLbf[high] - Engine::kThrustLbf[low]);
            return Engine::kChamberPressurePsi[low] + fraction *
                (Engine::kChamberPressurePsi[high] - Engine::kChamberPressurePsi[low]);
        }
        return NAN;
    }

    // Call for each position command so direction follows changes in target.
    // Add the offset only on an upward move after feedback confirms the setpoint.
    inline double backlashCompensation(MotorState& motorState,
                                             double targetPosition,
                                             bool atSetpoint){
        const bool movingUp = motorState.update_target_direction(targetPosition);
        if(movingUp){
            return Config::kBacklashCompensation;
        }
        return 0.0;
    }
}
