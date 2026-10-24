#pragma once

#include <float.h>
#include <math.h>
#include <stddef.h>

#include "config.h"

namespace Feedforward {

// unit conversion constants
constexpr double kGravityConversion = 32.174;
constexpr double kSquareInchesPerSquareFoot = 144.0;
constexpr double kCvPerCdaIn2 = 37.97;
constexpr double kPoundsMassPerKilogram = 2.20462262185;

// to correct for rounding errors
constexpr double kEndpointTolerance = 8.0 * DBL_EPSILON;

struct ThrustTargets {
    double massFlowLbmPerSec;
    double downstreamPressurePsi;
    bool valid;
};

/**
 * @brief Converts a thrust setpoint and real upstream pressure to goal mdot,
 * and downstream pressure by interpolating from existing values taken from
 * fluid sims. This is currently intended only for one valve
 * @return ThrustTargets which contains mass flow rate and downstream pressure
 */
inline ThrustTargets set_thrust_to_mdot(
    double setThrustLbf, double upstreamReferencePsi) {

    // checks if set thrust and upstream pressure are valid
    const ThrustTargets invalid = {NAN, NAN, false};
    if (!isfinite(setThrustLbf) || setThrustLbf < 0.0) return invalid;
    if (!isfinite(upstreamReferencePsi) || upstreamReferencePsi <= 0.0) return invalid;
    if (setThrustLbf == 0.0) return {0.0, 0.0, true}; // valid setpoint, 0 mdot and pressure

    //doesn't allow thrust setpoint outside of range in thrust map
    if (setThrustLbf < Engine::kThrustLbf[0] ||
        setThrustLbf > Engine::kThrustLbf[Engine::kPointCount - 1]) return invalid;

    size_t low;
    size_t high;
    // finds index's of thrust values that the set thrust is in between
    for (size_t index = 1; index < Engine::kPointCount; ++index) {
        if (setThrustLbf <= Engine::kThrustLbf[index]) {
            low = index - 1;
            high = index;
            break;
        }
    }

    //interpolate thrust, mdot, and dp
    const double fraction = high == low ? 0.0 : (setThrustLbf - Engine::kThrustLbf[low]) / (Engine::kThrustLbf[high] - Engine::kThrustLbf[low]);
    const double massFlowKgPerSec = Engine::kLoxMdotKg[low] + fraction *
                                                                          (Engine::kLoxMdotKg[high] - Engine::kLoxMdotKg[low]);
    const double valveDpPsi = Engine::kValveDpPsi[low] + fraction *
                                                              (Engine::kValveDpPsi[high] - Engine::kValveDpPsi[low]);

    const double massFlowLbmPerSec = massFlowKgPerSec * kPoundsMassPerKilogram; // kg to lb
    const double downstreamPressurePsi = upstreamReferencePsi - valveDpPsi;

    //check values are valid
    if (!isfinite(massFlowLbmPerSec) || !isfinite(downstreamPressurePsi) ||
        downstreamPressurePsi < 0.0 || valveDpPsi <= 0.0) return invalid;

    return {massFlowLbmPerSec, downstreamPressurePsi, true};
}

/**
 * @brief takes in lox mass flow, upstream pressure and downstream goal pressure,
 * and fluid density, outputting a goal angle in degrees.
 * @return goal valve angle, capped at the maximum tabulated opening;
 * NAN for invalid inputs or required CdA below the table range
 */
inline double calculate_feedforward(
    double goalMdotLbmPerSec, double upstreamPressurePsi,
    double goalDownstreamPressurePsi, double densityLbmPerFt3) {
    if (goalMdotLbmPerSec == 0.0) return 0.0;

    const double pressureDropPsi = upstreamPressurePsi - goalDownstreamPressurePsi;
    if (pressureDropPsi <= 0.0) return NAN;

    const double denominator = sqrt(2.0 * pressureDropPsi *
                                    kSquareInchesPerSquareFoot * densityLbmPerFt3 * kGravityConversion);
    if (!isfinite(denominator) || denominator <= 0.0) return NAN;

    const double requiredCdAIn2 = kSquareInchesPerSquareFoot *
                                  goalMdotLbmPerSec / denominator;

    // Reject invalid or below-range requests, allowing roundoff at the lower bound.
    if (!isfinite(requiredCdAIn2) ||
        requiredCdAIn2 < Config::kValveCdA[0] * (1.0 - kEndpointTolerance)) return NAN;

    // Cap excess CdA demand at the maximum tabulated angle (90 degrees).
    if (requiredCdAIn2 <= Config::kValveCdA[0]) return Config::kValveAngleDeg[0];
    if (requiredCdAIn2 >= Config::kValveCdA[Config::kValvePointCount - 1]) return Config::kValveAngleDeg[Config::kValvePointCount - 1];

    // Interpolate using valve angle and CdA lookup tables
    for (size_t index = 1; index < Config::kValvePointCount; ++index) {
        // skip if required required CdA greater than the next CdA element in array
        if (requiredCdAIn2 > Config::kValveCdA[index]) continue;

        const double fraction = (requiredCdAIn2 - Config::kValveCdA[index - 1]) /
                                (Config::kValveCdA[index] - Config::kValveCdA[index - 1]);
        return Config::kValveAngleDeg[index - 1] + fraction *
                                                       (Config::kValveAngleDeg[index] - Config::kValveAngleDeg[index - 1]);
    }
    return NAN;
}
}  // namespace Feedforward
