// Host regression checks for thrust-to-chamber-pressure targets.
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#define A2 56
#define A3 57
#include "utils.h"
#include "thrust_profiles.h"

static bool near(double actual, double expected) {
    return fabs(actual - expected) < 1e-8;
}

template <size_t count>
static void check_profile(const double (&profile)[count]) {
    for (double thrust : profile) {
        const double pressure = utils::chamber_pressure_for_thrust(thrust);
        assert(isfinite(pressure));
        assert(pressure >= Engine::kChamberPressurePsi[0]);
        assert(pressure <= Engine::kChamberPressurePsi[Engine::kPointCount - 1]);
    }
}

int main() {
    assert(utils::chamber_pressure_for_thrust(0.0) == 0.0);
    for (size_t index = 0; index < Engine::kPointCount; ++index) {
        assert(near(utils::chamber_pressure_for_thrust(Engine::kThrustLbf[index]),
                    Engine::kChamberPressurePsi[index]));
    }

    // Interior points must interpolate on thrust, not the simulation time axis.
    const double fractions[] = {0.25, 0.5, 0.75};
    for (size_t high = 1; high < Engine::kPointCount; ++high) {
        for (double fraction : fractions) {
            const double thrust = Engine::kThrustLbf[high - 1] + fraction *
                (Engine::kThrustLbf[high] - Engine::kThrustLbf[high - 1]);
            const double expected = Engine::kChamberPressurePsi[high - 1] + fraction *
                (Engine::kChamberPressurePsi[high] - Engine::kChamberPressurePsi[high - 1]);
            assert(near(utils::chamber_pressure_for_thrust(thrust), expected));
        }
    }

    const double maxThrust = Engine::kThrustLbf[Engine::kPointCount - 1];
    const double maxPressure = Engine::kChamberPressurePsi[Engine::kPointCount - 1];
    const double cappedThrusts[] = {maxThrust + 1e-6, 500.0, DBL_MAX};
    for (double thrust : cappedThrusts) {
        assert(utils::chamber_pressure_for_thrust(thrust) == maxPressure);
    }
    const double invalidThrusts[] = {
        -1.0, 1.0, Engine::kThrustLbf[0] - 1e-6, NAN, INFINITY, -INFINITY
    };
    for (double thrust : invalidThrusts) {
        assert(isnan(utils::chamber_pressure_for_thrust(thrust)));
    }

    check_profile(ThrustProfile::kFlightProfile);
    check_profile(ThrustProfile::kRampUpThrust);
    puts("Engine target regression checks passed.");
}
