// Host regression checks for thrust targets and pressure-dependent valve angles.
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#define A2 56
#define A3 57
#include "feedforward.h"

static bool near(double actual, double expected) {
    return fabs(actual - expected) < 1e-8;
}

int main() {
    const double density = 62.26;
    const double maxAngle = Config::kValveAngleDeg[Config::kValvePointCount - 1];
    const auto target = Feedforward::set_thrust_to_mdot(300.0, 500.0);
    assert(target.valid);
    assert(near(target.massFlowLbmPerSec, 0.9785446735869422));
    assert(near(target.downstreamPressurePsi, 297.0769034231479));

    const double upstream[] = {500.0, 400.0, target.downstreamPressurePsi,
                               target.downstreamPressurePsi - 1.0, 0.0, -1.0};
    for (double pressure : upstream) {
        const auto other = Feedforward::set_thrust_to_mdot(300.0, pressure);
        assert(other.valid);
        assert(near(other.massFlowLbmPerSec, target.massFlowLbmPerSec));
        assert(near(other.downstreamPressurePsi, target.downstreamPressurePsi));
        const double angle = Feedforward::calculate_feedforward(
            other.massFlowLbmPerSec, pressure, other.downstreamPressurePsi, density);
        assert(isfinite(angle));
        if (pressure <= target.downstreamPressurePsi) assert(angle == maxAngle);
    }

    const double angle500 = Feedforward::calculate_feedforward(
        target.massFlowLbmPerSec, 500.0, target.downstreamPressurePsi, density);
    const double angle400 = Feedforward::calculate_feedforward(
        target.massFlowLbmPerSec, 400.0, target.downstreamPressurePsi, density);
    assert(angle500 > Config::kValveAngleDeg[0]);
    assert(angle500 < angle400 && angle400 < maxAngle);
    assert(Feedforward::calculate_feedforward(target.massFlowLbmPerSec,
        target.downstreamPressurePsi + 0.001, target.downstreamPressurePsi, density)
        == maxAngle);

    const auto zero = Feedforward::set_thrust_to_mdot(0.0, 0.0);
    assert(zero.valid && zero.massFlowLbmPerSec == 0.0);
    assert(Feedforward::calculate_feedforward(0.0, -1.0, 0.0, density) == 0.0);
    assert(!Feedforward::set_thrust_to_mdot(1.0, 500.0).valid);
    assert(!Feedforward::set_thrust_to_mdot(-1.0, 500.0).valid);
    const size_t last = Engine::kPointCount - 1;
    const double cappedThrust[] = {Engine::kThrustLbf[last],
        Engine::kThrustLbf[last] + 1e-6, 500.0, DBL_MAX};
    const double capPressures[] = {Engine::kDownstreamPsi[last] + 200.0,
                                  Engine::kDownstreamPsi[last] - 1.0};
    for (double pressure : capPressures) {
        const auto maximum = Feedforward::set_thrust_to_mdot(Engine::kThrustLbf[last], pressure);
        assert(maximum.valid);
        assert(near(maximum.massFlowLbmPerSec,
                    Engine::kLoxMdotKg[last] * Feedforward::kPoundsMassPerKilogram));
        assert(near(maximum.downstreamPressurePsi, Engine::kDownstreamPsi[last]));
        const double expectedAngle = Feedforward::calculate_feedforward(
            maximum.massFlowLbmPerSec, pressure, maximum.downstreamPressurePsi, density);
        assert(isfinite(expectedAngle));
        for (double thrust : cappedThrust) {
            const auto capped = Feedforward::set_thrust_to_mdot(thrust, pressure);
            assert(capped.valid);
            assert(near(capped.massFlowLbmPerSec, maximum.massFlowLbmPerSec));
            assert(near(capped.downstreamPressurePsi, maximum.downstreamPressurePsi));
            assert(near(Feedforward::calculate_feedforward(capped.massFlowLbmPerSec,
                pressure, capped.downstreamPressurePsi, density), expectedAngle));
        }
    }
    const double nonfinite[] = {NAN, INFINITY, -INFINITY};
    for (double invalid : nonfinite) {
        assert(isnan(Feedforward::equivalent_mass_flow(invalid, 4.0, 1.0)));
        assert(isnan(Feedforward::equivalent_mass_flow(1.0, invalid, 1.0)));
        assert(isnan(Feedforward::equivalent_mass_flow(1.0, 4.0, invalid)));
        assert(!Feedforward::set_thrust_to_mdot(invalid, 500.0).valid);
        assert(!Feedforward::set_thrust_to_mdot(300.0, invalid).valid);
        assert(isnan(Feedforward::calculate_feedforward(
            target.massFlowLbmPerSec, invalid, target.downstreamPressurePsi, density)));
        assert(isnan(Feedforward::calculate_feedforward(
            target.massFlowLbmPerSec, 0.0, invalid, density)));
        assert(isnan(Feedforward::calculate_feedforward(
            target.massFlowLbmPerSec, 0.0, target.downstreamPressurePsi, invalid)));
    }
    const double invalidDensity[] = {0.0, -1.0};
    for (double invalid : invalidDensity) {
        assert(isnan(Feedforward::equivalent_mass_flow(1.0, invalid, density)));
        assert(isnan(Feedforward::equivalent_mass_flow(1.0, density, invalid)));
        assert(isnan(Feedforward::calculate_feedforward(
            target.massFlowLbmPerSec, 0.0, target.downstreamPressurePsi, invalid)));
    }
    // Known density ratios, including identity and zero-flow cases.
    assert(Feedforward::equivalent_mass_flow(2.0, 4.0, 1.0) == 1.0);
    assert(Feedforward::equivalent_mass_flow(2.0, 1.0, 4.0) == 4.0);
    assert(Feedforward::equivalent_mass_flow(2.0, 4.0, 4.0) == 2.0);
    assert(Feedforward::equivalent_mass_flow(0.0, 4.0, 1.0) == 0.0);
    assert(isnan(Feedforward::equivalent_mass_flow(-1.0, 4.0, 1.0)));

    // Equivalent water flow must reproduce the LOX opening across operating points,
    // including interpolated thrusts, pressure saturation, and the upper thrust clamp.
    const double matchingPressures[] = {0.0, 500.0, 900.0};
    for (size_t index = 0; index <= Engine::kPointCount * 2; ++index) {
        double thrust;
        if (index == Engine::kPointCount * 2) thrust = 500.0;
        else if (index == Engine::kPointCount * 2 - 1) thrust = 0.0;
        else {
            const size_t low = index / 2;
            thrust = Engine::kThrustLbf[low];
            if (index % 2) thrust = (thrust + Engine::kThrustLbf[low + 1]) / 2.0;
        }
        for (double pressure : matchingPressures) {
            const auto goals = Feedforward::set_thrust_to_mdot(thrust, pressure);
            assert(goals.valid);
            const double waterFlow = Feedforward::equivalent_mass_flow(
                goals.massFlowLbmPerSec, Engine::kLoxDensityLbmPerFt3, density);
            if (thrust > 0.0) assert(waterFlow > 0.0 && waterFlow < goals.massFlowLbmPerSec);
            const double waterAngle = Feedforward::calculate_feedforward(
                waterFlow, pressure, goals.downstreamPressurePsi, density);
            const double loxAngle = Feedforward::calculate_feedforward(
                goals.massFlowLbmPerSec, pressure, goals.downstreamPressurePsi,
                Engine::kLoxDensityLbmPerFt3);
            assert(isfinite(waterAngle) && isfinite(loxAngle));
            assert(near(waterAngle, loxAngle));
        }
    }
    puts("Feedforward regression checks passed.");
}
