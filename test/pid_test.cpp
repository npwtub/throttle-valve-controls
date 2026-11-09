// Host regression checks for feedback correction, timing, and bounded PID state.
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "pid.h"

static bool near(double actual, double expected) {
    return fabs(actual - expected) < 1e-8;
}

int main() {
    PidController proportional(2.0, 0.0, 0.0, -1.0, 1.0);
    assert(near(proportional.update(3.0, 1.0, 0.1), 4.0));
    assert(near(proportional.update(1.0, 2.0, 0.1), -2.0));
    assert(near(proportional.update(2.0, 2.0, 0.1), 0.0));

    // Equal elapsed seconds produce equal I contributions despite different steps.
    PidController shortSteps(0.0, 2.0, 0.0, -10.0, 10.0);
    PidController longStep(0.0, 2.0, 0.0, -10.0, 10.0);
    assert(near(shortSteps.update(2.0, 1.0, 0.2), 0.4));
    const double accumulated = shortSteps.update(2.0, 1.0, 0.3);
    assert(near(accumulated, 1.0));
    assert(near(accumulated, longStep.update(2.0, 1.0, 0.5)));

    // Reversing error immediately unwinds either limit, even after prolonged saturation.
    PidController boundedIntegral(0.0, 1.0, 0.0, -0.25, 0.75);
    for (int sample = 0; sample < 100; ++sample) {
        assert(near(boundedIntegral.update(100.0, 0.0, 0.1), 0.75));
    }
    assert(near(boundedIntegral.update(0.0, 0.1, 0.5), 0.70));
    for (int sample = 0; sample < 100; ++sample) {
        assert(near(boundedIntegral.update(0.0, 100.0, 0.1), -0.25));
    }
    assert(near(boundedIntegral.update(0.1, 0.0, 0.5), -0.20));

    PidController zeroIntegralRange(0.0, 1.0, 0.0, 0.0, 0.0);
    assert(near(zeroIntegralRange.update(10.0, 0.0, 1.0), 0.0));
    assert(near(zeroIntegralRange.update(0.0, 10.0, 1.0), 0.0));

    // The first measurement seeds D; goal changes alone cannot produce D output.
    PidController derivative(0.0, 0.0, 0.5, -1.0, 1.0);
    assert(near(derivative.update(3.0, 1.0, 0.1), 0.0));
    assert(near(derivative.update(8.0, 1.0, 0.1), 0.0));
    assert(near(derivative.update(8.0, 1.2, 0.1), -1.0));
    assert(near(derivative.update(8.0, 1.1, 0.2), 0.25));
    derivative.reset();
    assert(near(derivative.update(3.0, 9.0, 0.1), 0.0));

    PidController resetState(0.0, 1.0, 1.0, -10.0, 10.0);
    assert(near(resetState.update(2.0, 1.0, 0.5), 0.5));
    assert(near(resetState.update(2.0, 1.25, 0.5), 0.375));
    resetState.reset();
    assert(near(resetState.update(4.0, 3.0, 0.5), 0.5));
    resetState.reset();
    assert(near(resetState.update(3.0, 3.0, 0.5), 0.0));

    // Invalid timing must not accumulate I or differentiate across the skipped step.
    const double invalidIntervals[] = {0.0, -0.1, NAN, INFINITY, -INFINITY};
    for (double interval : invalidIntervals) {
        PidController timing(0.0, 1.0, 1.0, -10.0, 10.0);
        assert(near(timing.update(2.0, 1.0, 1.0), 1.0));
        assert(near(timing.update(100.0, 20.0, interval), 0.0));
        assert(near(timing.update(3.0, 3.0, 0.5), 1.0));
    }

    const double invalidInputs[] = {NAN, INFINITY, -INFINITY};
    for (double input : invalidInputs) {
        PidController invalidGoal(0.0, 1.0, 1.0, -10.0, 10.0);
        assert(near(invalidGoal.update(2.0, 1.0, 1.0), 1.0));
        assert(isnan(invalidGoal.update(input, 20.0, 0.5)));
        assert(near(invalidGoal.update(3.0, 3.0, 0.5), 1.0));

        PidController invalidMeasurement(0.0, 1.0, 1.0, -10.0, 10.0);
        assert(near(invalidMeasurement.update(2.0, 1.0, 1.0), 1.0));
        assert(isnan(invalidMeasurement.update(20.0, input, 0.5)));
        assert(near(invalidMeasurement.update(3.0, 3.0, 0.5), 1.0));
    }

    PidController disabled(0.0, 0.0, 0.0, -1.0, 1.0);
    for (int sample = 0; sample < 100; ++sample) {
        assert(near(disabled.update(10.0, sample, 0.01), 0.0));
    }

    puts("PID regression checks passed.");
}
