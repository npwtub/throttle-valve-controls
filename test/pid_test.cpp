// Host regression checks for feedback correction, timing, and bounded PID state.
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "pid.h"

static bool near(double actual, double expected) {
    return fabs(actual - expected) < 1e-8;
}

int main() {
    // Logging reads the actual terms from one update, without advancing state.
    PidController logged(2.0, 0.5, 0.25, -0.75, 0.75);
    assert(!logged.has_last_output());
    assert(near(logged.last_output(), 0.0));
    assert(near(logged.update(3.0, 1.0, 0.5), 4.5));
    assert(logged.has_last_output());
    assert(near(logged.last_p_output(), 4.0));
    assert(near(logged.last_i_output(), 0.5));
    assert(near(logged.last_d_output(), 0.0));
    const double mixedOutput = logged.update(4.0, 2.0, 0.5);
    assert(near(mixedOutput, 4.25));
    for (int read = 0; read < 10; ++read) {
        assert(near(logged.last_p_output(), 4.0));
        assert(near(logged.last_i_output(), 0.75)); // Bounded I, not the raw sum.
        assert(near(logged.last_d_output(), -0.5)); // D opposes measured increase.
        assert(near(logged.last_output(), mixedOutput));
        assert(near(logged.last_p_output() + logged.last_i_output() +
                    logged.last_d_output(), mixedOutput));
    }
    assert(near(logged.update(4.0, 2.0, 0.5), 4.75));
    logged.reset();
    assert(!logged.has_last_output());
    assert(near(logged.last_p_output(), 0.0));
    assert(near(logged.last_i_output(), 0.0));
    assert(near(logged.last_d_output(), 0.0));
    assert(near(logged.last_output(), 0.0));

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
        assert(timing.has_last_output());
        assert(near(timing.last_p_output(), 0.0));
        assert(near(timing.last_i_output(), 0.0)); // Stored I is held, not applied.
        assert(near(timing.last_d_output(), 0.0));
        assert(near(timing.last_output(), 0.0));
        assert(near(timing.update(3.0, 3.0, 0.5), 1.0));
        assert(near(timing.last_i_output(), 1.0));
    }

    const double invalidInputs[] = {NAN, INFINITY, -INFINITY};
    for (double input : invalidInputs) {
        PidController invalidGoal(0.0, 1.0, 1.0, -10.0, 10.0);
        assert(near(invalidGoal.update(2.0, 1.0, 1.0), 1.0));
        assert(isnan(invalidGoal.update(input, 20.0, 0.5)));
        assert(!invalidGoal.has_last_output());
        assert(isnan(invalidGoal.last_p_output()));
        assert(isnan(invalidGoal.last_i_output()));
        assert(isnan(invalidGoal.last_d_output()));
        assert(isnan(invalidGoal.last_output()));
        assert(near(invalidGoal.update(3.0, 3.0, 0.5), 1.0));

        PidController invalidMeasurement(0.0, 1.0, 1.0, -10.0, 10.0);
        assert(near(invalidMeasurement.update(2.0, 1.0, 1.0), 1.0));
        assert(isnan(invalidMeasurement.update(20.0, input, 0.5)));
        assert(!invalidMeasurement.has_last_output());
        assert(isnan(invalidMeasurement.last_output()));
        assert(near(invalidMeasurement.update(3.0, 3.0, 0.5), 1.0));
    }

    PidController overflow(1e308, 0.0, 0.0, -1.0, 1.0);
    assert(near(overflow.update(1.0, 1.0, 0.1), 0.0));
    assert(isnan(overflow.update(4.0, 1.0, 0.1)));
    assert(!overflow.has_last_output());
    assert(isnan(overflow.last_p_output()));
    assert(isnan(overflow.last_i_output()));
    assert(isnan(overflow.last_d_output()));
    assert(isnan(overflow.last_output()));

    PidController disabled(0.0, 0.0, 0.0, -1.0, 1.0);
    for (int sample = 0; sample < 100; ++sample) {
        assert(near(disabled.update(10.0, sample, 0.01), 0.0));
    }

    puts("PID regression checks passed.");
}
