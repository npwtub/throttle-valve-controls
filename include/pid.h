#pragma once

#include <math.h>

/**
 * Signed correction to add to a feedforward motor-position target. The throttle
 * loop supplies goal/measured chamber pressure in psi; output and I limits use
 * motor rotations.
 */
class PidController {
public:
    // Kp [rot/psi], Ki [rot/(psi*s)], Kd [rot*s/psi]
    PidController(double kp, double ki, double kd,
                  double minIntegral, double maxIntegral)
        : kp_(kp), ki_(ki), kd_(kd),
          minIntegral_(minIntegral), maxIntegral_(maxIntegral) {}

    /**
     * Goal and measurement share input units (psi for chamber pressure); returns
     * a motor-rotation correction. dtSeconds is actual elapsed time between calls. 
     * the seconds are converted from microseconds in the calling function
     * Nonpositive/nonfinite dt returns zero and holds I; 
     * nonfinite inputs/results return NAN and hold I. Either invalidates D
     * history so the next valid call starts without a derivative spike.
     */
    double update(double goalChamberPressurePSI, double measuredChamberPressurePSI, double dtSeconds) {
        // 0 output for invalid dt(negative or nonfinite)
        if (!isfinite(dtSeconds) || dtSeconds <= 0.0) {
            hasPreviousMeasurement_ = false;
            cache_output(0.0, 0.0, 0.0, 0.0);
            return 0.0;
        }
        // invalid goal or measurement
        if (!isfinite(goalChamberPressurePSI) || !isfinite(measuredChamberPressurePSI)) {
            hasPreviousMeasurement_ = false;
            invalidate_output();
            return NAN;
        }

        const double error = goalChamberPressurePSI - measuredChamberPressurePSI;
        // Store the Ki-scaled contribution itself, so no hidden accumulator can
        // grow past the output-unit limits and delay unwinding after saturation.
        double integral = 0.0;
        if (ki_ != 0.0) {
            integral = integralContribution_ + ki_ * error * dtSeconds;
            if (integral < minIntegral_) integral = minIntegral_;
            if (integral > maxIntegral_) integral = maxIntegral_;
        }

        double derivative = 0.0;
        // D opposes changes in measurement, avoiding a kick when only the goal
        // changes. The first valid update establishes history without D output.
        if (kd_ != 0.0 && hasPreviousMeasurement_) {
            derivative = -kd_ * (measuredChamberPressurePSI - previousMeasurement_) / dtSeconds;
        }

        const double proportional = kp_ * error;
        const double correction = proportional + integral + derivative;
        if (!isfinite(correction)) {
            hasPreviousMeasurement_ = false;
            invalidate_output();
            return NAN;
        }
        integralContribution_ = integral;
        previousMeasurement_ = measuredChamberPressurePSI;
        hasPreviousMeasurement_ = true;
        cache_output(proportional, integral, derivative, correction);
        return correction;
    }

    // Cached contributions from the last update, in motor rotations. Reading
    // these never advances I or D state. Invalid updates expose NAN and false;
    // skipped timing exposes the zero output returned while holding internal I.
    double last_p_output() const { return lastPOutput_; }
    double last_i_output() const { return lastIOutput_; }
    double last_d_output() const { return lastDOutput_; }
    double last_output() const { return lastOutput_; }
    bool has_last_output() const { return hasLastOutput_; }

    // Start a new run with zero accumulated I and no measurement history for D.
    void reset() {
        integralContribution_ = 0.0;
        previousMeasurement_ = 0.0;
        hasPreviousMeasurement_ = false;
        lastPOutput_ = lastIOutput_ = lastDOutput_ = lastOutput_ = 0.0;
        hasLastOutput_ = false;
    }

private:
    void cache_output(double proportional, double integral, double derivative,
                      double output) {
        lastPOutput_ = proportional;
        lastIOutput_ = integral;
        lastDOutput_ = derivative;
        lastOutput_ = output;
        hasLastOutput_ = true;
    }

    void invalidate_output() {
        lastPOutput_ = lastIOutput_ = lastDOutput_ = lastOutput_ = NAN;
        hasLastOutput_ = false;
    }

    double kp_;
    double ki_;
    double kd_;
    double minIntegral_;
    double maxIntegral_;
    double integralContribution_ = 0.0;
    double previousMeasurement_ = 0.0;
    bool hasPreviousMeasurement_ = false;
    double lastPOutput_ = 0.0;
    double lastIOutput_ = 0.0;
    double lastDOutput_ = 0.0;
    double lastOutput_ = 0.0;
    bool hasLastOutput_ = false;
};
