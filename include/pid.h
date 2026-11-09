#pragma once

#include <math.h>

/**
 * Signed correction to add to a feedforward motor-position target. The throttle
 * loop supplies goal/measured chamber pressure in psi; output and I limits use
 * motor rotations. Positive pressure error opens the valve for positive gains.
 * Motor I/O, backlash, and final limits are external.
 */
class PidController {
public:
    // For pressure input, gains use Kp [rot/psi], Ki [rot/(psi*s)], Kd [rot*s/psi].
    // Supply finite gains and I bounds with minIntegral <= 0 <= maxIntegral [rot].
    PidController(double kp, double ki, double kd,
                  double minIntegral, double maxIntegral)
        : kp_(kp), ki_(ki), kd_(kd),
          minIntegral_(minIntegral), maxIntegral_(maxIntegral) {}

    /**
     * Goal and measurement share input units (psi for chamber pressure); returns
     * a motor-rotation correction using error = goal - measurement. dtSeconds is
     * actual elapsed time between calls. Nonpositive/nonfinite dt returns zero
     * and holds I; nonfinite inputs/results return NAN and hold I. Either invalidates D
     * history so the next valid call starts without a derivative spike.
     */
    double update(double goal, double measurement, double dtSeconds) {
        if (!isfinite(dtSeconds) || dtSeconds <= 0.0) {
            hasPreviousMeasurement_ = false;
            return 0.0;
        }
        if (!isfinite(goal) || !isfinite(measurement)) {
            hasPreviousMeasurement_ = false;
            return NAN;
        }

        const double error = goal - measurement;
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
            derivative = -kd_ * (measurement - previousMeasurement_) / dtSeconds;
        }

        const double correction = kp_ * error + integral + derivative;
        if (!isfinite(correction)) {
            hasPreviousMeasurement_ = false;
            return NAN;
        }
        integralContribution_ = integral;
        previousMeasurement_ = measurement;
        hasPreviousMeasurement_ = true;
        return correction;
    }

    // Start a new run with zero accumulated I and no measurement history for D.
    void reset() {
        integralContribution_ = 0.0;
        previousMeasurement_ = 0.0;
        hasPreviousMeasurement_ = false;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double minIntegral_;
    double maxIntegral_;
    double integralContribution_ = 0.0;
    double previousMeasurement_ = 0.0;
    bool hasPreviousMeasurement_ = false;
};
