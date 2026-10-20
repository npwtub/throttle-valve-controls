> Updated October 20, 2026: the combined review branch is sd-card-logging.
> See HANDOFF_2026-10-20.md for the SD actuation test, profile selection, and
> Marshall's student-authored test exercise. The existing feedforward_test is
> an AI-assisted calculation reference, not a completed flow-test controller.

# Feedforward model: first calculation-only version

This feature belongs in the existing throttle-valve-controls repository, on sd-card-logging. It adds one PlatformIO environment and does not replace the dwell, actuation, pressure, or motor tests.

## What is ready

- include/feedforward_model.h: portable C++ calculation and inverse linear interpolation, with no hardware dependencies.
- include/feedforward_data.h: replaceable synthetic valve CdA table, assumed downstream restriction, density, and demonstration input.
- src/feedforward_test.cpp: Arduino Mega calculation-only program. No CAN initialization, heartbeats, motor position commands, PID, live pressure readings, or SD logging.
- tests/feedforward_model_test.cpp: host numerical tests that exercise the same C++ header the Arduino uses.

This is a software prototype, NOT a calibrated feedforward controller or a pressurized-test controller. Never upload it in place of a controller needed by an active setup: uploading resets the Mega, and this program does not maintain motor heartbeats. It does not command the valve closed.

## Your first step

In PlatformIO Project Tasks, select feedforward_test > General > Build. Without hardware, stop after the successful build. The tested example below shows the expected calculation; real flow testing requires further integration and review.

## Model and units

The requested controlled variable is downstream pressure. The output is predicted valve angle, using a replaceable CdA-versus-angle table.

We explicitly assume a valve followed by a fixed outlet restriction, carrying the same steady, incompressible, single-phase liquid. Neglect pipe losses and pressure recovery. The outlet may represent a test orifice; this assumption has NOT been confirmed for CTS.

For each restriction:

    mass_flow = CdA * sqrt(2 * density * pressure_drop)

Equating the two flows at the requested downstream pressure gives:

    required_valve_CdA = outlet_CdA * sqrt(
        (target_downstream_pressure - outlet_backpressure)
        / (upstream_pressure - target_downstream_pressure)
    )

We then invert the valve table with linear interpolation to obtain angle. Density cancels in the required-area ratio but is used for the reported predicted mass flow. That flow is a prediction, not a flowmeter reading.

All input pressures are ABSOLUTE pascals, not PSI or gauge pressure. Both CdA values use mm^2; flow calculation converts mm^2 to m^2 internally. Density uses kg/m^3. Output angle is valve degrees, not motor revolutions. Conversion to the motor's reference/rotations is deliberately left to a later hardware adapter.

This is an explicit extension of the liquid-flow feedforward approach in Berkeley's eReg paper (Section 2.4, equation 3), using a synthetic downstream-orifice assumption instead of a known nominal flow. It is not a copied equation or fitted model from that paper. Cv and CdA must not be substituted numerically for each other.

Supporting reference for restriction modeling and discharge coefficient:
https://www.mathworks.com/help/simscape/ref/localrestrictionil.html

## Synthetic example

Every number below is a software example, not an operating recommendation:

- Upstream: 500,000 Pa absolute
- Target downstream: 300,000 Pa absolute
- Outlet backpressure: 100,000 Pa absolute
- Outlet CdA: 5 mm^2
- Constant example density: 998 kg/m^3

Both pressure drops are 200,000 Pa, so required valve CdA is 5 mm^2. The SYNTHETIC valve table maps this to 30 valve degrees. Predicted mass flow is approximately 0.099900 kg/s.

Successful code tests validate the arithmetic and interpolation; they do not validate this prediction on a real valve.

## When a board is available for an inactive electronics bench

Build/upload feedforward_test under the team's procedure. Open the serial monitor at 115200 baud and select a newline line ending. It prints a banner, CSV header, and the example on boot.

Send:

    DEMO

Or:

    CALC,500000,300000,100000

The three numeric inputs are upstream absolute Pa, target downstream absolute Pa, and outlet backpressure absolute Pa. The command computes once and prints results; it never moves a motor. This environment is not compatible with valve_log_console.py, which speaks the dwell-test protocol.

CSV columns:

    data_source,upstream_abs_pa,target_abs_pa,backpressure_abs_pa,required_cda_mm2,predicted_angle_deg,predicted_mdot_kg_s,status,calculation_us

calculation_us measures only the computation, excluding serial parsing/output. Serial printing can block; this is not a real-time control timing benchmark. The input line is bounded to 95 characters and rejects malformed/non-finite values.

## Replacing the example data later

Edit include/feedforward_data.h and rebuild:

1. Replace valveTable with validated valve-angle/CdA pairs. Angles must strictly increase from within 0-90 degrees. CdA must be nonnegative and nondecreasing. A zero-CdA deadband is allowed. If measurements are noisy or non-monotonic, fit/select a physically justified monotonic branch rather than silently sorting CdA independently of angle. Opening/closing hysteresis is not modeled.
2. Replace outletCdaMm2 with a characterized outlet value, or replace the outlet model if a fixed restriction is not appropriate. A calibrated valve table alone does not calibrate downstream pressure prediction.
3. Confirm density, actual sensor locations, sensor calibration, pressure reference, and operating range. minValveDropPa=1000 is an illustrative numerical margin, NOT a safety limit or cavitation criterion.
4. synthetic=true marks example data. Only change it after all example inputs/model parameters have been replaced or explicitly accepted. USER_SUPPLIED does not mean independently validated.

The model does not handle gas flow, cavitation, boiling, compressibility, transients, pressure-varying CdA, variable downstream restrictions, or two-phase cryogenic flow. Do not treat water/constant-density assumptions as a validated LN2 or LOX model.

## Invalid and unreachable requests

Results contain a status. Call valid() before using an angle. Invalid/unreachable cases leave valveAngleDeg as NaN, not a fallback angle or a close command.

- INVALID_TABLE: null/short, non-finite, decreasing CdA, unordered/out-of-range angles, or entirely flat table.
- INVALID_PARAMETERS: invalid outlet CdA, density, or pressure-drop margin.
- INVALID_INPUT: non-finite or nonpositive absolute pressure.
- PRESSURE_ORDER: target below backpressure or upstream no greater than target.
- INSUFFICIENT_VALVE_DP: available valve pressure drop below configured margin.
- BELOW_TABLE / ABOVE_TABLE: required CdA outside table limits. No extrapolation or silent saturation.
- NUMERICAL_ERROR: invalid numeric result.

At exactly target=backpressure, the steady model predicts zero flow. If the table contains zero CdA, the smallest corresponding angle is returned as a calculation only. At a CdA plateau, the smallest tabulated angle for that exact CdA is selected.

## Remaining integration before a flow-control test

Confirm the downstream restriction assumption and parameters, and obtain the actual controller/test program to integrate with. Add upstream sensor sampling and a requested-pressure source; map predicted angle into the verified motor coordinate system and preserve existing CAN handling and heartbeats. Log commanded pressure, measured downstream pressure, predicted angle, actual angle, model status, and synthetic-data flag. The outer pressure PID remains disabled for a feedforward-only test; the motor's position loop remains active in the eventual integrated test.

This version intentionally does not implement those hardware actions. Missing/invalid model results must be handled using the team's existing test procedure, not an invented automatic valve-close behavior.

## Host tests for developers

With a C++11 compiler installed, compile and run the actual numerical tests, for example from this project root:

    c++ -std=c++11 -Wall -Wextra -I include tests/feedforward_model_test.cpp -o feedforward_model_test
    ./feedforward_model_test

On Windows, the executable is typically feedforward_model_test.exe. A compiler is not required just to build the Arduino environment using PlatformIO. Do not commit generated host binaries or build caches.

## Interchangeable profiles (October 20)

include/test_profile.h contains a pure timeline sampler. include/feedforward_profiles.h
selects a steady, ramp-up/down, or stepped example with FEEDFORWARD_PROFILE_ID.
The calculation reference also accepts PROFILE,<elapsed_ms> to preview one sample.
Levels are uncalibrated 0..1 requests; no linear thrust-to-pressure mapping is assumed.
The lead will specify final timing, mapping and the third profile. See the handoff
for Marshall's small test-writing exercise and expected results.
