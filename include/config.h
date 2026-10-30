/**
 * Shared test settings.
 * - Defines serial speeds, the CAN chip select pin, and motor IDs.
 * - Sets telemetry timeout and logging interval.
 * - Converts valve angles to motor rotations.
 * - Sets the target position tolerance.
 */

#pragma once

#pragma once

#include <Arduino.h>

// TODO: Split config into motor config and logging config
namespace Config {
constexpr unsigned long kBaudRateShort = 115200;  // shorter cable
constexpr unsigned long kBaudRateLong = 57600;    // 50 ft cable

// PWM output port on arduino that CAN shield connects too
constexpr uint8_t kCanCsPin = 10;
constexpr uint8_t kSdCsPin = 9;

// The control period is 150 hz.
constexpr uint32_t kControlPeriodUs = 6667;

// Time left needed in a control cycle for SD write to begin
// kSdWriteEstimateUs + kSdWriteMarginUs
constexpr uint32_t kSdWriteEstimateUs = 2500;
constexpr uint32_t kSdWriteMarginUs = 500;

// About 175 minutes at 200 records/s, including time waiting to start.
constexpr uint32_t kSdPreallocateBytes = 128UL * 1024UL * 1024UL;

constexpr uint8_t kLoxMotorId = 1;
constexpr uint8_t kFuelMotorId = 2;

// determines when telemetry becomes stale
constexpr unsigned long kTelemetryTimeoutMs = 250;
constexpr unsigned long kLogFrequency25Hz = 40;

constexpr double kGearRatio = 32.0017;
constexpr double kOpenTheta = 90.0;
constexpr double kClosedTheta = 0.0;
constexpr double kOpenRotations = (kOpenTheta / 360.0) * kGearRatio;
constexpr double kClosedRotations = (kClosedTheta / 360.0) * kGearRatio;

constexpr double kGoalPositionTolerance = 0.15;  // motor rotations away from goal position

constexpr double kValveCdA[] = {0.00026337, 0.00158019, 0.00553068, 0.01211483, 0.02185936, 0.03555439, 0.05372663, 0.07716618};
constexpr double kValveAngleDeg[] = {27.0, 36.0, 45.0, 54.0, 63.0, 72.0, 81.0, 90.0};
constexpr size_t kValvePointCount = sizeof(kValveCdA) / sizeof(kValveCdA[0]);

// pt constants
//  PressureTransducer pt1 = PressureTransducer(A2, 1000, 0.47, 4.5);
constexpr int kPt1InputPin = A2;
constexpr int kPt2InputPin = A3;
constexpr float kPt1MaxPsi = 500;
constexpr float kPt2MaxPsi = 1000;
constexpr double kPtMinVoltage = 0.0135;
constexpr double kPtMaxVoltage = 5;

// backlash constants
constexpr double backlash = 0.5; // motor rotations

}  // namespace Config

/**
 * lookup tables for engine performance. generally used to create mdot
 * and downstream pressure setpoints for feedforward model
 */
namespace Engine {
// Provisional LOX reference density: 1141 kg/m^3 converted to lbm/ft^3.
// Replace with the density used by the simulation that supplied these tables.
constexpr double kLoxDensityLbmPerFt3 = 1141.0 * 0.0624279605761;

// Parallel operating-point arrays; thrust interpolation uses the lbf axis.
constexpr double kThrustNewtons[] = {754.78, 871.52, 931.78, 976.18, 1031.32, 1111.47, 1200.41, 1289.4, 1378.4, 1467.39, 1556.15, 1645.15, 1734.13, 1823.09, 1910.32, 1989.61, 2053.72, 2098.17};
constexpr double kThrustLbf[] = {169.6813, 195.9255, 209.4725, 219.4540, 231.8500, 249.8684, 269.8629, 289.8687, 309.8766, 329.8824, 349.8364, 369.8444, 389.8479, 409.8469, 429.4570, 447.2821, 461.6946, 471.6874};

constexpr double kChamberPressurePsi[] = {87.89, 136.25, 158.45, 169.31, 178.1, 189.45, 201.97, 214.54, 227.11, 239.69, 252.15, 264.16, 276.1, 287.98, 299.51, 309.14, 316.96, 322.53};
// Both mass-flow tables are kg/s. The feedforward helper converts LOX to lbm/s.
constexpr double kLoxMdotKg[] = {0.3205, 0.3256, 0.3331, 0.3432, 0.3582, 0.3809, 0.406, 0.4311, 0.4563, 0.4814, 0.5063, 0.5302, 0.5541, 0.5778, 0.6014, 0.6228, 0.6439, 0.6648};
constexpr double kFuelMdotKg[] = {0.1781, 0.1809, 0.1851, 0.1906, 0.199, 0.2116, 0.2256, 0.2395, 0.2535, 0.2674, 0.2813, 0.2946, 0.3078, 0.321, 0.3314, 0.3334, 0.3223, 0.3067};
// Drop across the throttle: upstream pressure minus immediate outlet pressure.
constexpr double kValveDpPsi[] = {450.8922, 336.3217, 303.5549, 310.4540, 273.6822, 261.6462, 239.2741, 210.1518, 205.8968, 164.3782, 144.4653, 140.0844, 126.4798, 87.9851, 78.1156, 63.0881, 43.5598, 21.2503};
constexpr double kDownstreamPsi[] = {70.700, 141.150, 172.980, 188.060, 201.750, 218.300,
                                     237.010, 288.970, 304.98, 321.650, 338.670, 334.310,
                                     353.810, 390.760, 408.540, 410.690, 426.680, 451.840};

// all of the above performance characteristics come from general fluid network sim
// these are the timestamps that each element was gotten from. just for reference
constexpr double kThrustTimestampsSeconds[] = {
    0.100, 0.400, 0.800, 1.400, 2.300, 3.300,
    4.300, 5.300, 6.300, 7.300, 8.300, 9.300,
    10.300, 11.300, 12.300, 13.300, 14.300, 15.300};

// length of arrays above. each should be the same. otherwise interpolation cannot work
constexpr size_t kPointCount = sizeof(kThrustLbf) / sizeof(kThrustLbf[0]);

}  // namespace Engine
