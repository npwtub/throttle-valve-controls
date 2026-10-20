#pragma once
#include <math.h>
#include <stddef.h>

// No Arduino, CAN, sensor, storage, or motor dependencies.
// Units: Pa absolute, kg/m^3, mm^2 effective area, valve degrees.
namespace Feedforward {
struct Point { float angleDeg; float cdaMm2; };
struct Table { const Point* points; size_t count; };
struct Parameters {
    float outletCdaMm2;
    float densityKgM3;
    float minValveDropPa;
};
struct Input {
    float upstreamAbsPa;
    float targetDownstreamAbsPa;
    float outletBackpressureAbsPa;
};
enum class Status {
    Ok, InvalidTable, InvalidParameters, InvalidInput,
    PressureOrder, InsufficientValveDrop, BelowTable, AboveTable,
    NumericalError
};
struct Result {
    Status status = Status::InvalidInput;
    float requiredCdaMm2 = NAN;
    float valveAngleDeg = NAN;
    float predictedMassFlowKgS = NAN;
    bool valid() const { return status == Status::Ok; }
};
inline bool finite(float value) { return isfinite(value); }
inline bool validTable(Table table) {
    if (!table.points || table.count < 2) return false;
    for (size_t i = 0; i < table.count; ++i) {
        const Point& p = table.points[i];
        if (!finite(p.angleDeg) || !finite(p.cdaMm2) ||
            p.angleDeg < 0 || p.angleDeg > 90 || p.cdaMm2 < 0) return false;
        if (i && (p.angleDeg <= table.points[i-1].angleDeg ||
                  p.cdaMm2 < table.points[i-1].cdaMm2)) return false;
    }
    return table.points[table.count-1].cdaMm2 > table.points[0].cdaMm2;
}
// Nondecreasing CdA permits a closed-valve deadband. At any plateau value,
// return the smallest tabulated angle. No extrapolation or silent saturation.
inline Result angleForCda(Table table, float required) {
    Result result;
    if (!validTable(table)) { result.status = Status::InvalidTable; return result; }
    if (!finite(required) || required < 0) return result;
    result.requiredCdaMm2 = required;
    if (required < table.points[0].cdaMm2) {
        result.status = Status::BelowTable; return result;
    }
    if (required > table.points[table.count-1].cdaMm2) {
        result.status = Status::AboveTable; return result;
    }
    for (size_t i = 0; i < table.count; ++i) {
        const Point& high = table.points[i];
        if (required == high.cdaMm2) {
            result.valveAngleDeg = high.angleDeg;
            result.status = Status::Ok;
            return result;
        }
        if (required < high.cdaMm2) {
            const Point& low = table.points[i-1];
            const float fraction = (required - low.cdaMm2) / (high.cdaMm2 - low.cdaMm2);
            result.valveAngleDeg = low.angleDeg + fraction * (high.angleDeg - low.angleDeg);
            result.status = Status::Ok;
            return result;
        }
    }
    result.status = Status::NumericalError;
    return result;
}

// Steady, incompressible, single-phase liquid through two restrictions in series:
// mdot = CdA * sqrt(2*rho*deltaP).
// Equating valve and outlet flows gives:
// valveCdA = outletCdA * sqrt((target-backpressure)/(upstream-target)).
// This is a replaceable ASSUMED outlet model, not a calibrated CTS model.
inline Result predict(Table table, const Parameters& params, const Input& input) {
    Result result;
    if (!validTable(table)) { result.status = Status::InvalidTable; return result; }
    if (!finite(params.outletCdaMm2) || params.outletCdaMm2 <= 0 ||
        !finite(params.densityKgM3) || params.densityKgM3 <= 0 ||
        !finite(params.minValveDropPa) || params.minValveDropPa <= 0) {
        result.status = Status::InvalidParameters; return result;
    }
    if (!finite(input.upstreamAbsPa) || input.upstreamAbsPa <= 0 ||
        !finite(input.targetDownstreamAbsPa) || input.targetDownstreamAbsPa <= 0 ||
        !finite(input.outletBackpressureAbsPa) || input.outletBackpressureAbsPa <= 0)
        return result;
    if (input.targetDownstreamAbsPa < input.outletBackpressureAbsPa ||
        input.upstreamAbsPa <= input.targetDownstreamAbsPa) {
        result.status = Status::PressureOrder; return result;
    }
    const float valveDrop = input.upstreamAbsPa - input.targetDownstreamAbsPa;
    if (valveDrop < params.minValveDropPa) {
        result.status = Status::InsufficientValveDrop; return result;
    }
    const float outletDrop = input.targetDownstreamAbsPa - input.outletBackpressureAbsPa;
    const float cda = params.outletCdaMm2 * sqrtf(outletDrop / valveDrop);
    const float mdot = params.outletCdaMm2 * 1e-6f * sqrtf(2.0f * params.densityKgM3 * outletDrop);
    if (!finite(cda) || !finite(mdot)) {
        result.status = Status::NumericalError; return result;
    }
    result = angleForCda(table, cda);
    result.predictedMassFlowKgS = mdot;
    return result;
}
inline const char* statusName(Status status) {
    switch (status) {
        case Status::Ok: return "OK";
        case Status::InvalidTable: return "INVALID_TABLE";
        case Status::InvalidParameters: return "INVALID_PARAMETERS";
        case Status::InvalidInput: return "INVALID_INPUT";
        case Status::PressureOrder: return "PRESSURE_ORDER";
        case Status::InsufficientValveDrop: return "INSUFFICIENT_VALVE_DP";
        case Status::BelowTable: return "BELOW_TABLE";
        case Status::AboveTable: return "ABOVE_TABLE";
        default: return "NUMERICAL_ERROR";
    }
}
} // namespace Feedforward
