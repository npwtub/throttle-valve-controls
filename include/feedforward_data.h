#pragma once
#include "feedforward_model.h"

namespace FeedforwardData {
// SYNTHETIC: invented examples, NOT measured CdA values or recommended angles.
// Replace the entire table with validated data in valve degrees and mm^2.
// Angles must increase; CdA must not decrease. Zero-CdA plateaus are supported.
constexpr bool synthetic = true;
constexpr Feedforward::Point valveTable[] = {
    {0, 0}, {10, 0}, {20, 2}, {30, 5}, {40, 10},
    {50, 16}, {60, 24}, {70, 33}, {80, 42}, {90, 50}
};
inline Feedforward::Table table() {
    return {valveTable, sizeof(valveTable) / sizeof(valveTable[0])};
}
// These are example assumptions, NOT CTS characterization:
// outlet effective area 5 mm^2; constant density 998 kg/m^3;
// computational minimum valve deltaP 1000 Pa (not a cavitation criterion).
constexpr Feedforward::Parameters parameters = {5.0f, 998.0f, 1000.0f};
constexpr Feedforward::Input demo = {500000.0f, 300000.0f, 100000.0f};
}
