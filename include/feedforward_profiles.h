#pragma once
#include "test_profile.h"

// Illustrative timings/levels only. Noah has not specified the final profiles.
// Change this selection to use the same sampler with a different profile:
#ifndef FEEDFORWARD_PROFILE_ID
#define FEEDFORWARD_PROFILE_ID 0
#endif
namespace FeedforwardProfiles {
using TestProfiles::Point;
using TestProfiles::Transition;
constexpr Point steady[] = {
    {0, 0.5f, Transition::Hold}, {10000, 0.5f, Transition::Hold}
};
constexpr Point rampUpDown[] = {
    {0, 0.0f, Transition::Linear},
    {10000, 1.0f, Transition::Linear},
    {20000, 0.0f, Transition::Hold}
};
// A PLACEHOLDER stepped pattern; NOT a reproduction of Noah's pending profile.
constexpr Point steppedExample[] = {
    {0, 0.25f, Transition::Hold}, {5000, 0.75f, Transition::Hold},
    {10000, 0.4f, Transition::Hold}, {15000, 0.4f, Transition::Hold}
};
template <size_t N>
inline TestProfiles::Profile view(const Point (&points)[N]) { return {points, N}; }
inline TestProfiles::Profile selected() {
#if FEEDFORWARD_PROFILE_ID == 0
    return view(steady);
#elif FEEDFORWARD_PROFILE_ID == 1
    return view(rampUpDown);
#elif FEEDFORWARD_PROFILE_ID == 2
    return view(steppedExample);
#else
#error "FEEDFORWARD_PROFILE_ID must be 0, 1, or 2"
#endif
}
// Optional demonstration mapping only. There is deliberately NO default
// thrust-to-pressure conversion. A real test must supply its reviewed mapping.
}
