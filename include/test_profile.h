#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

// Pure timeline sampler. It does NOT turn a level into thrust, pressure, or
// angle, and never sends motor commands. A caller owns start/stop behavior.
namespace TestProfiles {
enum class Transition { Hold, Linear };
struct Point {
    uint32_t timeMs;
    float level;             // normalized request, 0..1; NOT calibrated thrust
    Transition toNext;
};
struct Profile { const Point* points; size_t count; };
enum class Status { Invalid, Running, Finished };
struct Sample {
    Status status;
    float level;
};
inline bool valid(Profile p) {
    if (!p.points || p.count < 2 || p.points[0].timeMs != 0) return false;
    for (size_t i=0; i<p.count; ++i) {
        const Point& point=p.points[i];
        if (!isfinite(point.level) || point.level < 0 || point.level > 1 ||
            (point.toNext != Transition::Hold && point.toNext != Transition::Linear)) return false;
        if (i && point.timeMs <= p.points[i-1].timeMs) return false;
    }
    return true;
}
inline Sample sample(Profile p, uint32_t elapsedMs) {
    if (!valid(p)) return {Status::Invalid, NAN};
    if (elapsedMs >= p.points[p.count-1].timeMs)
        return {Status::Finished, p.points[p.count-1].level};
    for (size_t i=1; i<p.count; ++i) {
        if (elapsedMs < p.points[i].timeMs) {
            const Point& low=p.points[i-1];
            const Point& high=p.points[i];
            if (low.toNext == Transition::Hold) return {Status::Running, low.level};
            const float fraction=static_cast<float>(elapsedMs-low.timeMs)/(high.timeMs-low.timeMs);
            return {Status::Running, low.level+fraction*(high.level-low.level)};
        }
    }
    return {Status::Invalid, NAN};
}
}
