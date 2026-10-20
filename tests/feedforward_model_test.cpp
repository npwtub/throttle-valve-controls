#include <stdio.h>
#include <math.h>
#include "feedforward_model.h"
#include "feedforward_data.h"
using namespace Feedforward;
static int checks = 0, failures = 0;
#define CHECK(expr) do { ++checks; if (!(expr)) { ++failures; printf("FAIL line %d: %s\n", __LINE__, #expr); } } while (0)
static bool near(float a, float b, float tolerance = 0.0001f) { return fabsf(a-b) <= tolerance; }
int main() {
    const Table t = FeedforwardData::table();
    const Parameters p = FeedforwardData::parameters;
    CHECK(validTable(t));
    CHECK(!validTable({nullptr, 2}));
    CHECK(!validTable({t.points, 1}));
    Point wrongAngle[] = {{20, 1}, {10, 2}};
    Point wrongCda[] = {{0, 2}, {10, 1}};
    Point flat[] = {{0, 1}, {10, 1}};
    Point nonfinite[] = {{0, 0}, {10, NAN}};
    Point overangle[] = {{0, 0}, {91, 1}};
    CHECK(!validTable({wrongAngle, 2}));
    CHECK(!validTable({wrongCda, 2}));
    CHECK(!validTable({flat, 2}));
    CHECK(!validTable({nonfinite, 2}));
    CHECK(!validTable({overangle, 2}));
    CHECK(near(angleForCda(t, 0).valveAngleDeg, 0));
    CHECK(near(angleForCda(t, 1).valveAngleDeg, 15));
    CHECK(near(angleForCda(t, 5).valveAngleDeg, 30));
    CHECK(near(angleForCda(t, 7.5f).valveAngleDeg, 35));
    CHECK(near(angleForCda(t, 50).valveAngleDeg, 90));
    Point plateau[] = {{0, 0}, {20, 5}, {30, 5}, {40, 10}};
    CHECK(near(angleForCda({plateau, 4}, 5).valveAngleDeg, 20));
    CHECK(near(angleForCda({plateau, 4}, 7.5f).valveAngleDeg, 35));
    Point leak[] = {{0, 1}, {90, 50}};
    CHECK(angleForCda({leak, 2}, 0).status == Status::BelowTable);
    CHECK(angleForCda(t, 51).status == Status::AboveTable);
    CHECK(isnan(angleForCda(t, 51).valveAngleDeg));
    CHECK(angleForCda(t, -1).status == Status::InvalidInput);
    CHECK(angleForCda(t, INFINITY).status == Status::InvalidInput);
    Result r = predict(t, p, {500000, 300000, 100000});
    CHECK(r.valid());
    CHECK(near(r.requiredCdaMm2, 5));
    CHECK(near(r.valveAngleDeg, 30));
    CHECK(near(r.predictedMassFlowKgS, 0.09989995f, 0.000001f));
    r = predict(t, p, {500000, 100000, 100000});
    CHECK(r.valid() && near(r.predictedMassFlowKgS, 0) && near(r.valveAngleDeg, 0));
    CHECK(predict(t, p, {500000, 500000, 100000}).status == Status::PressureOrder);
    CHECK(predict(t, p, {500000, 600000, 100000}).status == Status::PressureOrder);
    CHECK(predict(t, p, {500000, 90000, 100000}).status == Status::PressureOrder);
    CHECK(predict(t, p, {500000, 499500, 100000}).status == Status::InsufficientValveDrop);
    CHECK(predict(t, p, {500000, 499000, 100000}).status == Status::AboveTable);
    CHECK(predict(t, p, {NAN, 300000, 100000}).status == Status::InvalidInput);
    CHECK(predict(t, p, {500000, 300000, 0}).status == Status::InvalidInput);
    CHECK(predict(t, {0, 998, 1000}, FeedforwardData::demo).status == Status::InvalidParameters);
    CHECK(predict(t, {5, -1, 1000}, FeedforwardData::demo).status == Status::InvalidParameters);
    CHECK(predict(t, {5, 998, 0}, FeedforwardData::demo).status == Status::InvalidParameters);
    CHECK(predict(t, {5, INFINITY, 1000}, FeedforwardData::demo).status == Status::InvalidParameters);
    // Density cancels from the pressure-ratio angle calculation, but not flow.
    r = predict(t, {5, 249.5f, 1000}, FeedforwardData::demo);
    CHECK(near(r.valveAngleDeg, 30));
    CHECK(near(r.predictedMassFlowKgS, 0.049949975f, 0.000001f));
    float previousAngle = -1;
    // Check against the forward two-restriction pressure equation, independently
    // reconstructing CdA from the returned angle across a pressure sweep.
    for (int target = 101000; target <= 490000; target += 1000) {
        r = predict(t, p, {500000, static_cast<float>(target), 100000});
        CHECK(r.valid());
        CHECK(r.valveAngleDeg >= previousAngle);
        previousAngle = r.valveAngleDeg;
        float recovered = NAN;
        for (size_t i=1; i<t.count; ++i) {
            if (r.valveAngleDeg <= t.points[i].angleDeg) {
                const Point a=t.points[i-1], b=t.points[i];
                const float fraction=(r.valveAngleDeg-a.angleDeg)/(b.angleDeg-a.angleDeg);
                recovered=a.cdaMm2+fraction*(b.cdaMm2-a.cdaMm2);
                break;
            }
        }
        const float predictedPressure=(recovered*recovered*500000+p.outletCdaMm2*p.outletCdaMm2*100000)/
                                      (recovered*recovered+p.outletCdaMm2*p.outletCdaMm2);
        CHECK(near(predictedPressure, static_cast<float>(target), 0.2f));
    }
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
