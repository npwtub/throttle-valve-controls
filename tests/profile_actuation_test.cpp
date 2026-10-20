#include <stdio.h>
#include <math.h>
#include "test_profile.h"
#include "feedforward_profiles.h"
#include "actuation_sequence.h"
static int checks=0, failures=0;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; printf("FAIL line %d: %s\n",__LINE__,#x); } } while(0)
bool near(float a,float b) { return fabsf(a-b)<0.0001f; }
int main() {
    using namespace TestProfiles;
    CHECK(valid(FeedforwardProfiles::selected()));
    auto steady=FeedforwardProfiles::view(FeedforwardProfiles::steady);
    CHECK(near(sample(steady,0).level,0.5f));
    CHECK(near(sample(steady,9999).level,0.5f));
    CHECK(sample(steady,10000).status==Status::Finished);
    auto ramp=FeedforwardProfiles::view(FeedforwardProfiles::rampUpDown);
    CHECK(near(sample(ramp,0).level,0));
    CHECK(near(sample(ramp,5000).level,0.5f));
    CHECK(near(sample(ramp,10000).level,1));
    CHECK(near(sample(ramp,15000).level,0.5f));
    CHECK(sample(ramp,19999).status==Status::Running);
    CHECK(sample(ramp,20000).status==Status::Finished);
    CHECK(near(sample(ramp,UINT32_MAX).level,0));
    auto steps=FeedforwardProfiles::view(FeedforwardProfiles::steppedExample);
    CHECK(near(sample(steps,4999).level,0.25f));
    CHECK(near(sample(steps,5000).level,0.75f));
    CHECK(near(sample(steps,10000).level,0.4f));
    Point bad[]={{0,0,Transition::Hold},{0,1,Transition::Hold}};
    CHECK(!valid({bad,2})); bad[1].timeMs=1; bad[1].level=NAN;
    CHECK(!valid({bad,2})); bad[1].level=1.1f;
    CHECK(!valid({bad,2})); bad[1].level=1; bad[0].timeMs=1;
    CHECK(!valid({bad,2})); CHECK(!valid({nullptr,2}));
    CHECK(sample({nullptr,0},1).status==Status::Invalid);
    CHECK(isnan(sample({nullptr,0},1).level));
    for (uint32_t t=0;t<=20000;t+=10) {
        const Sample r=sample(ramp,t);
        CHECK(r.level>=0 && r.level<=1);
        const float expected=t<=10000 ? t/10000.0f : (20000-t)/10000.0f;
        CHECK(near(r.level,expected));
    }
    using Actuation::State;
    Actuation::Sequence sequence;
    sequence.update(100000,true); CHECK(sequence.state()==State::Waiting);
    CHECK(sequence.start(100)); CHECK(!sequence.start(100));
    uint32_t now=100;
    for(int cycle=1;cycle<=5;++cycle) {
        CHECK(sequence.state()==State::Closed);
        sequence.update(now+4999,false); CHECK(sequence.state()==State::Closed);
        now+=5000; sequence.update(now,false);
        CHECK(sequence.state()==State::Opening && sequence.targetOpen());
        CHECK(sequence.cycles()==cycle);
        sequence.update(now+50,false); CHECK(sequence.state()==State::Opening);
        now+=100; sequence.update(now,true); CHECK(sequence.state()==State::Open);
        sequence.update(now+4999,true); CHECK(sequence.state()==State::Open);
        now+=5000; sequence.update(now,false); CHECK(sequence.state()==State::Closing);
        now+=100; sequence.update(now,true);
    }
    CHECK(sequence.state()==State::Done && sequence.terminal() && !sequence.targetOpen());
    sequence.update(now+100000,true); CHECK(sequence.state()==State::Done);
    CHECK(!sequence.start(0));
    Actuation::Sequence timeout;
    timeout.start(0); timeout.update(5000,false); timeout.update(6000,false);
    CHECK(timeout.state()==State::Timeout && !timeout.targetOpen());
    Actuation::Sequence late;
    late.start(0); late.update(5000,false); late.update(6001,true);
    CHECK(late.state()==State::Timeout);
    Actuation::Sequence exact;
    exact.start(0); exact.update(5000,false); exact.update(6000,true);
    CHECK(exact.state()==State::Open);
    Actuation::Sequence closingTimeout;
    closingTimeout.start(0); closingTimeout.update(5000,false);
    closingTimeout.update(5100,true); closingTimeout.update(10100,false);
    closingTimeout.update(11100,false); CHECK(closingTimeout.state()==State::Timeout);
    Actuation::Sequence rollover;
    const uint32_t start=UINT32_MAX-1000;
    rollover.start(start); rollover.update(start+5000u,false);
    CHECK(rollover.state()==State::Opening);
    printf("%d checks, %d failures\n",checks,failures);
    return failures ? 1 : 0;
}
