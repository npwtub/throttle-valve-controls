#pragma once
#include <stdint.h>

// Existing actuation test cadence, isolated for host tests. No I/O here.
// Logging STOP must never call or reset this object.
namespace Actuation {
enum class State { Waiting, Closed, Opening, Open, Closing, Done, Timeout };
class Sequence {
public:
    static constexpr uint32_t closedMs = 5000;
    static constexpr uint32_t openMs = 5000;
    static constexpr uint32_t travelTimeoutMs = 1000;
    static constexpr uint8_t totalCycles = 5;
    bool start(uint32_t now) {
        if (state_ != State::Waiting) return false;
        enteredMs_ = now; state_ = State::Closed; return true;
    }
    void update(uint32_t now, bool freshAtTarget) {
        const uint32_t elapsed = now-enteredMs_;
        switch (state_) {
            case State::Closed:
                if (elapsed >= closedMs) { ++cycles_; enter(State::Opening, now); }
                break;
            case State::Opening:
            case State::Closing:
                if (elapsed > travelTimeoutMs) enter(State::Timeout, now);
                else if (freshAtTarget) {
                    if (state_ == State::Opening) enter(State::Open, now);
                    else enter(cycles_ >= totalCycles ? State::Done : State::Closed, now);
                } else if (elapsed >= travelTimeoutMs) enter(State::Timeout, now);
                break;
            case State::Open:
                if (elapsed >= openMs) enter(State::Closing, now);
                break;
            default: break;
        }
    }
    State state() const { return state_; }
    uint8_t cycles() const { return cycles_; }
    bool active() const { return state_ != State::Waiting; }
    bool terminal() const { return state_ == State::Done || state_ == State::Timeout; }
    bool targetOpen() const { return state_ == State::Opening || state_ == State::Open; }
private:
    void enter(State next, uint32_t now) { state_=next; enteredMs_=now; }
    State state_=State::Waiting;
    uint8_t cycles_=0;
    uint32_t enteredMs_=0;
};
}
