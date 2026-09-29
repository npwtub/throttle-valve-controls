#include <Arduino.h>
#include <CanControl.h>
#include <SPI.h>
#include <math.h>
#include <mcp2515.h>

#include "config.h"
#include "motor_state.h"
#include "pressure_transducer.h"

namespace {
MCP2515 mcp2515(Config::CAN_CS_PIN, 4000000UL);
CanControl::SparkMax motor(mcp2515, Config::LOX_MOTOR_ID);
MotorState motorState(Config::LOX_MOTOR_ID);
pressure_transducer pt1(A0, 1000, 0.5, 4.5);
pressure_transducer pt2(A1, 1000, 0.5, 4.5);

// Change this constant to set the closed wait after the initial actuation.
constexpr unsigned long soakTimeMs = 30UL * 60UL * 1000UL;
constexpr unsigned long initialOpenMs = 2000;
constexpr unsigned long finalOpenLogMs = 30000;
constexpr unsigned long logIntervalMs = 40;       // 25 Hz during actuation
constexpr unsigned long soakLogIntervalMs = 500; // 2 Hz during the closed wait
constexpr unsigned long waitingLogIntervalMs = 5000;
constexpr unsigned long controlIntervalMs = 20;
constexpr unsigned long travelTimeoutMs = 3000;  // Same as dwell_csv_test
constexpr double positionTolerance = 0.15;       // Motor rotations

enum class State {
    Waiting, InitialOpening, InitialOpen, Closing, Soaking,
    FinalOpening, FinalOpen, Done, Frozen
};
State state = State::Waiting;
unsigned long stateStart = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastCommand = 0;
unsigned long lastLog = 0;
bool positionReceivedDuringTravel = false;
double targetPosition = Config::CLOSED_ROTATIONS;

void commandPosition(unsigned long now) {
    motor.set_position(targetPosition, 0, 0, 0);
    lastCommand = now;
}

void enterState(State next, unsigned long now) {
    state = next;
    stateStart = now;
    // Start each state's logging cadence from its transition time.
    lastLog = now;
}

void beginTravel(State next, double target, unsigned long now) {
    enterState(next, now);
    targetPosition = target;
    // Cached position from before the command cannot complete this travel.
    positionReceivedDuringTravel = false;
    commandPosition(now);
}

bool atTarget(unsigned long now) {
    return positionReceivedDuringTravel && motorState.hasStatus2() &&
           now - motorState.getLastStatus2Time() <= Config::TELEMETRY_TIMEOUT_MS &&
           isfinite(motorState.getPosition()) &&
           fabs(motorState.getPosition() - targetPosition) <= positionTolerance;
}

const __FlashStringHelper* stateName() {
    switch (state) {
        case State::Waiting: return F("waiting");
        case State::InitialOpening: return F("initial_opening");
        case State::InitialOpen: return F("initial_open");
        case State::Closing: return F("closing");
        case State::Soaking: return F("soaking");
        case State::FinalOpening: return F("final_opening");
        case State::FinalOpen: return F("final_open");
        case State::Done: return F("done");
        case State::Frozen: return F("motor froze");
    }
    return F("unknown");
}

void logRow(unsigned long now) {
    // Match dwell_csv_test's column order, units, and precision.
    // Disable PT voltage debug prints so each line remains a single CSV row.
    Serial.print(pt1.getPressure(false), 2); Serial.print(',');
    Serial.print(pt2.getPressure(false), 2); Serial.print(',');
    Serial.print(now); Serial.print(','); // Milliseconds since boot
    // Convert motor revolutions to valve degrees using the gearbox ratio.
    Serial.print(motorState.getPosition() * 360.0 / Config::GEAR_RATIO, 3); Serial.print(',');
    Serial.print(motorState.getAppliedOutput(), 4); Serial.print(',');
    Serial.print(motorState.getTemperature(), 1); Serial.print(',');
    Serial.print(motorState.getAppliedOutputPercent(), 2); Serial.print(',');
    Serial.print(motorState.getRPM(), 2); Serial.print(',');
    Serial.print(motorState.getVoltageRaw()); Serial.print(',');
    Serial.print(motorState.getCurrentRaw()); Serial.print(',');
    // Validity flags distinguish missing/stale telemetry from actual zeroes.
    Serial.print(motorState.hasStatus0()); Serial.print(',');
    Serial.print(motorState.hasStatus2()); Serial.print(',');
    Serial.print(motorState.hasTelemetry()); Serial.print(',');
    Serial.print(motorState.isFresh(Config::TELEMETRY_TIMEOUT_MS)); Serial.print(',');
    Serial.print(motorState.getLastStatus0Time()); Serial.print(',');
    Serial.print(motorState.getLastStatus2Time()); Serial.print(',');
    Serial.println(stateName());
}

void freeze(unsigned long now) {
    // Initial travel failures abort toward closed, as in the original test.
    // Once final opening begins, even a timeout must never command closed.
    if (state != State::FinalOpening) targetPosition = Config::CLOSED_ROTATIONS;
    enterState(State::Frozen, now);
    commandPosition(now);
    logRow(now); // Final fault row, followed by permanent serial silence.
}
} // namespace

void setup() {
    Serial.begin(Config::BAUD_RATE_LONG);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalOneShotMode();
    Serial.println(F("pt1_psi,pt2_psi,timestamp_ms,position_deg,output,temp_c,output_percent,rpm,voltage_raw,current_raw,has_status0,has_status2,has_telemetry,is_fresh,last_status0_ms,last_status2_ms,state"));
    lastLog = millis();
}

void loop() {
    // Bound receive processing so a busy CAN bus cannot monopolize the loop.
    struct can_frame frame;
    for (uint8_t count = 0; count < 8 &&
         mcp2515.readMessage(&frame) == MCP2515::ERROR_OK; ++count) {
        motor.handle_received_frame(frame);
        if (motorState.processFrame(frame) &&
            (frame.can_id & CAN_EFF_MASK &
             ~CanControl::LowLevel::SparkMax::SPARK_DEVICE_ID_MASK) ==
                CanControl::LowLevel::SparkMax::SPARK_ARB_STATUS_2) {
            positionReceivedDuringTravel = true;
        }
    }
    const unsigned long now = millis();
    // Heartbeats run in every state, including before start and after logging ends.
    if (now - lastHeartbeat >= controlIntervalMs) {
        CanControl::send_heartbeat(mcp2515, CanControl::default_heartbeat());
        lastHeartbeat = now;
    }
    for (uint8_t count = 0; count < 16 && Serial.available() > 0; ++count) {
        // A space starts once; no prompt or newline is required.
        if (Serial.read() == ' ' && state == State::Waiting) {
            beginTravel(State::InitialOpening, Config::OPEN_ROTATIONS, now);
        }
    }

    switch (state) {
        case State::Waiting:
            break;
        case State::InitialOpening:
        case State::Closing:
        case State::FinalOpening:
            // Reject late arrival as well as missing or stale position feedback.
            if (now - stateStart > travelTimeoutMs) {
                freeze(now);
            } else if (atTarget(now)) {
                const State next = state == State::InitialOpening ? State::InitialOpen :
                                   state == State::Closing ? State::Soaking : State::FinalOpen;
                enterState(next, now);
                // Capture the arrival, then continue at this state's log interval.
                logRow(now);
            } else if (now - stateStart >= travelTimeoutMs) {
                freeze(now);
            }
            break;
        case State::InitialOpen:
            // Hold for two full seconds AFTER the valve reaches open.
            if (now - stateStart >= initialOpenMs) {
                beginTravel(State::Closing, Config::CLOSED_ROTATIONS, now);
            }
            break;
        case State::Soaking:
            // The 30-minute wait begins only once closed position is confirmed.
            if (now - stateStart >= soakTimeMs) {
                beginTravel(State::FinalOpening, Config::OPEN_ROTATIONS, now);
            }
            break;
        case State::FinalOpen:
            // Stop logging 30 seconds after confirmed opening, but hold open forever.
            if (now - stateStart >= finalOpenLogMs) enterState(State::Done, now);
            break;
        case State::Done:
        case State::Frozen:
            break;
    }

    // The target remains OPEN after final opening, including in Done/Frozen.
    // Waiting sends no position command; it only receives telemetry and heartbeats.
    if (state != State::Waiting && now - lastCommand >= controlIntervalMs) {
        commandPosition(now);
    }
    const unsigned long interval = state == State::Waiting ? waitingLogIntervalMs :
                                   state == State::Soaking ? soakLogIntervalMs : logIntervalMs;
    if (state != State::Done && state != State::Frozen && now - lastLog >= interval) {
        lastLog += interval;
        // Skip missed samples instead of sending a burst after a delay.
        if (now - lastLog >= interval) lastLog = now;
        logRow(now);
    }
}
