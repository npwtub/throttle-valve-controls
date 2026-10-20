/** AI-assisted calculation reference (not the student-authored hardware test). No CAN initialization, heartbeat, or motor commands.
 * It is NOT a replacement controller for an active valve test.
 * Uploading any firmware resets the Mega; use only on an inactive setup.
 */
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include "feedforward_model.h"
#include "feedforward_data.h"
#include "feedforward_profiles.h"
#include <errno.h>

namespace {
char line[96] = {};
uint8_t length = 0;
bool overflow = false;

void report(const Feedforward::Input& input) {
    const uint32_t startUs = micros();
    const Feedforward::Result result = Feedforward::predict(
        FeedforwardData::table(), FeedforwardData::parameters, input);
    const uint32_t elapsedUs = static_cast<uint32_t>(micros() - startUs);
    Serial.print(FeedforwardData::synthetic ? F("SYNTHETIC,") : F("USER_SUPPLIED,"));
    Serial.print(input.upstreamAbsPa, 0); Serial.print(',');
    Serial.print(input.targetDownstreamAbsPa, 0); Serial.print(',');
    Serial.print(input.outletBackpressureAbsPa, 0); Serial.print(',');
    Serial.print(result.requiredCdaMm2, 6); Serial.print(',');
    Serial.print(result.valveAngleDeg, 4); Serial.print(',');
    Serial.print(result.predictedMassFlowKgS, 6); Serial.print(',');
    Serial.print(Feedforward::statusName(result.status)); Serial.print(',');
    Serial.println(elapsedUs);
}

bool parse(const char* text, Feedforward::Input& input) {
    if (strncmp(text, "CALC,", 5) != 0) return false;
    const char* cursor = text + 5;
    float values[3];
    for (uint8_t i = 0; i < 3; ++i) {
        char* end;
        values[i] = static_cast<float>(strtod(cursor, &end));
        if (end == cursor || !Feedforward::finite(values[i])) return false;
        if (i < 2) {
            if (*end != ',') return false;
            cursor = end + 1;
        } else if (*end != '\0') return false;
    }
    input = {values[0], values[1], values[2]};
    return true;
}
void handleLine() {
    if (overflow) { Serial.println(F("# ERROR,input_too_long")); return; }
    if (length && line[length-1] == '\r') --length;
    line[length] = 0;
    if (!length) return;
    if (strcmp(line, "DEMO") == 0) { report(FeedforwardData::demo); return; }
    if (strncmp(line, "PROFILE,", 8) == 0) {
        const char* text = line+8;
        char* end;
        errno = 0;
        const unsigned long elapsed = strtoul(text, &end, 10);
        if (*text < '0' || *text > '9' || end == text || *end || errno == ERANGE) {
            Serial.println(F("# ERROR,PROFILE_requires_elapsed_ms_uint32"));
            return;
        }
        const TestProfiles::Sample sample = TestProfiles::sample(FeedforwardProfiles::selected(), elapsed);
        Serial.print(F("# PROFILE,id=")); Serial.print(FEEDFORWARD_PROFILE_ID);
        Serial.print(F(",elapsed_ms=")); Serial.print(elapsed);
        Serial.print(F(",uncalibrated_level=")); Serial.print(sample.level, 4);
        Serial.print(F(",status="));
        Serial.println(sample.status == TestProfiles::Status::Running ? F("RUNNING") :
                       sample.status == TestProfiles::Status::Finished ? F("FINISHED") : F("INVALID"));
        return;
    }
    Feedforward::Input input;
    if (!parse(line, input)) {
        Serial.println(F("# ERROR,use DEMO or CALC,upstream_abs_pa,target_abs_pa,backpressure_abs_pa"));
        return;
    }
    report(input);
}
}
void setup() {
    Serial.begin(115200);
    Serial.println(F("# FEEDFORWARD_CALCULATION_ONLY_NO_MOTOR_COMMANDS"));
    Serial.println(F("# Synthetic data and assumed outlet model: not calibrated pressure control"));
    Serial.println(F("data_source,upstream_abs_pa,target_abs_pa,backpressure_abs_pa,required_cda_mm2,predicted_angle_deg,predicted_mdot_kg_s,status,calculation_us"));
    report(FeedforwardData::demo);
    Serial.println(F("# PROFILE,<elapsed_ms> previews the selected uncalibrated profile; no motor output"));
    Serial.println(F("# Send DEMO or CALC,500000,300000,100000 followed by newline"));
}
void loop() {
    for (uint8_t count = 0; count < 16 && Serial.available(); ++count) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\n') {
            handleLine(); length = 0; overflow = false;
        } else if (!overflow) {
            if (length < sizeof(line)-1) line[length++] = ch;
            else overflow = true;
        }
    }
}
