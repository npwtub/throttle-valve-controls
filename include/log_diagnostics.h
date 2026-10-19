#pragma once
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "log.h"

struct LogTiming {
    uint32_t maxLoopUs = 0, loopOverBudget = 0;
    uint32_t maxHeartbeatGapUs = 0, maxCommandGapUs = 0;
    uint32_t maxCaptureUs = 0, maxServiceUs = 0, initUs = 0;
    uint32_t lastLoop = 0, lastHeartbeat = 0, lastCommand = 0;
    bool active = false, heartbeatSeen = false, commandSeen = false;
    static void maximum(uint32_t& value, uint32_t sample) {
        if (sample > value) value = sample;
    }
    void start(uint32_t now) { active = true; lastLoop = now; }
    void loop(uint32_t now) {
        if (!active) return;
        const uint32_t gap = now - lastLoop;
        maximum(maxLoopUs, gap);
        if (gap > LogConfig::loopBudgetUs) ++loopOverBudget;
        lastLoop = now;
    }
    void heartbeat(uint32_t now) {
        if (!active) return;
        if (heartbeatSeen) maximum(maxHeartbeatGapUs, now - lastHeartbeat);
        heartbeatSeen = true; lastHeartbeat = now;
    }
    void command(uint32_t now) {
        if (!active) return;
        if (commandSeen) maximum(maxCommandGapUs, now - lastCommand);
        commandSeen = true; lastCommand = now;
    }
};

// One immutable snapshot, sent one short line at a time only when TX has room.
// Called after control work. Repeated requests are coalesced by the caller.
class LogDiagnostics {
public:
    bool busy() const { return stage_ != 255; }
    void begin(const SdCsvLogger& log, const LogTiming& t,
               uint32_t missed, uint32_t statusDrops, uint32_t controlMs) {
        if (busy()) return;
        strncpy(filename_, log.filename(), sizeof(filename_));
        filename_[sizeof(filename_) - 1] = 0;
        const uint32_t snapshot[] = {
            static_cast<uint32_t>(log.status()), log.queuedRecords(), missed,
            log.queueFullRecords(), log.oversizedRecords(), log.peakQueuedBytes(),
            log.queuedBytes(), log.bytesWritten(), log.maxWriteUs(), log.closeUs(),
            t.maxServiceUs, t.maxCaptureUs, t.maxLoopUs, t.loopOverBudget,
            t.maxHeartbeatGapUs, t.maxCommandGapUs, log.capacityReached(),
            log.errorCode(), statusDrops, t.initUs, LogConfig::loopBudgetUs,
            controlMs, LogConfig::intervalMs
        };
        memcpy(values_, snapshot, sizeof(values_));
        stage_ = 0;
    }
    void service() {
        if (!busy()) return;
        char line[64];
        int count;
        if (stage_ == 0) count = snprintf_P(line, sizeof(line), PSTR("# STATS_BEGIN\n"));
        else if (stage_ == 1) count = snprintf_P(line, sizeof(line), PSTR("# FILE,%s\n"), filename_);
        else if (stage_ == 25) count = snprintf_P(line, sizeof(line), PSTR("# STATS_END\n"));
        else count = snprintf_P(line, sizeof(line), PSTR("# %S,%lu\n"), label(stage_ - 2),
                                static_cast<unsigned long>(values_[stage_ - 2]));
        if (count <= 0 || count >= static_cast<int>(sizeof(line))) { stage_ = 255; return; }
        if (Serial.availableForWrite() < count) return;
        Serial.write(reinterpret_cast<const uint8_t*>(line), count);
        if (++stage_ == 26) stage_ = 255;
    }
private:
    static PGM_P label(uint8_t index) {
        switch (index) {
            case 0: return PSTR("log_status");
            case 1: return PSTR("queued_records");
            case 2: return PSTR("missed_log_slots");
            case 3: return PSTR("queue_full_records");
            case 4: return PSTR("oversized_records");
            case 5: return PSTR("peak_queue_bytes");
            case 6: return PSTR("queued_bytes");
            case 7: return PSTR("bytes_written");
            case 8: return PSTR("max_write_us");
            case 9: return PSTR("close_us");
            case 10: return PSTR("max_storage_service_us");
            case 11: return PSTR("max_capture_us");
            case 12: return PSTR("max_loop_gap_us");
            case 13: return PSTR("loops_over_budget");
            case 14: return PSTR("max_heartbeat_gap_us");
            case 15: return PSTR("max_command_gap_us");
            case 16: return PSTR("capacity_reached");
            case 17: return PSTR("storage_error");
            case 18: return PSTR("status_messages_dropped");
            case 19: return PSTR("init_us");
            case 20: return PSTR("loop_budget_us");
            case 21: return PSTR("motor_interval_ms");
            default: return PSTR("log_interval_ms");
        }
    }
    uint32_t values_[23] = {};
    char filename_[13] = {};
    uint8_t stage_ = 255;
};
