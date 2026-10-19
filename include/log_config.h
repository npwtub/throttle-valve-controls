#pragma once
#include <Arduino.h>
#ifndef SD_LOG_CS_PIN
#define SD_LOG_CS_PIN 9
#endif
namespace LogConfig {
// Confirm the physical wiring; never share CAN's chip select.
constexpr int chipSelectPin = SD_LOG_CS_PIN;
constexpr unsigned long intervalMs = 10;
// Reserve before motion. Stop logging at this size; valve control continues.
constexpr uint32_t reservedBytes = 64UL * 1024UL * 1024UL;
// Diagnostic threshold, NOT a change to the existing 20 ms motor schedule.
// Confirm the original 200 Hz control requirement with the lead.
constexpr uint32_t loopBudgetUs = 5000;
constexpr uint32_t busyTimeoutMs = 5000;
}
