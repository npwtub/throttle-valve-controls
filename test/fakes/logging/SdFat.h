#pragma once
#include <stddef.h>
#include <stdint.h>
#include <vector>

constexpr int SHARED_SPI = 0;
#define SD_SCK_MHZ(value) (value)
constexpr int O_WRONLY = 1, O_CREAT = 2, O_EXCL = 4;
struct SdSpiConfig { SdSpiConfig(int, int, int) {} };
struct LoggingTestDisk {
    std::vector<uint8_t> data;
    unsigned writeCalls = 0;
};
static LoggingTestDisk loggingTestDisk;
class SdFat32 {
public:
    bool begin(SdSpiConfig) { return true; }
    bool exists(const char*) { return false; }
};
class File32 {
public:
    bool open(const char*, int) { return true; }
    bool preAllocate(uint32_t) { return true; }
    bool seekSet(uint32_t) { return true; }
    size_t write(const void* source, size_t bytes) {
        const uint8_t* data = static_cast<const uint8_t*>(source);
        loggingTestDisk.data.insert(loggingTestDisk.data.end(), data, data + bytes);
        ++loggingTestDisk.writeCalls;
        return bytes;
    }
    bool sync() { return true; }
    bool truncate(uint32_t) { return true; }
    bool close() { return true; }
};
