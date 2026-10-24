#pragma once
#include <algorithm>
#include <vector>
#include "Arduino.h"

constexpr int SHARED_SPI = 0;
#define SD_SCK_MHZ(x) (x)
constexpr int O_WRONLY = 1, O_CREAT = 2, O_EXCL = 4;
struct SdSpiConfig { SdSpiConfig(int, int, int) {} };
struct FakeDisk {
    bool beginOk = true, openOk = true, allocateOk = true, syncOk = true;
    bool truncateOk = true, closeOk = true;
    int shortWrite = -1;
    uint32_t writeUs = 1000, allocated = 0, position = 0, trimmed = 0;
    unsigned writeCalls = 0, closeCalls = 0;
    std::vector<uint8_t> data;
};
static FakeDisk disk;
class SdFat32 {
public:
    bool begin(SdSpiConfig) { return disk.beginOk; }
    bool exists(const char*) { return false; }
};
class File32 {
public:
    bool open(const char*, int) { return disk.openOk; }
    bool preAllocate(uint32_t bytes) { disk.allocated = bytes; return disk.allocateOk; }
    bool seekSet(uint32_t position) { disk.position = position; return true; }
    size_t write(const void* source, size_t bytes) {
        ++disk.writeCalls;
        fakeUs += disk.writeUs;
        const size_t accepted = disk.shortWrite < 0 ? bytes :
            std::min(bytes, static_cast<size_t>(disk.shortWrite));
        const uint8_t* data = static_cast<const uint8_t*>(source);
        disk.data.insert(disk.data.end(), data, data + accepted);
        disk.position += accepted;
        return accepted;
    }
    bool sync() { return disk.syncOk; }
    bool truncate(uint32_t bytes) { disk.trimmed = bytes; return disk.truncateOk; }
    bool close() { ++disk.closeCalls; return disk.closeOk; }
};
