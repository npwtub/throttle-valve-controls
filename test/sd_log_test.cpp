// Host simulation of the real SdLog class; no physical SD or Arduino required.
#include <assert.h>
#include <stdio.h>
#include "sd_log.h"

static void resetDisk() { disk = FakeDisk{}; fakeUs = 0; }
static LogRecord record(uint8_t value) {
    LogRecord result{};
    memset(result.bytes, value, sizeof(result.bytes));
    return result;
}

int main() {
    resetDisk();
    {
        SdLog log;
        assert(log.begin());
        for (int sample = 0; sample < 100; ++sample) {
            disk.writeUs = sample < 95 ? 1000 : (sample < 99 ? 3000 : 70000);
            for (int i = 0; i < 8; ++i) assert(log.enqueue(record(i)));
            assert(log.writeOneBlock());
        }
        assert(log.blockPercentileUpperUs(95) == 1023);
        assert(log.blockPercentileUpperUs(99) == 3071);
        assert(log.blockPercentileUpperUs(100) == 70000);
        assert(log.close());
    }
    resetDisk();
    {
        SdLog log;
        assert(log.blockWriteCount() == 0 && log.averageBlockUs() == 0.0);
        assert(log.blockPercentileUpperUs(95) == 0);
        assert(log.begin());
        for (int i = 0; i < 8; ++i) assert(log.enqueue(record(i)));
        assert(log.writeOneBlock()); // 1000 us.
        for (int i = 0; i < 9; ++i) assert(log.enqueue(record(i)));
        disk.writeUs = 3000;
        assert(log.close()); // One full block and one partial write.
        assert(log.blockWriteCount() == 2);
        assert(log.averageBlockUs() == 2000.0);
        assert(log.worstBlockUs() == 3000);
        assert(log.blockPercentileUpperUs(50) == 1023);
        assert(log.blockPercentileUpperUs(95) == 3071);
        assert(log.blockPercentileUpperUs(99) == 3071);
    }
    resetDisk();
    {
        SdLog log;
        assert(log.begin());
        assert(disk.allocated == Config::SD_PREALLOCATE_BYTES);
        for (uint8_t i = 0; i < 9; ++i) assert(log.enqueue(record(i)));
        log.writeUntil(2500); // Exactly the guard: no write allowed.
        assert(disk.writeCalls == 0);
        log.writeUntil(5000);
        assert(disk.writeCalls == 1 && log.buffered() == 1);
        assert(log.worstBlockUs() == 1000);
        fakeUs = 2000000;
        assert(log.service()); // Periodic sync must not write the partial block.
        assert(disk.writeCalls == 1);
        assert(log.close());
        assert(log.isFinalized() && disk.trimmed == 9 * 64);
        assert(disk.data.size() == 9 * 64);
        for (unsigned i = 0; i < disk.data.size(); ++i) assert(disk.data[i] == i / 64);
        assert(!log.enqueue(record(10)));
        assert(log.close() && disk.closeCalls == 1);
    }
    resetDisk();
    {
        SdLog log;
        assert(log.begin());
        for (int i = 0; i < 16; ++i) assert(log.enqueue(record(i)));
        assert(!log.enqueue(record(99)) && log.dropped() == 1);
        assert(log.peakBuffered() == 16);
        disk.writeUs = 3000;
        log.writeUntil(5000);
        assert(disk.writeCalls == 1 && log.buffered() == 8); // Recheck after write.
        assert(log.close());
    }
    resetDisk();
    {
        SdLog log;
        assert(log.begin());
        for (int i = 0; i < 8; ++i) log.enqueue(record(i));
        fakeUs = UINT32_MAX - 1000;
        log.writeUntil(fakeUs + 5000); // Deadline crosses micros rollover.
        assert(disk.writeCalls == 1);
        assert(log.close());
    }
    resetDisk();
    {
        SdLog log;
        assert(log.begin());
        for (int i = 0; i < 8; ++i) log.enqueue(record(i));
        log.writeUntil(0); // Deadline reached.
        assert(disk.writeCalls == 0);
        disk.shortWrite = 100;
        assert(!log.writeOneBlock() && log.hasError());
        assert(log.blockWriteCount() == 1 && log.averageBlockUs() == 1000.0);
        log.writeUntil(5000);
        assert(disk.writeCalls == 1); // No retry that could duplicate data.
        assert(!log.close());
        assert(disk.trimmed == 64 && !log.isFinalized());
    }
    resetDisk();
    {
        disk.allocateOk = false;
        SdLog log;
        assert(!log.begin() && log.hasError());
        assert(disk.closeCalls == 1);
    }
    resetDisk();
    {
        SdLog log;
        assert(log.begin());
        assert(log.enqueue(record(1)));
        disk.syncOk = false;
        assert(!log.close() && log.hasError() && !log.isFinalized());
    }
    resetDisk();
    {
        SdLog log;
        assert(!log.hasUnixTime() && log.unixTime() == 0);
        log.setUnixTime(1792540800000ULL);
        log.updateClock(1234);
        assert(log.unixTime() == 1792540801234ULL);
        log.updateClock(UINT32_MAX - 2);
        const uint64_t before = log.unixTime();
        log.updateClock(3);
        assert(log.unixTime() == before + 6);
    }
    puts("SD transport tests passed");
}
