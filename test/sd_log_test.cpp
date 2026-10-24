// Host simulation of the real SdLog class; no physical SD or Arduino required.
#include <assert.h>
#include <stdio.h>
#include "sd_log.h"

static void reset_disk() { disk = FakeDisk{}; fakeUs = 0; }
static LogRecord record(uint8_t value) {
    LogRecord result{};
    memset(result.bytes, value, sizeof(result.bytes));
    return result;
}

int main() {
    reset_disk();
    {
        SdLog log;
        assert(log.begin());
        for (int sample = 0; sample < 100; ++sample) {
            disk.writeUs = sample < 95 ? 1000 : (sample < 99 ? 3000 : 70000);
            for (int i = 0; i < 8; ++i) assert(log.enqueue(record(i)));
            assert(log.write_one_block());
        }
        assert(log.block_percentile_upper_us(95) == 1023);
        assert(log.block_percentile_upper_us(99) == 3071);
        assert(log.block_percentile_upper_us(100) == 70000);
        assert(log.close());
    }
    reset_disk();
    {
        SdLog log;
        assert(log.block_write_count() == 0 && log.average_block_us() == 0.0);
        assert(log.block_percentile_upper_us(95) == 0);
        assert(log.begin());
        for (int i = 0; i < 8; ++i) assert(log.enqueue(record(i)));
        assert(log.write_one_block()); // 1000 us.
        for (int i = 0; i < 9; ++i) assert(log.enqueue(record(i)));
        disk.writeUs = 3000;
        assert(log.close()); // One full block and one partial write.
        assert(log.block_write_count() == 2);
        assert(log.average_block_us() == 2000.0);
        assert(log.worst_block_us() == 3000);
        assert(log.block_percentile_upper_us(50) == 1023);
        assert(log.block_percentile_upper_us(95) == 3071);
        assert(log.block_percentile_upper_us(99) == 3071);
    }
    reset_disk();
    {
        SdLog log;
        assert(log.begin());
        assert(disk.allocated == Config::kSdPreallocateBytes);
        for (uint8_t i = 0; i < 9; ++i) assert(log.enqueue(record(i)));
        log.write_until(2500); // Exactly the guard: no write allowed.
        assert(disk.writeCalls == 0);
        log.write_until(5000);
        assert(disk.writeCalls == 1 && log.buffered() == 1);
        assert(log.worst_block_us() == 1000);
        fakeUs = 2000000;
        assert(log.service()); // Periodic sync must not write the partial block.
        assert(disk.writeCalls == 1);
        assert(log.close());
        assert(log.is_finalized() && disk.trimmed == 9 * 64);
        assert(disk.data.size() == 9 * 64);
        for (unsigned i = 0; i < disk.data.size(); ++i) assert(disk.data[i] == i / 64);
        assert(!log.enqueue(record(10)));
        assert(log.close() && disk.closeCalls == 1);
    }
    reset_disk();
    {
        SdLog log;
        assert(log.begin());
        for (int i = 0; i < 16; ++i) assert(log.enqueue(record(i)));
        assert(!log.enqueue(record(99)) && log.dropped() == 1);
        assert(log.peak_buffered() == 16);
        disk.writeUs = 3000;
        log.write_until(5000);
        assert(disk.writeCalls == 1 && log.buffered() == 8); // Recheck after write.
        assert(log.close());
    }
    reset_disk();
    {
        SdLog log;
        assert(log.begin());
        for (int i = 0; i < 8; ++i) log.enqueue(record(i));
        fakeUs = UINT32_MAX - 1000;
        log.write_until(fakeUs + 5000); // Deadline crosses micros rollover.
        assert(disk.writeCalls == 1);
        assert(log.close());
    }
    reset_disk();
    {
        SdLog log;
        assert(log.begin());
        for (int i = 0; i < 8; ++i) log.enqueue(record(i));
        log.write_until(0); // Deadline reached.
        assert(disk.writeCalls == 0);
        disk.shortWrite = 100;
        assert(!log.write_one_block() && log.has_error());
        assert(log.block_write_count() == 1 && log.average_block_us() == 1000.0);
        log.write_until(5000);
        assert(disk.writeCalls == 1); // No retry that could duplicate data.
        assert(!log.close());
        assert(disk.trimmed == 64 && !log.is_finalized());
    }
    reset_disk();
    {
        disk.allocateOk = false;
        SdLog log;
        assert(!log.begin() && log.has_error());
        assert(disk.closeCalls == 1);
    }
    reset_disk();
    {
        SdLog log;
        assert(log.begin());
        assert(log.enqueue(record(1)));
        disk.syncOk = false;
        assert(!log.close() && log.has_error() && !log.is_finalized());
    }
    reset_disk();
    {
        SdLog log;
        assert(!log.has_unix_time() && log.unix_time() == 0);
        log.set_unix_time(1792540800000ULL);
        log.update_clock(1234);
        assert(log.unix_time() == 1792540801234ULL);
        log.update_clock(UINT32_MAX - 2);
        const uint64_t before = log.unix_time();
        log.update_clock(3);
        assert(log.unix_time() == before + 6);
    }
    puts("SD transport tests passed");
}
