#pragma once

#include <stdint.h>

// Fixed-memory, interval histogram. Buckets 0..62 have inclusive upper bounds
// 127, 255, ... 8063 us. Bucket 63 contains all durations >= 8064 us.
// Percentiles use nearest rank; finite estimates are at most 127 us high.
struct ControlTimingTail {
    static constexpr uint8_t bucketCount = 64;
    static constexpr uint32_t bucketWidthUs = 128;
    uint32_t buckets[bucketCount] = {};
    uint32_t samples = 0;
    uint32_t previousP95ToP99 = 0;
    uint32_t previousAboveP99 = 0;
    uint8_t previousP95 = 0;
    uint8_t previousP99 = 0;
    bool baselineValid = false;

    void record(uint32_t durationUs) {
        const uint32_t index = durationUs / bucketWidthUs;
        const uint8_t bucket = index < bucketCount ? index : bucketCount - 1;
        ++buckets[bucket];
        ++samples;
        if (baselineValid) {
            if (bucket > previousP99) ++previousAboveP99;
            else if (bucket > previousP95) ++previousP95ToP99;
        }
    }

    uint8_t percentileBucket(uint8_t percent) const {
        // ceil(samples * percent / 100), without overflowing the product.
        const uint32_t rank = (samples / 100) * percent +
                              ((samples % 100) * percent + 99) / 100;
        uint32_t cumulative = 0;
        for (uint8_t bucket = 0; bucket < bucketCount; ++bucket) {
            cumulative += buckets[bucket];
            if (cumulative >= rank) return bucket;
        }
        return bucketCount - 1;
    }

    void startNextInterval(uint8_t p95, uint8_t p99) {
        previousP95 = p95;
        previousP99 = p99;
        // An overflow percentile has no finite threshold to compare against.
        baselineValid = samples != 0 && p99 < bucketCount - 1;
        for (uint8_t bucket = 0; bucket < bucketCount; ++bucket) buckets[bucket] = 0;
        samples = 0;
        previousP95ToP99 = 0;
        previousAboveP99 = 0;
    }
};
