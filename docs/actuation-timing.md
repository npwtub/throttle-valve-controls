# Actuation timing measurements

Serial timing reports have been removed from actuation. The counters and
histogram still update in RAM, but are not saved in the current SD record format.
The field names below describe the former report and the underlying measurements.

The actuation test now also uses SD logging and a fixed 5 ms schedule. See
[bench-test.md](bench-test.md) for the SD diagnostics and hardware test procedure.
`cycle_overruns` includes SD work and final file closure; `missed_slots` counts
skipped scheduled starts. `cmd_spacing_worst_us` measures gaps between position
submission calls. These are host-side measurements, not CAN on-wire timing.

The timing histogram uses one-second windows. It measures the same duration
as the former `avg_us` and `worst_us` fields:
CAN reception, control calculations, and both CAN submission calls. They do not
measure actual transmission time on the bus or include the subsequent logging
and wait. Existing timing fields remain cumulative; the new fields cover only
the samples since the previous report.

| Field | Meaning |
| --- | --- |
| `window_n` | Number of measured cycles in this reporting interval. |
| `p95_le_us`, `p99_le_us` | Inclusive upper bounds of the buckets containing this interval's nearest-rank 95th and 99th percentiles. |
| `prev_tail_valid` | Whether the previous report supplied usable finite percentile thresholds. |
| `prev_p95_to_p99` | This interval's cycles strictly above the previous p95 bucket's upper bound and at or below the previous p99 bucket's upper bound. |
| `prev_above_p99` | This interval's cycles strictly above the previous p99 bucket's upper bound. |

For example, `window_n=200 prev_tail_valid=1 prev_p95_to_p99=12
prev_above_p99=4` means 6% of this interval's cycles were in the previous
percentile band, and 2% exceeded its upper threshold. Divide either count by
`window_n` to get its frequency. Previous thresholds are used because measuring
against the same samples that define the percentiles would normally yield about
4% in the band and 1% above it, even as latency gets worse.

The first report has `prev_tail_valid=0` and zero comparison counts. Each report
sets the thresholds for the next interval, so this measures short-term changes;
the percentile durations themselves show sustained increases. At the nominal
200 Hz cycle rate there are approximately 200 samples per interval, so p99 is
sensitive to a few individual samples.

The histogram has 64 counters of four bytes each (256 bytes), plus 15 bytes of
state on the AVR target. It uses no allocation, sorting, stored sample history,
or floating-point calculations. Recording a sample uses a constant-time bucket
lookup and integer counters. Percentile scans and clearing occur once per
report, after both CAN frames have been submitted.

Finite buckets are 128 microseconds wide: percentile upper bounds can overstate
the corresponding exact sample by up to 127 microseconds. Values in the same
bucket cannot be distinguished; if p95 and p99 share a bucket, the band count
is zero. The last bucket contains all durations >= 8064 microseconds and prints
as `>8063`. If p99 lands there, the next interval's comparison counts are invalid
and zero (`prev_tail_valid=0`). The existing `worst_us`, `late_5ms`, and
`cycle_overruns` fields continue to capture severe delays.
