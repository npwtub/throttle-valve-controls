# Bench test: binary SD logging

This procedure targets `sd_logging_test` first, then `actuation_test`. The dwell
programs build with the shared logger but retain their earlier rates and do not
perform the Unix-time handshake. Use the two environments below for today's work.

## What the code does

1. `SdLog::begin()` opens the next unused `LOG0000.BIN`-style filename, reserves
   16 MiB, and starts at byte zero. It uses FAT16/FAT32 and shared SPI on CS D4.
2. `waitForUnixTime()` announces readiness and receives Unix milliseconds from
   the computer. The control schedule starts only after `TIME_SET`.
3. `logRow()` packs version-2, 64-byte records into a 16-record RAM ring buffer.
4. `finishControlCycle()` allows complete 512-byte writes while more than
   2,500 us remain: a 2,000 us estimate plus 500 us margin. It rechecks after
   every write. Expired control slots are counted and skipped.
5. On the final row, the logger drains the queue, synchronizes, trims unused
   reserved space, and closes once. This final operation is intentionally
   blocking. The actuation controller resumes closed commands and heartbeats.

The 16 MiB reservation covers about 21.8 minutes at 200 records/s, including
waiting time after synchronization. Reaching that limit latches an SD error;
the file does not silently grow beyond the reservation. Adjust
`SD_PREALLOCATE_BYTES` for longer tests before compiling.

Normal actuation writes remain aligned to 512-byte file positions. Only the
final flush may be smaller. There is no periodic explicit sync in the scheduled
actuation path: orderly finalization is required before card removal. Reset or
power loss can leave a reserved/unfinalized file, including unwritten data.

## Wiring and card

Use the Adafruit #254 breakout and a FAT16/FAT32-formatted card with at least
16 MiB of free contiguous space. This firmware uses `SdFat32`, not exFAT.

| Breakout | Mega 2560 |
| --- | --- |
| 5V | 5V |
| GND | GND |
| CLK | D52 (SCK) |
| DO | D50 (MISO) |
| DI | D51 (MOSI) |
| CS | D4 |

CAN keeps CS D10 and shares clock/data lines. Do not use the Uno's D11-D13 SPI
pin numbers for the Mega. Do not connect the breakout's 3V output to 5V.

## Commands on this Windows computer

Open PowerShell in the project directory. Set these convenient executable paths:

```powershell
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"
$py = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
& $py tools/run_test.py --list
```

PlatformIO's Python already has pyserial on this machine. For another Python
installation, install `tools/requirements.txt` with that interpreter's pip.
Replace `COM5` below with the port printed by `--list`.

### 1. SD-only check (no motor commands)

```powershell
& $pio run -e sd_logging_test -t upload --upload-port COM5
& $py tools/run_test.py --port COM5
```

Close other serial monitors first. Opening the port normally resets the Mega.
The script waits for `READY_FOR_TIME`, sends the current computer timestamp,
and waits for `TIME_SET`. This keeps one connection open; switching to another
monitor could reset the board and restart setup. If no readiness arrives, check
the port and press reset while the script is waiting.

For the SD-only firmware, **do not type start**: it automatically captures 10,000
records (1,250 blocks) after time synchronization, then closes. Allow about
50 seconds plus overruns and finalization. Adjust `sampleLimit` in
`src/sd_logging_test.cpp` for another sample size. Expected final diagnostics:

```text
bytes=640000 ... block_writes=1250 ... sd_error=0 sd_closed=1
```

The test has no sensor objects and transmits no CAN or motor commands. It logs
a fixed commanded position of approximately 90 valve degrees, with sensor
presence flags clear. It is a storage/decoding check, not a CAN timing test.

After `sd_closed=1`, type `quit`, power down the board, and move the card to the
computer. Copy the named BIN file into `outputs/`, then decode it:

```powershell
& $py tools/decode_log.py outputs/LOG0000.BIN
```

Expected: 10,000 CSV rows, increasing Unix milliseconds, first state `waiting`,
last state `done`, absent sensor columns blank, commanded position about 90,
and dropped records zero. Approximately 5 ms timestamp spacing is expected;
overruns may increase spacing. The decoder never overwrites an existing CSV.

### 2. Actuation with CAN and SD

Reinstall the card while powered down, connect the existing CAN/motor test
hardware, then:

```powershell
& $pio run -e actuation_test -t upload --upload-port COM5
& $py tools/run_test.py --port COM5
```

After `TIME_SET`, type `start` and press Enter when ready to actuate. This sends
the existing single-space start command. The script does not send it
automatically. It saves all received lines under `outputs/serial_*.txt`.

Actuation logs during WAITING too. It commands closed after synchronization,
then performs the existing five-cycle test after the start command. Its original
one-second travel timeout is unchanged. Pressure fields are absent in this test.

Actuation prints one `RUN:` line followed by an `SD:` status line after DONE or
TIMEOUT. Check `sd_error=0 sd_closed=1` for successful finalization. No periodic
reports are printed. Finalization and the one-time Serial report may
cause an overrun, as explicitly accepted for shutdown. The board keeps sending
closed-position commands and heartbeats afterward. `quit` or Ctrl+C only exits
the monitor; it is not a motor stop command. Power down before removing the card.

Copy/decode this run's filename as above. Confirm the last CSV state is `done`
or `timeout`, not just `waiting` or `closing`. A timeout file is not evidence of
successful valve closure. `is_fresh` and status flags help identify missing or
stale CAN feedback.

## Read the measurements

`SD:` also prints `block_p95_upper_us` and `block_p99_upper_us`: nearest-rank
percentile upper estimates using 512 us histogram bins. They cover the same
full-block attempts as the average (including final drain and failed attempts).
For values below 65,024 us, rounding adds at most 511 us. If a percentile falls
in the overflow bin, the observed maximum is reported as a conservative bound.
Zero block count means no percentile samples.

Actuation prints an additional `OVER_5MS:` line for active non-final cycles
whose execution exceeded 5 ms. Categories are mutually exclusive:
`control` means reception/calculation/CAN submission already exceeded 5 ms;
`pre_sd_logging` means the threshold was crossed afterward but before SD service;
`service_with_write` means it crossed during service in a cycle with a block
attempt; `service_without_write` means it crossed during service without one.
These locate threshold crossings, not exclusive causes: control work and SD
work share the budget. These counts differ from skipped schedule slots and
exclude WAITING, the final cycle, and Serial reporting.

The actuation `RUN:` report measures from the cycle accepting `start` through
the cycle reaching DONE or TIMEOUT. `control_hz` is `(cycles - 1) / elapsed seconds`
using cycle-start timestamps; it is loop frequency, not confirmed CAN delivery.
`spacing_worst_us` and `missed_slots` on this line exclude WAITING and final
SD closure/reporting. `dropped_records` counts queue rejections during the run,
not dropped files or samples never collected during a stall. CAN submission
failures appear as `control_errors` and `heartbeat_errors`.
`write_allowance_us=2500` is the unchanged 2000 us estimate plus 500 us margin.

Both tests print the `SD:` status. Its measurements cover the whole logging
session, including WAITING and finalization. The table also describes existing
RAM-only counters; `worst_us`, `cmd_spacing_worst_us`, and `cycle_overruns` are
not printed. Timing measurements are not stored in the binary records.

| Field | Meaning |
| --- | --- |
| `worst_us` | Largest execution time through CAN reception, calculations, and both submission calls; excludes SD logging. |
| `cmd_spacing_worst_us` | Largest gap between starts of position submission calls, not on-wire timing. |
| `spacing_worst_us` | Largest gap between actual control-cycle starts. |
| `cycle_overruns` | Cycles whose work extends past the following scheduled start. Includes finalization. |
| `missed_slots` | Scheduled starts skipped because work ran late. |
| `buffered` / `peak` | Current and maximum queued records; capacity is 16. |
| `dropped` | Records rejected because the queue was full, not all possible I/O losses. |
| `bytes` | Bytes accepted by file writes; only a successful close confirms finalization. |
| `block_worst_us` | Largest measured 512-byte operation, including copying records and the write call. Includes final-drain full blocks. |
| `block_avg_us` / `block_writes` | Mean duration in microseconds and count of attempted 512-byte operations, with the same scope as `block_worst_us`. Includes failed attempts and final-drain full blocks; excludes partial writes and sync/close work. Zero count means no measurements. Printed by both tests. |
| `close_us` | Time in close(), including its remaining flush, sync, truncate, and close. A preceding final-row room-making block is separate. |
| `sd_error` | Latched storage/init/capacity/write/sync/close failure. |
| `sd_closed` | Successful finalization. Do not treat an SD error as a successful close. |

Use the `RUN:` fields for active-test timing and the `SD:` fields for overall
storage results. The runner saves both lines in its Serial transcript.

Start with a short run. Check that records are valid, drops and CAN errors are
zero, and the buffer peak is not trending toward capacity. Compare the largest
observed block time with `SD_WRITE_ESTIMATE_US`; choose an estimate above measured
values plus margin, then repeat. If no blocks fit in the remaining budget, the
queue will fill: increasing the threshold alone does not solve that condition.

At 200 Hz the data rate is 12,800 bytes/s; 16 records buffer only 80 ms of samples.
Observed timings are not a proof of a hard bound on blocking SD latency. This
bench implementation counts overruns but does not guarantee none can occur.

## Desktop verification (no board)

```powershell
& $py -m unittest discover -s test -p 'test_*.py' -v
$env:Path = "C:\msys64\ucrt64\bin;$env:Path"
& $pio run -e ring_buffer_test -t exec
g++ -std=c++11 -Wall -Wextra -Itest/fakes -Iinclude test/sd_log_test.cpp -o .pio/build/sd_log_host_test.exe
& .\.pio\build\sd_log_host_test.exe
```

The SD transport test uses the real SdLog code with fake hardware to test block
ordering, time-budget rechecks, rollover, full queues, short writes, preallocation
failure, sync failure, and final trimming. These do not substitute for card tests.

## Current scope

Today targets 200 Hz capture as requested (one record per control cycle), not
the earlier 100 Hz alternative. Unix time is approximate: it inherits the host
clock, serial delay, and Arduino oscillator drift. The firmware still uses the
original actuation state machine and position checks. It does not validate the
mechanical system or change the motor's configured control behavior.
