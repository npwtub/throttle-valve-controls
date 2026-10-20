> Updated October 20, 2026: use the sd-card-logging branch. The additional
> actuation_sd environment logs motor-only five-cycle actuations; see
> HANDOFF_2026-10-20.md for behavior and the review/test sequence. SD CS is
> configured as pin 9, and the CSV column is timestamp_unix_ms.

# SD logging update â€” start here

This is a complete updated source project. Open this folder in VS Code (the folder containing platformio.ini). Your original project was not overwritten. No firmware has been uploaded.

## What changed

- Shared SdCsvLogger remains in include/log.h, available to any test. Only dwell_csv_test.cpp opts in; other tests retain their existing logging behavior.
- Each run selects the first unused RUN0001.CSV through RUN9999.CSV. Existing files are never overwritten. One recording per Arduino reset is still supported.
- Before motion, the SD writer reserves 64 MiB of contiguous space. This avoids allocating more clusters during acquisition. It needs that much contiguous free space, not just total free space.
- Recording stops and drains at the reservation limit, on STOP, or through the existing travel-fault path. It never writes a partial record merely to fit the reservation. capacity_reached identifies a capacity stop. The automatic capacity stop does not change valve control.
- File closure truncates the unused reservation and syncs metadata. It can still block, and its duration is measured. A reset/power failure before clean closure can leave a 64 MiB file with an invalid trailing region; do not treat its apparent size as valid captured data.
- Diagnostic reports show losses, queue use, and worst observed timing. They are emitted in short lines only when serial transmit space is available.
- Flash-string state names no longer allocate an Arduino String on each capture. Misleading timing comments and existing constructor/unused-variable warnings were corrected.
- Both SPI devices are deselected before setup transactions; known CS conflicts are rejected.
- The console supports STATUS and saves received messages to a new valve_console_<unique number>.txt in the terminal's current folder. --transcript PATH selects another new file; existing files are not overwritten.

## What still needs your team

1. Confirm the SD module's actual chip-select wiring and electrical compatibility. In include/log_config.h, verify that SD_LOG_CS_PIN=9 matches the physical wiring; change it only to the confirmed pin. It must not conflict with CAN pin 10, Mega SPI pins 50/51/52, serial pins 0/1, pressure inputs A2/A3, or other hardware in use. The software cannot discover every wiring conflict. Pin 4 used during compilation was only a test placeholder, not a wiring recommendation.
2. Confirm the required control deadline. This update preserves the existing 20 ms (50 Hz) heartbeat/position-command schedule. loopBudgetUs=5000 is a diagnostic threshold based on the original 200 Hz task, not an implemented 200 Hz controller. Logging requests a record every 10 ms (100 Hz).
3. Confirm required test duration. At the maximum 256-byte record size and 100 Hz, 64 MiB holds about 43 minutes plus allowance for the header. Real rows may be shorter. Increase reservedBytes before building if needed, subject to card capacity and startup/closure timing.
4. Measure hardware behavior. Preallocation and buffering do NOT make the synchronous SD library nonblocking. Writes, card checks, and especially truncation/closure can delay CAN servicing. Initialization can pause heartbeats before the test starts. If measured worst-case timing exceeds the team's accepted deadline, this prototype does not meet the requirement; consider a separate logging processor or an architecture with demonstrated bounded storage operations.
5. Define the required UTC alignment accuracy. UTC values have millisecond units. One-way time transmission has transport delay and Arduino oscillator drift; exact millisecond agreement with CTS has not been established. Verify the PC clock and compare recorded events against CTS.

## Software checks you can run now

In the VS Code terminal at this project's root:

    python valve_log_console.py --help
    python -m unittest discover -s tests -v

In PlatformIO, build dwell_test (or use pio run to build all environments). No Arduino is needed for these checks. An invalid/unset SD pin deliberately prevents START on hardware; the current configured pin is 9.

## Hardware session (with the test operator)

Use a FAT16/FAT32-formatted card and confirmed wiring. Configure the SD pin and build/upload the dwell_test environment under the team's procedure. Close PlatformIO's serial monitor before using the console.

    python valve_log_console.py --list-ports
    python valve_log_console.py --port COMx

Replace COMx with the observed port. Opening a serial port can reset the Mega; connect before the test, not to an already running test.

- CONNECT opens the port, waits for firmware startup, and sends the PC's UTC time. It sends no start command.
- START explicitly starts the valve dwell test after the card is initialized and space reserved. This moves the valve. Initialization can take time; the console waits up to 120 seconds and never retries START automatically.
- STATUS requests a measurement snapshot. It does not change valve control or recording.
- STOP stops logging, drains the queue, and closes the file. It waits for LOG_CLOSED and the closed-state diagnostic report. It does not stop the valve test or command it closed. The existing dwell cycle and timeout behavior continue.
- QUIT is permitted before START or after logging has closed. That permission is not proof the valve test has stopped. Coordinate ending the test and disconnecting with the operator; serial disconnection/reconnection can reset the board.
- EXIT explicitly disconnects even when logging closure is unknown. It does not send a valve-close command.

Do not treat a response timeout as proof START failed. Check with the operator before retrying or resetting. If the card is missing/full/faulted or initialization fails, the console reports the error. Reset recovery is required after a failed logger; do not reset during an active test without the operator's procedure.

## Reading STATUS and the saved console transcript

Each block from STATS_BEGIN to STATS_END is one snapshot. Reports are automatic after start, failure, and closure and available on demand with STATUS. Values are cumulative since the test started unless noted.

- FILE: selected card filename.
- log_status: 0=not started, 1=recording, 2=stopping/draining, 3=closed, 4=failed.
- queued_records: complete data rows accepted into RAM. Not a durable-write count.
- missed_log_slots: scheduled 10 ms opportunities skipped because the program was late. The logger does not invent catch-up readings.
- queue_full_records / oversized_records: rejected whole records. Aim for zero in acceptance tests.
- peak_queue_bytes / queued_bytes: maximum/current occupancy of the 2,048-byte queue.
- bytes_written: bytes accepted by the SD file API, including header. Persistence still requires successful closure.
- max_write_us: longest file.write call.
- close_us: final truncate-and-close duration.
- max_storage_service_us: longest whole storage-service call, including finalization.
- max_capture_us: longest sensor-read/CSV formatting/queue operation.
- max_loop_gap_us / loops_over_budget: longest loop-entry interval and number exceeding loop_budget_us.
- max_heartbeat_gap_us / max_command_gap_us: longest interval between software send attempts. These do not confirm reception by the motor or successful CAN transmission. They include ongoing valve control after recording stops; request STATUS again when needed.
- capacity_reached: 1 means the recording stopped at its reserved size; valve control continued.
- storage_error: 0=none, 1=card initialization, 2=filename invalid/exhausted, 3=file open, 4=space reservation/sync, 5=write, 6=card busy timeout, 7=finalization. A write error leaves the file unconfirmed rather than repeatedly attempting blocking recovery during valve control.
- status_messages_dropped: status queue overflow count.
- init_us: initialization duration before motion; excluded from active loop timing.
- motor_interval_ms / log_interval_ms / loop_budget_us: configured schedules/diagnostic threshold.

The CSV has UTC epoch milliseconds, a sequence number, uptime milliseconds, pressures, cached motor telemetry/freshness fields, and test state. last_status_*_time remains uptime milliseconds. Timestamp spacing helps identify skipped samples; sequence gaps identify rejected capture attempts. 100 rows/second does not establish 100 fresh motor telemetry updates/second.

## Hardware acceptance evidence

Record the agreed deadlines first. Test at the intended duration and CAN load, comparing operation with and without SD logging. Check sample intervals, zero rejected/missed records, fresh telemetry, CAN errors, and measured loop/command/heartbeat gaps. Include STOP/closure timing, repeated runs with different filenames, and operator-approved card/error/reset scenarios. Confirm UTC alignment against CTS. Keep the CSV and console transcript together.

Compilation and mocked console tests are software checks only. No SD, CAN, motor, or timing behavior was physically validated here.
