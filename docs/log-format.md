# Binary log format, version 2

Each record is exactly 64 bytes. There is no file header or CSV text. Multi-byte
integers are unsigned little-endian. Float fields use 4-byte IEEE-754 binary32
encoded little-endian on the Mega target. Reserved bytes remain zero.

| Offset | Bytes | Type | Field / units |
| --- | --- | --- | --- |
| 0 | 1 | uint8 | Version: 2 |
| 1 | 1 | uint8 | State ID (below) |
| 2 | 1 | bits | Presence: bit 0 PT1, bit 1 PT2, bit 2 motor, bit 3 Unix time valid |
| 3 | 1 | bits | Motor flags: bit 0 status-0 received, bit 1 status-2 received, bit 2 telemetry available, bit 3 fresh |
| 4 | 8 | uint64 | Unix milliseconds; zero when time is unset |
| 12 | 4 | float32 | PT1 psi |
| 16 | 4 | float32 | PT2 psi |
| 20 | 4 | float32 | Measured valve degrees |
| 24 | 4 | float32 | Motor temperature, degrees C |
| 28 | 4 | float32 | Applied output, percent |
| 32 | 4 | float32 | Motor RPM |
| 36 | 4 | float32 | Voltage, V |
| 40 | 4 | float32 | Current, A |
| 44 | 4 | float32 | Commanded valve degrees; NaN if no command has been issued |
| 48 | 4 | uint32 | Last status-0 receive time, milliseconds since boot |
| 52 | 4 | uint32 | Last status-2 receive time, milliseconds since boot |
| 56 | 4 | uint32 | Buffer-full drop count before this record is queued |
| 60 | 4 | reserved | All zero |

Absent sensor fields are zero-filled in binary but decoded to blank CSV columns.
Presence does not imply fresh or valid feedback: inspect the motor flags too.
Commanded position is the requested target, not confirmation of CAN delivery or
physical position. Timing summaries are in the saved Serial transcript, not BIN.

State IDs: 0 waiting, 1 closed, 2 opening, 3 open, 4 closing, 5 motor froze,
6 initial_opening, 7 initial_open, 8 soaking, 9 cycle_opening, 10 cycle_open,
11 cycle_closing, 12 cycle_closed, 13 done, 14 timeout, 255 unknown.

Python layout: `struct.Struct("<4BQ9f3I4s")`. `tools/decode_log.py` validates the
whole input before creating a new CSV. It rejects unsupported versions, partial
records, invalid flags/reserved bytes, and invalid timestamps. Existing output
files are never overwritten. Earlier experimental version-1 files are not
supported by this converter because their state/time semantics differed.

Only orderly finalized files are supported. Files left after reset/power loss
may contain unwritten preallocated data. The decoder has no CRC or recovery
guarantee; do not interpret a successfully parsed unfinalized file as verified
data. Copy only after the firmware reports successful closure.
