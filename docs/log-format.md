# Binary log format, version 3

Each record is exactly 64 bytes. There is no file header or CSV text. Multi-byte
integers are unsigned little-endian. Float fields use 4-byte IEEE-754 binary32
encoded little-endian on the Mega target. All 64 bytes are now assigned.

| Offset | Bytes | Type | Field / units |
| --- | --- | --- | --- |
| 0 | 1 | uint8 | Version: 3 |
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
| 60 | 4 | float32 | Commanded thrust, lbf; defaults to zero when omitted |

Absent sensor fields are zero-filled in binary but decoded to blank CSV columns.
Presence does not imply fresh or valid feedback: inspect the motor flags too.
Commanded position is the requested target, not confirmation of CAN delivery or
physical position. Timing summaries are in the saved Serial transcript, not BIN.

Commanded thrust is the requested thrust setpoint, not measured or achieved
thrust. `log_row()` accepts it as an optional final argument after `finalRow`:

```cpp
log_row(&motorState, &pt1, &pt2, now, state_name(), targetMotorRotations,
        finalRow, commandedThrustLbf);
```

Existing calls can omit both optional arguments, or supply only `finalRow`,
and record zero thrust. Zero therefore means either no thrust setpoint was
provided or an explicit zero-thrust request; there is no thrust-presence flag.
The feedforward test records the profile setpoint used for the current model
calculation and zero during closing or other cycles without a calculation.
A calculation that fails can still record its attempted thrust request;
inspect the state and commanded valve angle to see the resulting close command.

State IDs: 0 waiting, 1 closed, 2 opening, 3 open, 4 closing, 5 motor froze,
6 initial_opening, 7 initial_open, 8 soaking, 9 cycle_opening, 10 cycle_open,
11 cycle_closing, 12 cycle_closed, 13 done, 14 timeout, 15 feedforward,
16 failed, 17 moving_up, 18 moving_down, 19 at_setpoint_up,
20 at_setpoint_down, 21 finished, 22 travel_timeout, 255 unknown.
IDs 0-16 retain their existing meanings; IDs 17-22 preserve the CdA
characterization sweep and terminal state names.

Python layout: `struct.Struct("<4BQ9f3If")`. The decoder exports commanded
thrust as `commanded_thrust_lbf`, followed by two derived CSV columns:

| CSV column | Units | Calculation |
| --- | --- | --- |
| `commanded_mdot_lbm_per_sec` | lbm/s | Interpolate `Engine::kLoxMdotKg` on `Engine::kThrustLbf`, then multiply by `Feedforward::kPoundsMassPerKilogram` |
| `commanded_dp_psi` | psi | Interpolate `Engine::kValveDpPsi` using the same thrust interval and fraction |

These are nominal model targets reconstructed from requested thrust, not
measured flow or pressure drop. `commanded_dp_psi` is the tabulated drop across
the valve, not its downstream-pressure target or measured `PT1 - PT2`.
Both use the interpolation order in `Feedforward::set_thrust_to_mdot()`, with
constants and each arithmetic operation rounded to binary32 to match the Mega.
The decoder does not repeat the firmware's live-PT1 validation, since the logger
records a later PT1 sample; derived targets do not confirm the request succeeded.

Zero thrust produces zero in both derived columns, including tests that omit
thrust and use the zero default. Missing thrust (version 2), negative/nonfinite
thrust, or positive thrust outside the engine table leaves both columns blank;
the decoder does not extrapolate. All earlier CSV columns retain their order.

The decoder reads the numeric lookup arrays from `include/config.h` and the
mass conversion constant from `include/feedforward.h`, once per conversion
process. Use headers matching the firmware used for that test. For archived
headers, select their directory with:

```powershell
python tools/decode_log.py outputs/LOG0000.BIN outputs/LOG0000_derived.csv --model-dir path/to/saved/include
```

The saved directory must contain `config.h` and `feedforward.h`. The decoder
requires numeric literals for the three engine arrays and conversion constant;
missing or malformed model declarations stop conversion before a CSV is created.

`tools/decode_log.py` supports both version 3 and older version 2 logs. Version 2
uses `struct.Struct("<4BQ9f3I4s")`, with bytes 60-63 reserved and required to be
zero. Its `commanded_thrust_lbf` CSV column is blank because thrust was not
recorded. Version 3 uses those bytes for thrust; offsets 1-59 and the record size
are unchanged. Older decoders that only support version 2 cannot read new logs.

The decoder validates the whole input before creating a new CSV. It rejects
unsupported versions, partial records, invalid flags, nonzero version-2 reserved
bytes, and invalid timestamps. Existing output files are never overwritten.
Earlier experimental version-1 files are not supported by this converter
because their state/time semantics differed.

Only orderly finalized files are supported. Files left after reset/power loss
may contain unwritten preallocated data. The decoder has no CRC or recovery
guarantee; do not interpret a successfully parsed unfinalized file as verified
data. Copy only after the firmware reports successful closure.
