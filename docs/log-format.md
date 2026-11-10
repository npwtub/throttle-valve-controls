# Binary log format, version 4

Each record is exactly 64 bytes. There is no file header or CSV text. Multi-byte
integers are little-endian. Float fields use 4-byte IEEE-754 binary32 encoded
little-endian on the Mega target. Byte 31 remains free and must be zero.
The Python layout is `struct.Struct("<4BQ4fbbBB3fI4f")`.

## Fields stored in each binary record

| Offset | Bytes | Type | Field / units |
| --- | --- | --- | --- |
| 0 | 1 | uint8 | Version: 4 |
| 1 | 1 | uint8 | State ID (below) |
| 2 | 1 | bits | Presence / command flags: bit 0 PT1, bit 1 PT2, bit 2 motor, bit 3 Unix time valid, bit 4 backlash compensation applied, bit 5 PT3, bit 6 valid PID snapshot, bit 7 combined valve command clipped |
| 3 | 1 | bits | Motor flags: bit 0 status-0 received, bit 1 status-2 received, bit 2 telemetry available, bit 3 fresh |
| 4 | 8 | uint64 | Unix milliseconds; zero when time is unset |
| 12 | 4 | float32 | PT1 psi |
| 16 | 4 | float32 | PT2 psi |
| 20 | 4 | float32 | PT3 psi; measured chamber pressure |
| 24 | 4 | float32 | Measured motor angle in valve-side degrees: motor rotations * 360 / gear ratio, including backlash travel; CSV `measured_motor_angle_deg` |
| 28 | 1 | int8 | Motor temperature, whole degrees C; -127 to 127, -128 means invalid |
| 29 | 1 | int8 | Applied output, whole percent; -100 to 100, -128 means invalid |
| 30 | 1 | uint8 | Current, whole A; 0 to 254, 255 means invalid |
| 31 | 1 | uint8 | Free / reserved; must be zero |
| 32 | 4 | float32 | Motor RPM |
| 36 | 4 | float32 | Voltage, V |
| 40 | 4 | float32 | Commanded valve angle in degrees before backlash compensation; CSV `commanded_valve_angle_deg`; NaN if no command has been issued |
| 44 | 4 | uint32 | Buffer-full drop count before this record is queued |
| 48 | 4 | float32 | Commanded thrust, lbf; defaults to zero when omitted |
| 52 | 4 | float32 | PID P output, valve-side degrees; CSV `pid_p_output_deg` |
| 56 | 4 | float32 | PID I output after its integral bounds, valve-side degrees; CSV `pid_i_output_deg` |
| 60 | 4 | float32 | PID D output, valve-side degrees; CSV `pid_d_output_deg` |

The motor temperature, output, and current values occupy one four-byte group,
including its free byte. Values are rounded to the nearest whole unit, with
half-units rounded away from zero, and saturated to the stated ranges without
integer wraparound. Nonfinite values use the invalid codes above and decode to
blank CSV cells. Output remains signed so motor direction is preserved.

Removing the two receive timestamps saves eight bytes; replacing three float32
motor values with the four-byte group saves another eight bytes. Those 16 bytes
hold PT3 and the three PID contributions. The removed timestamps were status
**0** and status **2** in this firmware; there was no logged status-1 timestamp.
Status-received and freshness flags remain in byte 3, and RPM remains logged.

Absent sensor fields are zero-filled in binary but decoded to blank CSV columns.
Presence does not imply fresh or valid feedback: inspect the motor flags too.
Commanded position is the requested target, not confirmation of CAN delivery or
physical position. Timing summaries are in the saved Serial transcript, not BIN.

Byte 2 bit 4 is exported directly as `backlash_compensated` (0 or 1). It means
the fixed `Config::kBacklashCompensation` offset was added to that cycle's
requested motor position. It does not confirm the command reached the motor
or that the mechanical backlash has been taken up. Unassigned bits in byte 3
must be zero. All eight bits of byte 2 are assigned in version 4.

Byte 2 bit 6 marks cached PID terms from a valid update; otherwise all three
PID CSV cells are blank. Bit 7 is exported as `pid_output_clamped` when a PID
snapshot exists. It records actual clipping of the combined feedforward plus
PID target to the valve travel limits, before backlash compensation. Merely
equaling a limit without clipping does not set this bit.

## Firmware logging arguments

`log_row()` takes the desired valve target expressed in **motor rotations
before backlash compensation**. Pass the same offset decision used for the
motor command; do not convert the target to degrees first or pass an already
compensated motor command. The logger converts the target to degrees for
offset 40. Closed-loop targets include feedforward and PID correction before
the separate backlash offset.

Commanded thrust is the requested thrust setpoint, not measured or achieved
thrust. PT3 follows PT2 in the required sensor arguments. The five optional
arguments, in order, are `finalRow`, `backlashCompensated`,
`commandedThrustLbf`, `pidController`, and `controlData`:

```cpp
LogControlData controlData;
controlData.pt3PressurePsi = &measuredChamberPressurePsi;
controlData.pidOutputClamped = pidOutputClamped;
log_row(&motorState, &pt1, &pt2, &pt3, now, state_name(),
        targetMotorRotations, finalRow, backlashCompensated,
        commandedThrustLbf, &chamberPressurePid, &controlData);
```

Calls can omit all five optional arguments, or supply only `finalRow`,
to record no compensation and zero thrust. To supply thrust without
compensation, pass `false` for `backlashCompensated` explicitly.
Zero thrust therefore means either no thrust setpoint was
provided or an explicit zero-thrust request; there is no thrust-presence flag.
The feedforward test records the profile setpoint used for the current model
calculation and zero during closing or other cycles without a calculation.
A calculation that fails can still record its attempted thrust request;
inspect the state and commanded valve angle to see the resulting close command.

Pass null sensor pointers for absent PTs, and omit or pass null for the PID
pointer in tests without PID or in cycles where it was not updated. The
closed-loop test passes a PID pointer only for the current cycle's update, so
closing and failure rows cannot repeat stale terms from the last running cycle.

`PidController::update()` caches its P/I/D contributions and final output in
motor rotations. The const getters `last_p_output()`, `last_i_output()`,
`last_d_output()`, and `last_output()` return those cached values; the logger
converts P/I/D to valve-side degrees. Reading them does not rerun the PID or
advance I/D history. `has_last_output()` indicates a valid snapshot. Reset
invalidates the snapshot. Invalid inputs or nonfinite output invalidate it too;
a skipped update for invalid `dt` has the zero terms/output returned by the
controller, while the internal accumulated I remains held for a later update.

PT3 uses A4, a 1000 psi range, and the same voltage calibration as PT2, as set
in `Config`. In the closed-loop test PT3 supplies real chamber-pressure feedback;
PT2 remains the valve-outlet pressure sensor. `controlData.pt3PressurePsi` can
point to the exact float sample already supplied to PID. The logger copies it
instead of taking another ADC sample. If this pointer is omitted, a nonnull
PT3 sensor is sampled normally. Keep the sample alive through `log_row()`.

## State IDs

State IDs: 0 waiting, 1 closed, 2 opening, 3 open, 4 closing, 5 motor froze,
6 initial_opening, 7 initial_open, 8 soaking, 9 cycle_opening, 10 cycle_open,
11 cycle_closing, 12 cycle_closed, 13 done, 14 timeout, 15 feedforward,
16 failed, 17 moving_up, 18 moving_down, 19 at_setpoint_up,
20 at_setpoint_down, 21 finished, 22 travel_timeout, 255 unknown.
IDs 0-16 retain their existing meanings; IDs 17-22 preserve the CdA
characterization sweep and terminal state names.

## Four CSV angle fields

The CSV contains four motor/valve position fields, all in **valve-side degrees**.
Motor angles are normalized by the gear ratio so they can be compared directly
with valve angles; they are not motor-shaft degrees or numbers of rotations.
Motor angles include backlash compensation, while valve angles exclude it.

The binary log stores the measured motor angle at offset 24 and the commanded
valve angle at offset 40. The decoder reads these directly, subject to the
availability and finite-value checks below. It reconstructs the other two
angles using the recorded backlash flag.

| CSV column | Units | Meaning |
| --- | --- | --- |
| `measured_motor_angle_deg` | degrees | Logged encoder angle, including backlash travel |
| `commanded_motor_angle_deg` | degrees | Requested motor angle, including the applied backlash offset |
| `measured_valve_angle_deg` | degrees | Estimated valve angle after removing the applied backlash offset from motor feedback |
| `commanded_valve_angle_deg` | degrees | Logged valve target before adding backlash compensation |

With the current gear ratio of 32.0017, a compensated command has an offset
of about 6.75 degrees. A 90-degree commanded valve angle therefore
produces a commanded motor angle of about 96.75 degrees when compensated.
When the measured motor angle reaches 96.75 degrees, the estimated measured
valve angle is about 90 degrees.

There is no independent valve-position measurement in this log. The estimate
assumes the motor feedback corresponds to the current command's backlash
offset; it cannot resolve transient slack, partial take-up, or a direction
change between the last feedback sample and the command. Angle values also
inherit rounding from the stored binary32 angle conversion.
The command fields do not verify CAN delivery or achieved position.

Nonfinite commanded position leaves both commanded-angle columns blank.
Measured angles are blank when the motor is absent, no status-2
position frame has been received, or position is nonfinite. Stale finite
feedback remains available; inspect `has_status_2` and `is_fresh`. Receive
timestamps are no longer in version-4 records or CSV exports.

## CSV fields calculated from logged fields

Let `M` be the measured motor angle stored at offset 24, `T` be the commanded
valve angle stored at offset 40, and `G` be `Config::kGearRatio`. The applied
offset in degrees is `D = Config::kBacklashCompensation * 360 / G` when
`backlash_compensated` is 1, otherwise zero. The compensation constant is in
motor rotations internally; no rotation-valued angle columns are exported.

The decoder calculates these fields without adding bytes to the binary record:

| CSV column | Units | Calculation |
| --- | --- | --- |
| `commanded_motor_angle_deg` | degrees | `T + D` |
| `measured_valve_angle_deg` | degrees | `M - D` |
| `commanded_mdot_lbm_per_sec` | lbm/s | Interpolate `Engine::kLoxMdotKg` on `Engine::kThrustLbf`, then multiply by `Feedforward::kPoundsMassPerKilogram` |
| `commanded_dp_psi` | psi | Interpolate `Engine::kValveDpPsi` using the same thrust interval and fraction |
| `commanded_chamber_pressure_psi` | psi | Interpolate `Engine::kChamberPressurePsi` on `Engine::kThrustLbf` using `commanded_thrust_lbf` |
| `pid_correction_deg` | degrees | `(pid_p_output_deg + pid_i_output_deg) + pid_d_output_deg` for a valid, unclipped PID snapshot |
| `feedforward_output_deg` | degrees | `commanded_valve_angle_deg - pid_correction_deg` |
| `pid_error_psi` | psi | `commanded_chamber_pressure_psi - pt3_psi` when both are finite and available |

The calculated correction includes the controller's existing I-term bounds.
When the combined valve command is not clipped, this is the correction actually
applied before backlash. The decoder rounds the PID additions, feedforward
subtraction, and pressure-error subtraction to binary32. P/I/D were converted
and stored separately, so reconstructed sums can differ slightly from converting
the controller's total correction once.

When `pid_output_clamped` is 1, both `pid_correction_deg` and
`feedforward_output_deg` are blank. A clipped target and three PID terms do not
uniquely identify the original feedforward or the correction actually applied.
RPM is retained instead of spending its slot on an additional feedforward
sample. P/I/D and the pressure error remain available during clipping. These
two calculated angle fields are also blank without a valid PID snapshot or with
nonfinite terms/commanded angle. They remain available when a command equals a
travel limit but no actual clipping occurred.

The error column uses PT3, not PT2, and is independent of PID-snapshot presence
and clipping. Zero thrust still yields a zero goal pressure, so a finite PT3
sample produces the corresponding negative error even outside a running PID
cycle. It is a reconstructed pressure difference, not a second PID update.

The flow and pressure-drop fields are nominal model targets reconstructed from
requested thrust, not measured flow or pressure drop. `commanded_dp_psi` is the
tabulated drop across the valve, not its downstream-pressure target or measured
`PT1 - PT2`.
Both use the interpolation order in `Feedforward::set_thrust_to_mdot()`, with
constants and each arithmetic operation rounded to binary32 to match the Mega.
The decoder does not repeat the firmware's live-PT1 validation, since the logger
records a later PT1 sample; derived targets do not confirm the request succeeded.

Zero thrust produces zero in both derived columns, including tests that omit
thrust and use the zero default. Missing thrust (version 2), negative/nonfinite
thrust, or positive thrust outside the engine table leaves both columns blank;
the decoder does not extrapolate.

`commanded_chamber_pressure_psi` is the nominal chamber-pressure request
reconstructed from thrust, not measured PT3 or the valve-outlet/downstream
pressure target. For thrust `F` between adjacent engine points, it uses
`f = (F - F_low) / (F_high - F_low)` and
`Pc = Pc_low + f * (Pc_high - Pc_low)`. Constants and each arithmetic operation
are rounded to binary32, matching `utils::chamber_pressure_for_thrust()`.
Zero thrust produces zero chamber pressure. Missing thrust (version 2),
negative/nonfinite thrust, and positive thrust below the first table point
produce a blank field. Thrust at or above the last point clamps to the last
chamber pressure, matching the firmware helper; the flow and valve-drop
columns still remain blank above the table. This calculation does not repeat
live-PT1 validation or confirm that the requested control calculation succeeded.

The CSV angle schema uses the four named motor/valve degree fields above. PT3
is immediately after PT1/PT2. The CSV column order is:

```text
timestamp_unix_ms,state,time_valid,motor_present,
pt1_psi,pt2_psi,pt3_psi,motor_temp_c,output_percent,rpm,voltage_v,current_a,
has_status_0,has_status_2,has_telemetry,is_fresh,dropped_records,
commanded_thrust_lbf,commanded_mdot_lbm_per_sec,commanded_dp_psi,
backlash_compensated,measured_motor_angle_deg,commanded_motor_angle_deg,
measured_valve_angle_deg,commanded_valve_angle_deg,commanded_chamber_pressure_psi,
pid_p_output_deg,pid_i_output_deg,pid_d_output_deg,pid_output_clamped,
pid_correction_deg,feedforward_output_deg,pid_error_psi
```

The displayed line breaks are for readability; the actual CSV has one header
row. The two status receive-time columns have been removed from all exports.
The compensation and clipping columns come directly from stored flags.

## Configuration used for calculated fields

The decoder reads `kGearRatio`, `kBacklashCompensation`, and the numeric engine
lookup arrays from `include/config.h`, and the mass conversion constant from
`include/feedforward.h`, caching them per selected header directory. These
constants are not embedded in the binary log. Use headers matching the firmware
used for that test. Angle constants are rounded to binary32 to match the Mega.
For archived headers, select their directory with:

```powershell
python tools/decode_log.py outputs/LOG0000.BIN outputs/LOG0000_derived.csv --model-dir path/to/saved/include
```

The saved directory must contain `config.h` and `feedforward.h`. The decoder
requires numeric literals for the four engine arrays (`kThrustLbf`,
`kLoxMdotKg`, `kValveDpPsi`, and `kChamberPressurePsi`), mass conversion
constant, gear ratio, and backlash compensation. Chamber pressure and thrust
arrays must have matching lengths and positive finite values; the thrust axis
must increase strictly. Gear ratio must be positive;
backlash compensation must be nonnegative; both must be finite.
Missing or malformed required model declarations stop conversion before a CSV
is created; engine tables are loaded only for positive, finite thrust requests.

## Compatibility and validation

`tools/decode_log.py` supports versions 2, 3, and 4, each with 64-byte records.
Version 4 changes field offsets and types, so it uses a new version byte;
decoders that only support versions 2/3 cannot read these new files. No file
header was added. The converter selects the layout from each record's version.

Version 3 uses `struct.Struct("<4BQ9f3If")`. Version 2 uses
`struct.Struct("<4BQ9f3I4s")`, with bytes 60-63 reserved and required to be zero.
The older offsets are retained here for archived logs:

| Offset | Bytes | Version-2 / version-3 field |
| --- | --- | --- |
| 0 | 1 | Version 2 or 3 |
| 1 | 1 | State ID |
| 2 | 1 | Presence/command flags; bits 0-3 as above, bit 4 only in version 3 |
| 3 | 1 | Motor status/freshness flags, bits 0-3 as above |
| 4 | 8 | Unix milliseconds |
| 12 | 4 | PT1 psi |
| 16 | 4 | PT2 psi |
| 20 | 4 | Measured motor angle in valve-side degrees |
| 24 | 4 | Motor temperature, float32 degrees C |
| 28 | 4 | Applied output, float32 percent |
| 32 | 4 | Motor RPM, float32 |
| 36 | 4 | Voltage, float32 V |
| 40 | 4 | Current, float32 A |
| 44 | 4 | Commanded valve angle before backlash, float32 degrees |
| 48 | 4 | Last status-0 receive time, uint32 milliseconds since boot |
| 52 | 4 | Last status-2 receive time, uint32 milliseconds since boot |
| 56 | 4 | Buffer-full drop count, uint32 |
| 60 | 4 | Version 2: zero reserved bytes; version 3: commanded thrust, float32 lbf |

Legacy receive timestamps are parsed as part of the old layout but are no
longer exported. PT3, P/I/D terms, clipping state, and all three new PID-derived
columns are blank for versions 2/3 because the required data were not recorded.
Legacy temperature/current/output retain their original float precision.
Version 2 also has no thrust or backlash flag: its commanded-thrust and derived
model targets are blank, byte 2 bit 4 must be zero, and decoded
`backlash_compensated` is zero so calculated motor/valve angles assume no offset.

Older version-3 firmware may omit the backlash flag or contain earlier caller
bugs: closed loop stored an already compensated command, CdA put the compensation boolean
in thrust and left the flag clear, and actuation converted its command to
degrees before passing it to the logger. The record version cannot distinguish
these files. Decoding them with the new tool does not repair those stored
values or recover an omitted compensation flag. Use the firmware revision to
identify affected runs; a clear flag in an old log is not proof that compensation
was inactive.

The decoder validates the whole input before creating a new CSV. It rejects
unsupported versions, partial records, invalid flags, nonzero version-2 reserved
bytes, a nonzero version-4 free byte, invalid version-4 output percentages, and
invalid timestamps. Existing output files are never overwritten.
Earlier experimental version-1 files are not supported by this converter
because their state/time semantics differed.

Only orderly finalized files are supported. Files left after reset/power loss
may contain unwritten preallocated data. The decoder has no CRC or recovery
guarantee; do not interpret a successfully parsed unfinalized file as verified
data. Copy only after the firmware reports successful closure.
