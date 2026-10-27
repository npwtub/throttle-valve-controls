# cda characterization stack

this script takes the full cts hotv log plus one or more arduino characterization logs and their yorg time offsets, calculates cda for each steady valve setpoint, and combines all tests into one final lookup table.

normal use should only need the logs and offsets. anything related to column names, units, hardware geometry, dwell trimming, or final lut spacing should be changed once in `cda_config.py` instead of changing the processing code.

## basic use

single test:

```powershell
python cda_characterization.py HOTV_Test_data.csv trial_1.csv 120.4
```

multiple tests:

```powershell
python cda_characterization.py HOTV_Test_data.csv `
    trial_1.csv 120.4 `
    trial_2.csv 240.8 `
    trial_3.csv 365.1 `
    trial_4.csv 487.3
```

for every arduino log, the next argument is its yorg offset in seconds.

```text
aligned arduino time = arduino elapsed time + offset
cts time             = cts elapsed time
```

for example, if `trial_6.csv` is the sixth arduino characterization test, its offset should line that file up with the sixth characterization event inside the full hotv cts log.

## what the script does

1. reads the full cts session and each arduino trial
2. finds the needed columns using the aliases in `cda_config.py`
3. converts timestamps and pressures into common units
4. applies the supplied yorg offset to each arduino trial
5. finds each steady valve dwell
6. throws out the beginning/end of each dwell so settling and timing error do not bias the result
7. calculates cts mass flow from the cts pressure drop and test-section geometry
8. calculates valve pressure drop from the arduino upstream/downstream pressure transducers
9. calculates cda for every usable dwell
10. averages up/down measurements within each test at the same angle
11. combines repeated tests and averages the per-test results at each final lut angle
12. writes the final c++ arrays plus csv/plot outputs for checking the result

## cts mass flow

the current hotv data uses `CFM2(psi)` and `CFM1(psi)` for the cts pressure drop:

```text
cts dp = CFM2 - CFM1
```

with the cts geometry and water density, the script uses:

```text
mdot = A1 * sqrt((2 * rho * dp) / ((A1/A2)^2 - 1))
```

`A1`, `A2`, and the fallback density are configured in `cda_config.py`.

## valve cda

for the throttle valve:

```text
valve dp = upstream pt - downstream pt
cda = mdot / sqrt(2 * rho * valve dp)
```

the cts pressure drop used to calculate mdot and the valve pressure drop used to calculate cda are kept separate.

## dwell handling

new arduino logs can use the explicit states:

```text
at_setpoint_up
at_setpoint_down
```

older logs without those states fall back to detecting steady angle plateaus from the valve-position data.

only the middle section of each dwell is used. the trim values are configurable:

```python
"window_start_after_setpoint_s": 0.50,
"window_end_before_departure_s": 0.25,
```

for a 3 second dwell this currently ignores the first 0.5 s and last 0.25 s.

## combining multiple tests

we will probably have tests on two staggered angle grids, for example:

```text
test group 1: 0, 10, 20, 30, ...
test group 2: 5, 15, 25, 35, ...
```

with the final lut spacing set to 5 degrees, these combine into:

```text
0, 5, 10, 15, 20, 25, ...
```

repeated tests at the same angle are averaged.

averaging happens in two stages:

```text
up/down values from one test at one angle
                ↓
      one value for that test
                ↓
values from every test at that angle
                ↓
         final lut value
```

this keeps every physical test equally weighted instead of giving extra weight to a test because it happened to contain more samples or both sweep directions.

## changing log formats

all format-specific settings are in `cda_config.py`.

that includes:

- accepted column names and aliases
- timestamp units
- pressure units
- csv parsing settings
- optional direct cts dp column
- cts upstream/downstream pressure columns
- optional density column
- optional sync/debug column
- arduino valve angle/state/pt columns

if a logger changes a column name, add the new name to the matching alias list. the main processing code should not need to change.

example:

```python
"p_up": {
    "aliases": [
        "pt_up_psi",
        "upstream_pressure_psi",
        "new_logger_name_here",
    ],
    "unit": "psi",
    "required": True,
},
```

## engineering/config values

these still need to be filled in with the real cts geometry/test values:

```python
"hardware": {
    "rho_kg_m3": None,
    "a1_m2": None,
    "a2_m2": None,
},
```

if density is included directly in the cts log, the script can use that instead of the fixed fallback density.

other important settings include:

```python
"lut_angle_increment_deg": 5.0,
"window_start_after_setpoint_s": 0.50,
"window_end_before_departure_s": 0.25,
"up_down_warning_percent": 15.0,
```

## outputs

results are written into `cda_results/`.

main combined outputs:

- `combined_characterization.csv` - every usable dwell from every test
- `combined_test_angle_means.csv` - one averaged value per test per angle
- `combined_lookup.csv` - final averaged lut
- `cda_lookup.txt` - paste-ready c++ arrays
- `combined_cda_vs_angle.png` - individual test results and final lookup curve

per-test intermediate outputs are also saved under:

```text
cda_results/tests/test_xx/
```

these are there so timing, pressure data, mdot, and cda can be checked before trusting the final lookup table.

## final c++ output

`cda_lookup.txt` is formatted so the result can be pasted directly into the feedforward code:

```cpp
const double CdA[] = {
    ...
};

const double angles[] = {
    0.0, 5.0, 10.0, 15.0, ...
};
```

## current status / remaining work

implemented:

- full-session cts log handling
- multiple arduino trials with separate yorg offsets
- configurable column aliases / units / csv formats
- current hotv `CFM2(psi)` / `CFM1(psi)` support
- new state-based dwell detection
- fallback plateau detection for older logs
- configurable middle-of-dwell filtering
- cts mdot calculation
- valve cda calculation
- up/down comparison
- staggered 0 degree / 5 degree test combination
- equal-weight averaging across repeated tests
- final c++ lookup-table output
- diagnostic csvs and plots

still need to do with the real test files:

- fill in / confirm the actual cts `A1`, `A2`, and density values
- use the raw may hotv export and matching raw arduino trials, not the pdf copies
- determine each trial offset in yorg and make sure trial number lines up with the same characterization event in the full cts log
- run the may tests end to end and check that calculated mdot/cda values are physically reasonable
- compare repeated tests and up/down sweeps for consistency
- adjust dwell trimming / warnings if the real data shows they need tuning
- validate the final combined lut before putting it into the feedforward controller

## new sd logger format

`LOG0018.csv` was used as the compatibility check for the newer sd-card logger format. the default config now recognizes:

```text
timestamp_unix_ms       -> time
position_deg            -> measured valve angle
commanded_position_deg  -> commanded valve angle
state                   -> test state
pt1_psi                 -> upstream valve pt
pt2_psi                 -> downstream valve pt
```

it also uses these validity fields when they are present:

```text
time_valid
pt1_present
pt2_present
motor_present
```

if the wiring convention changes and pt1/pt2 are swapped, only change the `p_up` / `p_down` aliases in `cda_config.py`.

there is no fixed arduino sample-rate assumption anymore. the script uses the actual timestamps and only reports the measured rate for diagnostics.

`LOG0018.csv` itself is a format-only example for this stack because both pt presence flags are zero and both pressure columns are empty. the parser/dwell detection works on it, but it cannot calculate valve cda without real pt data.

## what has been checked

- both python files compile cleanly
- the new `LOG0018.csv` format is recognized without editing the processing code
- its unix-ms timestamps are converted without losing the sub-second timing
- the measured logger rate is about 200 hz from the actual timestamps
- the newer presence/validity flags are respected
- angle-plateau fallback detects the repeated steady positions in the new-format example
- a four-test synthetic run was checked with two tests on `0,10,20...90` and two tests on `5,15,25...85`
- the combined result produced `0,5,10,15...90` with two physical tests averaged at each angle
- up/down values are averaged equally inside each test before tests are averaged together
- the script warns if a configured 5 degree lut angle is missing instead of silently filling/interpolating it

actual cda validation still requires logs that contain real valve pt data plus the matching cts data and yorg offsets.
