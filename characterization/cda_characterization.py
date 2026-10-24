#!/usr/bin/env python3
"""
combine one or more arduino cda tests against a full cts hotv log.

single test:
    python cda_characterization.py HOTV_Test_data.csv arduino_trial_1.csv 120.4

multiple tests:
    python cda_characterization.py HOTV_Test_data.csv \
        arduino_trial_1.csv 120.4 \
        arduino_trial_2.csv 240.8 \
        arduino_trial_3.csv 365.1

every arduino file is followed by its yorg offset in seconds.

    aligned_arduino_time = arduino_elapsed_time + offset
    aligned_cts_time     = cts_elapsed_time
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
import pandas as pd

from cda_config import CONFIG


PSI_TO_PA = 6894.757293168
KPA_TO_PA = 1000.0
MPA_TO_PA = 1_000_000.0

A1_M2 = CONFIG["hardware"]["a1_m2"]
A2_M2 = CONFIG["hardware"]["a2_m2"]
RHO_KG_M3 = CONFIG["hardware"]["rho_kg_m3"]
P = CONFIG["processing"]


@dataclass
class Columns:
    arduino_time: str
    arduino_angle: str
    arduino_state: Optional[str]
    arduino_time_valid: Optional[str]
    arduino_p_up_present: Optional[str]
    arduino_p_down_present: Optional[str]
    arduino_motor_present: Optional[str]
    arduino_commanded_angle: Optional[str]
    arduino_p_up: str
    arduino_p_down: str
    cts_time: str
    cts_dp: Optional[str]
    cts_upstream: Optional[str]
    cts_downstream: Optional[str]
    cts_density: Optional[str]
    cts_sync: Optional[str]


@dataclass
class Dwell:
    dwell_id: int
    direction: str
    nominal_angle_deg: float
    raw_start_s: float
    raw_end_s: float
    use_start_s: float
    use_end_s: float


@dataclass
class TrialInput:
    path: Path
    offset_s: float
    test_id: str


# =============================================================================
# input format helpers
# =============================================================================

def normalize_name(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", str(name).strip().lower()).strip("_")


def read_table(path: Path, source: str) -> pd.DataFrame:
    suffix = path.suffix.lower()
    if suffix == ".pdf":
        raise ValueError("use the raw csv/xlsx/log export instead of the pdf")
    if suffix in {".xlsx", ".xls"}:
        return pd.read_excel(path)
    if suffix == ".tsv":
        return pd.read_csv(path, sep="\t")

    # csv parsing lives in cda_config.py so a new logger format does not need code changes.
    kwargs = dict(CONFIG[source].get("read_csv_kwargs", {}))
    return pd.read_csv(path, **kwargs)


def field_config(source: str, field: str) -> dict:
    return CONFIG[source]["columns"][field]


def find_column(df: pd.DataFrame, source: str, field: str) -> Optional[str]:
    spec = field_config(source, field)
    aliases = spec.get("aliases", [])
    required = bool(spec.get("required", True))

    normalized = {normalize_name(c): c for c in df.columns}
    for alias in aliases:
        key = normalize_name(alias)
        if key in normalized:
            return normalized[key]

    if required:
        raise ValueError(
            f"could not find {source}.{field}. edit its aliases in cda_config.py. "
            f"available columns: {list(df.columns)}"
        )
    return None


def detect_columns(arduino: pd.DataFrame, cts: pd.DataFrame) -> Columns:
    cts_dp = find_column(cts, "cts", "dp")
    cts_upstream = None
    cts_downstream = None

    if cts_dp is None:
        cts_upstream = find_column(cts, "cts", "p_up")
        cts_downstream = find_column(cts, "cts", "p_down")
        if cts_upstream is None or cts_downstream is None:
            raise ValueError(
                "cts needs either a dp column or both p_up and p_down. edit cda_config.py if the column names changed"
            )

    return Columns(
        arduino_time=find_column(arduino, "arduino", "time"),
        arduino_angle=find_column(arduino, "arduino", "angle"),
        arduino_state=find_column(arduino, "arduino", "state"),
        arduino_time_valid=find_column(arduino, "arduino", "time_valid"),
        arduino_p_up_present=find_column(arduino, "arduino", "p_up_present"),
        arduino_p_down_present=find_column(arduino, "arduino", "p_down_present"),
        arduino_motor_present=find_column(arduino, "arduino", "motor_present"),
        arduino_commanded_angle=find_column(arduino, "arduino", "commanded_angle"),
        arduino_p_up=find_column(arduino, "arduino", "p_up"),
        arduino_p_down=find_column(arduino, "arduino", "p_down"),
        cts_time=find_column(cts, "cts", "time"),
        cts_dp=cts_dp,
        cts_upstream=cts_upstream,
        cts_downstream=cts_downstream,
        cts_density=find_column(cts, "cts", "density"),
        cts_sync=find_column(cts, "cts", "sync"),
    )


# =============================================================================
# time / units
# =============================================================================

def infer_time_unit(column_name: str, configured: str) -> str:
    if configured != "auto":
        return configured
    n = normalize_name(column_name)
    if n.endswith("_ms") or "millisecond" in n:
        return "ms"
    if n.endswith("_us") or "microsecond" in n:
        return "us"
    return "s"


def time_to_seconds(series: pd.Series, column_name: str, configured_unit: str) -> pd.Series:
    numeric = pd.to_numeric(series, errors="coerce")
    if numeric.notna().sum() >= max(3, int(0.8 * len(series))):
        unit = infer_time_unit(column_name, configured_unit)
        factor = {"s": 1.0, "ms": 1e-3, "us": 1e-6}[unit]
        numeric = numeric.astype(float)
        first = numeric.dropna().iloc[0]
        # subtract first so large unix timestamps keep their sub-second resolution.
        return (numeric - first) * factor

    dt = pd.to_datetime(series, errors="coerce")
    if dt.notna().sum() < 3:
        raise ValueError(f"could not parse time column '{column_name}'")
    first = dt.dropna().iloc[0]
    return (dt - first).dt.total_seconds()


def pressure_to_pa(values, unit: str):
    factor = {
        "pa": 1.0,
        "kpa": KPA_TO_PA,
        "mpa": MPA_TO_PA,
        "psi": PSI_TO_PA,
    }.get(str(unit).strip().lower())
    if factor is None:
        raise ValueError(f"unsupported pressure unit '{unit}'")
    return values * factor


def infer_pressure_unit(column_name: str, fallback: str) -> str:
    n = normalize_name(column_name)
    if "psi" in n:
        return "psi"
    if "kpa" in n:
        return "kpa"
    if "mpa" in n:
        return "mpa"
    if n.endswith("_pa") or n == "pa":
        return "pa"
    return fallback


def estimate_rate_hz(time_s: pd.Series) -> float:
    t = pd.to_numeric(time_s, errors="coerce").dropna().to_numpy(dtype=float)
    if len(t) < 3:
        return float("nan")
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0)]
    return float(1.0 / np.median(dt)) if len(dt) else float("nan")


def rate_warning(name: str, actual_hz: float, expected_hz: Optional[float]) -> None:
    if not np.isfinite(actual_hz):
        print(f"warning: could not estimate {name} sample rate", file=sys.stderr)
        return
    print(f"  {name}: {actual_hz:.2f} hz")
    if expected_hz is None or expected_hz <= 0:
        return
    diff_pct = abs(actual_hz - expected_hz) / expected_hz * 100.0
    if diff_pct > P["rate_warning_percent"]:
        print(
            f"warning: {name} rate is {actual_hz:.2f} hz, expected about {expected_hz:.2f} hz",
            file=sys.stderr,
        )


# =============================================================================
# physics
# =============================================================================

def validate_engineering_constants(cts_has_density: bool) -> None:
    if A1_M2 is None or A2_M2 is None:
        raise ValueError("set hardware.a1_m2 and hardware.a2_m2 in cda_config.py")
    if A1_M2 <= 0 or A2_M2 <= 0 or A1_M2 <= A2_M2:
        raise ValueError("cts geometry needs a1_m2 > a2_m2 > 0")
    if not cts_has_density and RHO_KG_M3 is None:
        raise ValueError("set hardware.rho_kg_m3 or configure a cts density column")


def calculate_mdot_kg_s(dp_pa: pd.Series, rho_kg_m3: pd.Series) -> pd.Series:
    ratio_term = (A1_M2 / A2_M2) ** 2 - 1.0
    valid = (dp_pa > P["min_cts_dp_pa"]) & (rho_kg_m3 > 0)
    out = pd.Series(np.nan, index=dp_pa.index, dtype=float)

    # this is noah's cts mdot calculation after solving the dp equation for v1.
    out.loc[valid] = A1_M2 * np.sqrt(
        (2.0 * rho_kg_m3.loc[valid] * dp_pa.loc[valid]) / ratio_term
    )
    return out


def calculate_cda_m2(mdot_kg_s: float, rho_kg_m3: float, valve_dp_pa: float) -> float:
    if not all(np.isfinite(v) for v in [mdot_kg_s, rho_kg_m3, valve_dp_pa]):
        return float("nan")
    if rho_kg_m3 <= 0 or valve_dp_pa <= P["min_valve_dp_pa"]:
        return float("nan")
    return mdot_kg_s / math.sqrt(2.0 * rho_kg_m3 * valve_dp_pa)


# =============================================================================
# preprocessing
# =============================================================================

def prepare_cts(df: pd.DataFrame, cols: Columns) -> pd.DataFrame:
    out = df.copy()
    time_unit = field_config("cts", "time").get("unit", "auto")
    out["time_s"] = time_to_seconds(out[cols.cts_time], cols.cts_time, time_unit)

    if cols.cts_dp is not None:
        fallback = field_config("cts", "dp").get("unit", "psi")
        unit = infer_pressure_unit(cols.cts_dp, fallback)
        out["cts_dp_pa"] = pressure_to_pa(pd.to_numeric(out[cols.cts_dp], errors="coerce"), unit)
    else:
        up_fallback = field_config("cts", "p_up").get("unit", "psi")
        down_fallback = field_config("cts", "p_down").get("unit", "psi")
        up_unit = infer_pressure_unit(cols.cts_upstream, up_fallback)
        down_unit = infer_pressure_unit(cols.cts_downstream, down_fallback)
        p_up = pressure_to_pa(pd.to_numeric(out[cols.cts_upstream], errors="coerce"), up_unit)
        p_down = pressure_to_pa(pd.to_numeric(out[cols.cts_downstream], errors="coerce"), down_unit)

        # the current hotv logs use cfm2-cfm1 for the cts pressure drop.
        out["cts_dp_pa"] = p_up - p_down

    if cols.cts_density is not None:
        out["rho_kg_m3"] = pd.to_numeric(out[cols.cts_density], errors="coerce")
    else:
        out["rho_kg_m3"] = float(RHO_KG_M3)

    if cols.cts_sync is not None:
        fallback = field_config("cts", "sync").get("unit", "psi")
        unit = infer_pressure_unit(cols.cts_sync, fallback)
        out["cts_sync_pa"] = pressure_to_pa(pd.to_numeric(out[cols.cts_sync], errors="coerce"), unit)

    out["mdot_kg_s"] = calculate_mdot_kg_s(out["cts_dp_pa"], out["rho_kg_m3"])
    return out.sort_values("time_s").reset_index(drop=True)


def prepare_arduino(df: pd.DataFrame, cols: Columns, offset_s: float) -> pd.DataFrame:
    out = df.copy()
    time_unit = field_config("arduino", "time").get("unit", "ms")
    elapsed = time_to_seconds(out[cols.arduino_time], cols.arduino_time, time_unit)

    # this is the only per-test timing value the user should have to provide.
    out["time_s"] = elapsed + offset_s
    out["angle_deg"] = pd.to_numeric(out[cols.arduino_angle], errors="coerce")
    out["state"] = (
        out[cols.arduino_state].astype(str).str.strip().str.lower()
        if cols.arduino_state is not None
        else ""
    )

    if cols.arduino_commanded_angle is not None:
        out["commanded_angle_deg"] = pd.to_numeric(
            out[cols.arduino_commanded_angle], errors="coerce"
        )
    else:
        out["commanded_angle_deg"] = np.nan

    up_fallback = field_config("arduino", "p_up").get("unit", "psi")
    down_fallback = field_config("arduino", "p_down").get("unit", "psi")
    up_unit = infer_pressure_unit(cols.arduino_p_up, up_fallback)
    down_unit = infer_pressure_unit(cols.arduino_p_down, down_fallback)
    out["p_up_pa"] = pressure_to_pa(pd.to_numeric(out[cols.arduino_p_up], errors="coerce"), up_unit)
    out["p_down_pa"] = pressure_to_pa(pd.to_numeric(out[cols.arduino_p_down], errors="coerce"), down_unit)

    # new sd logs carry presence flags, so use them instead of trusting placeholder values.
    if cols.arduino_p_up_present is not None:
        present = pd.to_numeric(out[cols.arduino_p_up_present], errors="coerce").fillna(0) != 0
        out.loc[~present, "p_up_pa"] = np.nan
    if cols.arduino_p_down_present is not None:
        present = pd.to_numeric(out[cols.arduino_p_down_present], errors="coerce").fillna(0) != 0
        out.loc[~present, "p_down_pa"] = np.nan
    if cols.arduino_motor_present is not None:
        present = pd.to_numeric(out[cols.arduino_motor_present], errors="coerce").fillna(0) != 0
        out.loc[~present, "angle_deg"] = np.nan
    if cols.arduino_time_valid is not None:
        valid_time = pd.to_numeric(out[cols.arduino_time_valid], errors="coerce").fillna(0) != 0
        out = out.loc[valid_time].copy()

    out["valve_dp_pa"] = out["p_up_pa"] - out["p_down_pa"]
    return out.sort_values("time_s").reset_index(drop=True)


def snap_angle(angle_deg: float) -> float:
    step = P["lut_angle_increment_deg"]
    snapped = math.floor((angle_deg / step) + 0.5) * step
    return float(np.clip(snapped, P["angle_min_deg"], P["angle_max_deg"]))


def make_dwell(dwell_id: int, direction: str, measured_angle: float, raw_start: float, raw_end: float) -> Dwell:
    nominal = snap_angle(measured_angle)
    if abs(measured_angle - nominal) > P["angle_snap_warning_deg"]:
        print(
            f"warning: measured dwell angle {measured_angle:.2f} deg snapped to {nominal:.1f} deg",
            file=sys.stderr,
        )

    use_start = raw_start + P["window_start_after_setpoint_s"]
    use_end = raw_end - P["window_end_before_departure_s"]
    if use_end <= use_start:
        raise ValueError(f"dwell {dwell_id} at {nominal:.1f} deg is too short after trimming")
    return Dwell(dwell_id, direction, nominal, raw_start, raw_end, use_start, use_end)


def infer_directions_from_sequence(items: list[dict]) -> None:
    for i, item in enumerate(items):
        if i == 0:
            if len(items) > 1 and items[1]["angle"] < item["angle"]:
                item["direction"] = "down"
            else:
                item["direction"] = "up"
            continue
        delta = item["angle"] - items[i - 1]["angle"]
        item["direction"] = "up" if delta > 0 else "down" if delta < 0 else items[i - 1]["direction"]


def detect_dwells_from_state(arduino: pd.DataFrame) -> list[Dwell]:
    state = arduino["state"]
    valid = state.isin(["at_setpoint_up", "at_setpoint_down", "at_setpoint"])
    if not valid.any():
        return []

    boundary = valid.ne(valid.shift(fill_value=False)) | state.ne(state.shift())
    group_id = boundary.cumsum()
    groups = []
    for _, group in arduino.loc[valid].groupby(group_id[valid]):
        label = str(group["state"].iloc[0])
        angle = float(group["angle_deg"].median())
        direction = "up" if label.endswith("_up") else "down" if label.endswith("_down") else None
        groups.append({
            "angle": angle,
            "direction": direction,
            "raw_start": float(group["time_s"].iloc[0]),
            "raw_end": float(group["time_s"].iloc[-1]),
        })

    if any(g["direction"] is None for g in groups):
        infer_directions_from_sequence(groups)

    return [
        make_dwell(i, g["direction"], g["angle"], g["raw_start"], g["raw_end"])
        for i, g in enumerate(groups)
    ]


def detect_dwells_from_angle(arduino: pd.DataFrame) -> list[Dwell]:
    work = arduino[["time_s", "angle_deg"]].dropna().copy()
    if len(work) < 3:
        return []

    dt = work["time_s"].diff()
    speed = (work["angle_deg"].diff() / dt).replace([np.inf, -np.inf], np.nan).abs()
    speed = speed.rolling(3, center=True, min_periods=1).median()
    nominal = work["angle_deg"].map(snap_angle)
    near_setpoint = (work["angle_deg"] - nominal).abs() <= P["angle_plateau_tolerance_deg"]
    slow = speed.fillna(0.0) <= P["max_plateau_speed_deg_s"]
    candidate = near_setpoint & slow
    boundary = candidate.ne(candidate.shift(fill_value=False)) | nominal.ne(nominal.shift())
    group_id = boundary.cumsum()

    plateaus = []
    for _, group in work.loc[candidate].groupby(group_id[candidate]):
        raw_start = float(group["time_s"].iloc[0])
        raw_end = float(group["time_s"].iloc[-1])
        if raw_end - raw_start < P["min_raw_dwell_duration_s"]:
            continue
        plateaus.append({
            "angle": float(group["angle_deg"].median()),
            "raw_start": raw_start,
            "raw_end": raw_end,
        })

    infer_directions_from_sequence(plateaus)
    return [
        make_dwell(i, p["direction"], p["angle"], p["raw_start"], p["raw_end"])
        for i, p in enumerate(plateaus)
    ]


def detect_dwells(arduino: pd.DataFrame) -> tuple[list[Dwell], str]:
    dwells = detect_dwells_from_state(arduino)
    if dwells:
        return dwells, "state"
    dwells = detect_dwells_from_angle(arduino)
    if dwells:
        return dwells, "angle plateau"
    raise ValueError("could not find characterization dwells")


def validate_overlap(arduino: pd.DataFrame, cts: pd.DataFrame, dwells: list[Dwell]) -> None:
    cts_start = float(cts["time_s"].min())
    cts_end = float(cts["time_s"].max())
    for dwell in dwells:
        if dwell.use_start_s < cts_start or dwell.use_end_s > cts_end:
            raise ValueError("one or more dwell windows fall outside the cts log. check the yorg offset")


# =============================================================================
# characterization / aggregation
# =============================================================================

def characterize(arduino: pd.DataFrame, cts: pd.DataFrame, dwells: Iterable[Dwell], test_id: str, source_file: str) -> pd.DataFrame:
    rows = []
    for dwell in dwells:
        # both logs are averaged over the same middle section of the dwell.
        a = arduino[(arduino["time_s"] >= dwell.use_start_s) & (arduino["time_s"] <= dwell.use_end_s)]
        c = cts[(cts["time_s"] >= dwell.use_start_s) & (cts["time_s"] <= dwell.use_end_s)]
        if a.empty or c.empty:
            print(f"warning: skipped {test_id} dwell {dwell.dwell_id} because one log had no samples", file=sys.stderr)
            continue

        mdot = float(c["mdot_kg_s"].mean())
        rho = float(c["rho_kg_m3"].mean())
        valve_dp = float(a["valve_dp_pa"].mean())

        rows.append({
            "test_id": test_id,
            "source_file": source_file,
            "dwell_id": dwell.dwell_id,
            "direction": dwell.direction,
            "nominal_angle_deg": dwell.nominal_angle_deg,
            "actual_angle_deg_mean": float(a["angle_deg"].mean()),
            "actual_angle_deg_std": float(a["angle_deg"].std(ddof=0)),
            "commanded_angle_deg_mean": float(a["commanded_angle_deg"].mean()),
            "window_start_s": dwell.use_start_s,
            "window_end_s": dwell.use_end_s,
            "arduino_samples": int(len(a)),
            "cts_samples": int(len(c)),
            "p_up_pa_mean": float(a["p_up_pa"].mean()),
            "p_down_pa_mean": float(a["p_down_pa"].mean()),
            "valve_dp_pa_mean": valve_dp,
            "cts_dp_pa_mean": float(c["cts_dp_pa"].mean()),
            "rho_kg_m3_mean": rho,
            "mdot_kg_s_mean": mdot,
            "cda_m2": calculate_cda_m2(mdot, rho, valve_dp),
        })

    result = pd.DataFrame(rows)
    if result.empty:
        raise ValueError(f"{test_id} produced no valid characterization rows")
    return result


def build_test_angle_means(characterization: pd.DataFrame) -> pd.DataFrame:
    # average each direction first so up/down get equal weight inside a test.
    rows = []
    for (test_id, angle), group in characterization.groupby(["test_id", "nominal_angle_deg"], sort=True):
        up = group.loc[group["direction"] == "up", "cda_m2"].dropna()
        down = group.loc[group["direction"] == "down", "cda_m2"].dropna()
        up_mean = float(up.mean()) if len(up) else float("nan")
        down_mean = float(down.mean()) if len(down) else float("nan")
        direction_means = [v for v in [up_mean, down_mean] if np.isfinite(v)]
        if not direction_means:
            continue

        if np.isfinite(up_mean) and np.isfinite(down_mean):
            denom = max(abs(up_mean), abs(down_mean), 1e-30)
            diff_pct = abs(up_mean - down_mean) / denom * 100.0
            if diff_pct > P["up_down_warning_percent"]:
                print(
                    f"warning: {test_id} {float(angle):.1f} deg up/down cda differ by {diff_pct:.1f}%",
                    file=sys.stderr,
                )
        else:
            diff_pct = float("nan")

        rows.append({
            "test_id": test_id,
            "angle_deg": float(angle),
            "cda_test_mean_m2": float(np.mean(direction_means)),
            "cda_up_m2": up_mean,
            "cda_down_m2": down_mean,
            "up_down_difference_percent": diff_pct,
            "measurements_in_test": int(group["cda_m2"].notna().sum()),
        })
    return pd.DataFrame(rows)


def build_combined_lookup(test_means: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for angle, group in test_means.groupby("angle_deg", sort=True):
        values = group["cda_test_mean_m2"].dropna()
        if values.empty:
            continue
        rows.append({
            "angle_deg": float(angle),
            "cda_final_m2": float(values.mean()),
            "cda_std_across_tests_m2": float(values.std(ddof=1)) if len(values) > 1 else 0.0,
            "tests_used": int(len(values)),
        })
    return pd.DataFrame(rows).sort_values("angle_deg").reset_index(drop=True)


def warn_for_missing_lut_angles(lookup: pd.DataFrame) -> None:
    if lookup.empty:
        raise ValueError("no valid cda values were produced")
    step = float(P["lut_angle_increment_deg"])
    expected = np.arange(P["angle_min_deg"], P["angle_max_deg"] + 0.5 * step, step)
    actual = set(np.round(lookup["angle_deg"].to_numpy(dtype=float), 9))
    missing = [float(v) for v in expected if round(float(v), 9) not in actual]
    if missing:
        values = ", ".join(f"{v:g}" for v in missing)
        print(f"warning: final lut is missing configured angles: {values}", file=sys.stderr)


# =============================================================================
# outputs
# =============================================================================

def format_cpp_array(name: str, values: Iterable[float], precision: int) -> str:
    formatted = []
    for value in values:
        if not np.isfinite(value):
            raise ValueError(f"cannot emit c++ array {name}: invalid value")
        formatted.append(f"{value:.1f}" if name == "angles" else f"{value:.{precision}e}")
    return f"const double {name}[] = {{{', '.join(formatted)}}};"


def write_cpp_lookup(lookup: pd.DataFrame, out_path: Path) -> str:
    cda_line = format_cpp_array("CdA", lookup["cda_final_m2"], 10)
    angles_line = format_cpp_array("angles", lookup["angle_deg"], 1)
    text = (
        "// generated by cda_characterization.py\n"
        f"{cda_line}\n"
        f"{angles_line}\n"
        "const unsigned int CDA_TABLE_SIZE = sizeof(angles) / sizeof(angles[0]);\n"
    )
    out_path.write_text(text, encoding="utf-8")
    return text


def make_combined_plot(lookup: pd.DataFrame, test_means: pd.DataFrame, out_dir: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"warning: plot skipped: {exc}", file=sys.stderr)
        return

    fig = plt.figure()
    ax = fig.add_subplot(111)
    for test_id, group in test_means.groupby("test_id"):
        ax.plot(group["angle_deg"], group["cda_test_mean_m2"], marker="o", label=test_id)
    ax.plot(lookup["angle_deg"], lookup["cda_final_m2"], marker="o", linewidth=2.5, label="final average")
    ax.set_xlabel("valve angle (deg)")
    ax.set_ylabel("CdA (m^2)")
    ax.set_title("combined throttle valve CdA characterization")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "combined_cda_vs_angle.png", dpi=160)
    plt.close(fig)


def process_trial(cts: pd.DataFrame, cts_raw: pd.DataFrame, trial: TrialInput, output_dir: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    arduino_raw = read_table(trial.path, "arduino")
    cols = detect_columns(arduino_raw, cts_raw)
    arduino = prepare_arduino(arduino_raw, cols, trial.offset_s)

    # the newer sd logger can still have pt columns even when the pts were not plugged in.
    # catch that here so a format-only log does not turn into an empty/invalid lut later.
    valid_pt = arduino["p_up_pa"].notna() & arduino["p_down_pa"].notna()
    if not valid_pt.any():
        raise ValueError(
            f"{trial.test_id} log format was recognized, but it has no usable upstream/downstream pt data"
        )

    dwells, dwell_source = detect_dwells(arduino)
    validate_overlap(arduino, cts, dwells)

    print(f"\n{trial.test_id}: {trial.path.name}")
    print(f"  offset: {trial.offset_s:.6f} s")
    print(f"  dwell detection: {dwell_source} ({len(dwells)} dwells)")
    rate_warning("arduino", estimate_rate_hz(arduino["time_s"]), P["expected_arduino_rate_hz"])

    characterization = characterize(arduino, cts, dwells, trial.test_id, trial.path.name)
    test_means = build_test_angle_means(characterization)

    trial_dir = output_dir / "tests" / trial.test_id
    trial_dir.mkdir(parents=True, exist_ok=True)
    characterization.to_csv(trial_dir / "characterization.csv", index=False)
    test_means.to_csv(trial_dir / "angle_means.csv", index=False)
    return characterization, test_means


def process(cts_path: Path, trials: list[TrialInput], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    cts_raw = read_table(cts_path, "cts")

    # use the first trial only to resolve the cts column mapping once.
    first_arduino = read_table(trials[0].path, "arduino")
    first_cols = detect_columns(first_arduino, cts_raw)
    validate_engineering_constants(first_cols.cts_density is not None)
    cts = prepare_cts(cts_raw, first_cols)

    print("estimated cts sample rate:")
    rate_warning("cts", estimate_rate_hz(cts["time_s"]), P["expected_cts_rate_hz"])

    all_characterization = []
    all_test_means = []
    for trial in trials:
        characterization, test_means = process_trial(cts, cts_raw, trial, output_dir)
        all_characterization.append(characterization)
        all_test_means.append(test_means)

    combined_characterization = pd.concat(all_characterization, ignore_index=True)
    combined_test_means = pd.concat(all_test_means, ignore_index=True)
    combined_lookup = build_combined_lookup(combined_test_means)
    warn_for_missing_lut_angles(combined_lookup)

    combined_characterization.to_csv(output_dir / "combined_characterization.csv", index=False)
    combined_test_means.to_csv(output_dir / "combined_test_angle_means.csv", index=False)
    combined_lookup.to_csv(output_dir / "combined_lookup.csv", index=False)
    cpp_text = write_cpp_lookup(combined_lookup, output_dir / "cda_lookup.txt")
    make_combined_plot(combined_lookup, combined_test_means, output_dir)

    print("\ncombined lookup:")
    for row in combined_lookup.itertuples(index=False):
        print(
            f"  {row.angle_deg:5.1f} deg | cda={row.cda_final_m2:.6e} | "
            f"tests={row.tests_used} | std={row.cda_std_across_tests_m2:.3e}"
        )

    print("\npaste-ready c++ lookup table:\n")
    print(cpp_text)
    print(f"outputs written to: {output_dir.resolve()}")


def parse_trial_pairs(values: list[str]) -> list[TrialInput]:
    if len(values) < 2 or len(values) % 2 != 0:
        raise ValueError("after the cts log, provide arduino_log offset pairs")

    trials = []
    for i in range(0, len(values), 2):
        path = Path(values[i])
        try:
            offset = float(values[i + 1])
        except ValueError as exc:
            raise ValueError(f"offset for {path} must be a number") from exc
        trials.append(TrialInput(path=path, offset_s=offset, test_id=f"test_{len(trials) + 1:02d}"))
    return trials


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="generate one combined cda lookup table from one or more arduino tests"
    )
    parser.add_argument("cts_log", type=Path, help="full hotv cts log")
    parser.add_argument(
        "trial_inputs",
        nargs="+",
        help="pairs of ARDUINO_LOG OFFSET_SECONDS, repeated for every test",
    )
    parser.add_argument("--output-dir", type=Path, default=Path("cda_results"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        trials = parse_trial_pairs(args.trial_inputs)
        process(args.cts_log, trials, args.output_dir)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
