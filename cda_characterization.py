#!/usr/bin/env python3
"""
process cts + arduino logs into an angle-to-cda lookup table.

usage:
    python cda_characterization.py HOTV_Test_data.csv arduino_log.csv OFFSET_SECONDS

offset convention:
    both logs get shifted so their first timestamp is t=0, then the supplied
    offset gets added to the cts time.
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


# =============================================================================
# one-time test / hardware config
# =============================================================================
# these should stay fixed for a given cts setup / water test configuration.

# only needed if density is not already in the cts log.
RHO_KG_M3: Optional[float] = None

# cts flow-section areas in m^2. this equation expects A1 > A2.
A1_M2: Optional[float] = None
A2_M2: Optional[float] = None

# trim the start/end of each dwell so transients do not get into the lut.
WINDOW_START_AFTER_SETPOINT_S = 0.50
WINDOW_END_BEFORE_DEPARTURE_S = 0.25

# maps the measured encoder angle back to the nominal test setpoint.
ANGLE_STEP_DEG = 10.0
ANGLE_MIN_DEG = 0.0
ANGLE_MAX_DEG = 90.0

# flag a big up/down mismatch instead of silently averaging it.
UP_DOWN_WARNING_PERCENT = 15.0

# reject tiny/nonphysical dp values before sqrt/division.
MIN_CTS_DP_PA = 1.0
MIN_VALVE_DP_PA = 1.0

# current arduino logger uses ms and psi.
ARDUINO_TIME_UNIT = "ms"
ARDUINO_PRESSURE_UNIT = "psi"

# defaults for the cts export; change these if the real file uses other units.
CTS_TIME_UNIT = "auto"
CTS_DP_UNIT = "psi"

# set a column name here only if auto-detection picks the wrong one.
ARDUINO_TIME_COLUMN: Optional[str] = None
ARDUINO_ANGLE_COLUMN: Optional[str] = None
ARDUINO_STATE_COLUMN: Optional[str] = None
ARDUINO_P_UP_COLUMN: Optional[str] = None
ARDUINO_P_DOWN_COLUMN: Optional[str] = None

CTS_TIME_COLUMN: Optional[str] = None
CTS_DP_COLUMN: Optional[str] = None
CTS_P1_COLUMN: Optional[str] = None
CTS_P2_COLUMN: Optional[str] = None
CTS_DENSITY_COLUMN: Optional[str] = None


COLUMN_ALIASES = {
    "arduino_time": [
        "timestamp_ms", "time_ms", "timestamp", "time", "t_ms", "t"
    ],
    "arduino_angle": [
        "position_deg", "angle_deg", "valve_angle_deg", "valve_angle",
        "position", "angle"
    ],
    "arduino_state": ["state", "test_state"],
    "arduino_p_up": [
        "pt_up_psi", "p_up_psi", "upstream_pressure_psi", "upstream_psi",
        "pt_up", "p_up", "upstream_pressure"
    ],
    "arduino_p_down": [
        "pt_down_psi", "p_down_psi", "downstream_pressure_psi", "downstream_psi",
        "pt_down", "p_down", "downstream_pressure"
    ],
    "cts_time": [
        "timestamp_ms", "time_ms", "timestamp_s", "time_s", "timestamp",
        "time", "elapsed_time_s", "elapsed_time", "t"
    ],
    "cts_dp": [
        "delta_p_psi", "dp_psi", "differential_pressure_psi", "delta_p",
        "dp", "differential_pressure", "pressure_drop_psi", "pressure_drop"
    ],
    "cts_p1": [
        "p1_psi", "pressure1_psi", "upstream_pressure_psi", "p1",
        "pressure1", "upstream_pressure"
    ],
    "cts_p2": [
        "p2_psi", "pressure2_psi", "downstream_pressure_psi", "p2",
        "pressure2", "downstream_pressure"
    ],
    "cts_density": [
        "rho_kg_m3", "density_kg_m3", "rho", "density"
    ],
}


PSI_TO_PA = 6894.757293168
KPA_TO_PA = 1000.0
MPA_TO_PA = 1_000_000.0


@dataclass
class Columns:
    arduino_time: str
    arduino_angle: str
    arduino_state: str
    arduino_p_up: str
    arduino_p_down: str
    cts_time: str
    cts_dp: Optional[str]
    cts_p1: Optional[str]
    cts_p2: Optional[str]
    cts_density: Optional[str]


@dataclass
class Dwell:
    dwell_id: int
    direction: str
    raw_start_s: float
    raw_end_s: float
    use_start_s: float
    use_end_s: float


# =============================================================================
# file / column helpers
# =============================================================================

def normalize_name(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", str(name).strip().lower()).strip("_")


def read_table(path: Path) -> pd.DataFrame:
    suffix = path.suffix.lower()
    if suffix in {".xlsx", ".xls"}:
        return pd.read_excel(path)
    if suffix in {".tsv"}:
        return pd.read_csv(path, sep="\t")
    # let pandas figure out comma/tab/semicolon exports.
    return pd.read_csv(path, sep=None, engine="python")


def find_column(
    df: pd.DataFrame,
    manual: Optional[str],
    alias_key: str,
    *,
    required: bool = True,
) -> Optional[str]:
    if manual is not None:
        if manual not in df.columns:
            raise ValueError(
                f"Configured column '{manual}' not found. Available columns: {list(df.columns)}"
            )
        return manual

    normalized = {normalize_name(c): c for c in df.columns}
    for alias in COLUMN_ALIASES[alias_key]:
        key = normalize_name(alias)
        if key in normalized:
            return normalized[key]

    if required:
        raise ValueError(
            f"Could not identify {alias_key} column. Available columns: {list(df.columns)}"
        )
    return None


def detect_columns(arduino: pd.DataFrame, cts: pd.DataFrame) -> Columns:
    cts_dp = find_column(cts, CTS_DP_COLUMN, "cts_dp", required=False)
    cts_p1 = None
    cts_p2 = None
    if cts_dp is None:
        cts_p1 = find_column(cts, CTS_P1_COLUMN, "cts_p1", required=False)
        cts_p2 = find_column(cts, CTS_P2_COLUMN, "cts_p2", required=False)
        if cts_p1 is None or cts_p2 is None:
            raise ValueError(
                "CTS log needs either a differential-pressure column or both P1 and P2 columns."
            )

    return Columns(
        arduino_time=find_column(arduino, ARDUINO_TIME_COLUMN, "arduino_time"),
        arduino_angle=find_column(arduino, ARDUINO_ANGLE_COLUMN, "arduino_angle"),
        arduino_state=find_column(arduino, ARDUINO_STATE_COLUMN, "arduino_state"),
        arduino_p_up=find_column(arduino, ARDUINO_P_UP_COLUMN, "arduino_p_up"),
        arduino_p_down=find_column(arduino, ARDUINO_P_DOWN_COLUMN, "arduino_p_down"),
        cts_time=find_column(cts, CTS_TIME_COLUMN, "cts_time"),
        cts_dp=cts_dp,
        cts_p1=cts_p1,
        cts_p2=cts_p2,
        cts_density=find_column(cts, CTS_DENSITY_COLUMN, "cts_density", required=False),
    )


# =============================================================================
# time / unit conversion
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
        out = numeric.astype(float) * factor
        first = out.dropna().iloc[0]
        return out - first

    dt = pd.to_datetime(series, errors="coerce")
    if dt.notna().sum() < 3:
        raise ValueError(f"Could not parse time column '{column_name}'.")
    first = dt.dropna().iloc[0]
    return (dt - first).dt.total_seconds()


def pressure_to_pa(values: pd.Series | np.ndarray | float, unit: str):
    unit = unit.strip().lower()
    factor = {
        "pa": 1.0,
        "kpa": KPA_TO_PA,
        "mpa": MPA_TO_PA,
        "psi": PSI_TO_PA,
    }.get(unit)
    if factor is None:
        raise ValueError(f"Unsupported pressure unit '{unit}'.")
    return values * factor


def infer_pressure_unit_from_column(column_name: str, fallback: str) -> str:
    n = normalize_name(column_name)
    if n.endswith("_psi") or "psi" in n:
        return "psi"
    if n.endswith("_kpa") or "kpa" in n:
        return "kpa"
    if n.endswith("_mpa") or "mpa" in n:
        return "mpa"
    if n.endswith("_pa") or n == "pa":
        return "pa"
    return fallback


# =============================================================================
# physics
# =============================================================================

def validate_engineering_constants(cts_has_density: bool) -> None:
    if A1_M2 is None or A2_M2 is None:
        raise ValueError(
            "Set A1_M2 and A2_M2 in the configuration block before processing real data."
        )
    if A1_M2 <= 0 or A2_M2 <= 0:
        raise ValueError("A1_M2 and A2_M2 must both be positive.")
    if A1_M2 <= A2_M2:
        raise ValueError(
            "Expected A1_M2 > A2_M2 for the supplied CTS equation. Check section definitions."
        )
    if not cts_has_density and RHO_KG_M3 is None:
        raise ValueError(
            "Set RHO_KG_M3 or provide a CTS density column before processing real data."
        )
    if RHO_KG_M3 is not None and RHO_KG_M3 <= 0:
        raise ValueError("RHO_KG_M3 must be positive.")


def calculate_mdot_kg_s(dp_pa: pd.Series, rho_kg_m3: pd.Series) -> pd.Series:
    """calculate mass flow from the cts pressure drop."""
    ratio_term = (A1_M2 / A2_M2) ** 2 - 1.0
    valid = (dp_pa > MIN_CTS_DP_PA) & (rho_kg_m3 > 0)
    out = pd.Series(np.nan, index=dp_pa.index, dtype=float)
    out.loc[valid] = A1_M2 * np.sqrt(
        (2.0 * rho_kg_m3.loc[valid] * dp_pa.loc[valid]) / ratio_term
    )
    return out


def calculate_cda_m2(mdot_kg_s: float, rho_kg_m3: float, valve_dp_pa: float) -> float:
    if not np.isfinite(mdot_kg_s) or not np.isfinite(rho_kg_m3) or not np.isfinite(valve_dp_pa):
        return float("nan")
    if rho_kg_m3 <= 0 or valve_dp_pa <= MIN_VALVE_DP_PA:
        return float("nan")
    return mdot_kg_s / math.sqrt(2.0 * rho_kg_m3 * valve_dp_pa)


# =============================================================================
# preprocessing / dwell detection
# =============================================================================

def prepare_arduino(df: pd.DataFrame, cols: Columns) -> pd.DataFrame:
    out = df.copy()
    out["time_s"] = time_to_seconds(out[cols.arduino_time], cols.arduino_time, ARDUINO_TIME_UNIT)
    out["angle_deg"] = pd.to_numeric(out[cols.arduino_angle], errors="coerce")
    out["state"] = out[cols.arduino_state].astype(str).str.strip().str.lower()

    p_up_unit = infer_pressure_unit_from_column(cols.arduino_p_up, ARDUINO_PRESSURE_UNIT)
    p_down_unit = infer_pressure_unit_from_column(cols.arduino_p_down, ARDUINO_PRESSURE_UNIT)
    out["p_up_pa"] = pressure_to_pa(pd.to_numeric(out[cols.arduino_p_up], errors="coerce"), p_up_unit)
    out["p_down_pa"] = pressure_to_pa(pd.to_numeric(out[cols.arduino_p_down], errors="coerce"), p_down_unit)
    out["valve_dp_pa"] = out["p_up_pa"] - out["p_down_pa"]

    out = out.sort_values("time_s").reset_index(drop=True)
    return out


def prepare_cts(df: pd.DataFrame, cols: Columns, offset_s: float) -> pd.DataFrame:
    out = df.copy()
    base_time = time_to_seconds(out[cols.cts_time], cols.cts_time, CTS_TIME_UNIT)

    # this is the one yorg value the user has to provide for each run.
    out["time_s"] = base_time + offset_s

    if cols.cts_dp is not None:
        unit = infer_pressure_unit_from_column(cols.cts_dp, CTS_DP_UNIT)
        out["cts_dp_pa"] = pressure_to_pa(pd.to_numeric(out[cols.cts_dp], errors="coerce"), unit)
    else:
        assert cols.cts_p1 is not None and cols.cts_p2 is not None
        p1_unit = infer_pressure_unit_from_column(cols.cts_p1, CTS_DP_UNIT)
        p2_unit = infer_pressure_unit_from_column(cols.cts_p2, CTS_DP_UNIT)
        p1 = pressure_to_pa(pd.to_numeric(out[cols.cts_p1], errors="coerce"), p1_unit)
        p2 = pressure_to_pa(pd.to_numeric(out[cols.cts_p2], errors="coerce"), p2_unit)

        # if the cts log does not give dp directly, build it from the two pressures.
        out["cts_dp_pa"] = p1 - p2

    if cols.cts_density is not None:
        out["rho_kg_m3"] = pd.to_numeric(out[cols.cts_density], errors="coerce")
    else:
        out["rho_kg_m3"] = float(RHO_KG_M3)

    # turn the cts pressure drop into the mass flow used in the valve cda equation.
    out["mdot_kg_s"] = calculate_mdot_kg_s(out["cts_dp_pa"], out["rho_kg_m3"])
    out = out.sort_values("time_s").reset_index(drop=True)
    return out


def detect_dwells(arduino: pd.DataFrame) -> list[Dwell]:
    state = arduino["state"]
    valid = state.isin(["at_setpoint_up", "at_setpoint_down"])

    # start a new dwell whenever the setpoint state changes.
    boundary = valid.ne(valid.shift(fill_value=False)) | state.ne(state.shift())
    group_id = boundary.cumsum()

    dwells: list[Dwell] = []
    next_id = 0
    for _, group in arduino.loc[valid].groupby(group_id[valid]):
        label = str(group["state"].iloc[0])
        direction = "up" if label.endswith("_up") else "down"
        raw_start = float(group["time_s"].iloc[0])
        raw_end = float(group["time_s"].iloc[-1])
        # only keep the middle of the dwell so settling/movement does not skew cda.
        use_start = raw_start + WINDOW_START_AFTER_SETPOINT_S
        use_end = raw_end - WINDOW_END_BEFORE_DEPARTURE_S
        if use_end <= use_start:
            raise ValueError(
                f"Dwell {next_id} is too short after trimming: raw=[{raw_start:.3f}, {raw_end:.3f}] s, "
                f"usable=[{use_start:.3f}, {use_end:.3f}] s."
            )
        dwells.append(Dwell(next_id, direction, raw_start, raw_end, use_start, use_end))
        next_id += 1

    if not dwells:
        raise ValueError(
            "No 'at_setpoint_up'/'at_setpoint_down' dwell regions were found in the Arduino log."
        )
    return dwells


def nominal_angle_from_measured(angle_deg: float) -> float:
    nominal = round(angle_deg / ANGLE_STEP_DEG) * ANGLE_STEP_DEG
    return float(np.clip(nominal, ANGLE_MIN_DEG, ANGLE_MAX_DEG))


# =============================================================================
# characterization
# =============================================================================

def characterize(arduino: pd.DataFrame, cts: pd.DataFrame, dwells: Iterable[Dwell]) -> pd.DataFrame:
    rows = []
    for dwell in dwells:
        # grab both logs over the exact same aligned-time window.
        a = arduino[(arduino["time_s"] >= dwell.use_start_s) & (arduino["time_s"] <= dwell.use_end_s)]
        c = cts[(cts["time_s"] >= dwell.use_start_s) & (cts["time_s"] <= dwell.use_end_s)]

        if len(a) == 0:
            print(f"WARNING: no Arduino samples in dwell {dwell.dwell_id}", file=sys.stderr)
            continue
        if len(c) == 0:
            print(f"WARNING: no CTS samples in dwell {dwell.dwell_id}; check time offset", file=sys.stderr)
            continue

        # collapse the usable part of one dwell into one characterization point.
        actual_angle = float(a["angle_deg"].mean())
        nominal_angle = nominal_angle_from_measured(actual_angle)
        p_up_pa = float(a["p_up_pa"].mean())
        p_down_pa = float(a["p_down_pa"].mean())
        valve_dp_pa = float(a["valve_dp_pa"].mean())

        mdot = float(c["mdot_kg_s"].mean())
        rho = float(c["rho_kg_m3"].mean())
        cts_dp_pa = float(c["cts_dp_pa"].mean())

        # mdot comes from cts; valve dp comes from the two arduino pressure transducers.
        cda = calculate_cda_m2(mdot, rho, valve_dp_pa)

        rows.append({
            "dwell_id": dwell.dwell_id,
            "direction": dwell.direction,
            "nominal_angle_deg": nominal_angle,
            "actual_angle_deg_mean": actual_angle,
            "actual_angle_deg_std": float(a["angle_deg"].std(ddof=0)),
            "window_start_s": dwell.use_start_s,
            "window_end_s": dwell.use_end_s,
            "arduino_samples": int(len(a)),
            "cts_samples": int(len(c)),
            "p_up_pa_mean": p_up_pa,
            "p_down_pa_mean": p_down_pa,
            "valve_dp_pa_mean": valve_dp_pa,
            "valve_dp_pa_std": float(a["valve_dp_pa"].std(ddof=0)),
            "cts_dp_pa_mean": cts_dp_pa,
            "cts_dp_pa_std": float(c["cts_dp_pa"].std(ddof=0)),
            "rho_kg_m3_mean": rho,
            "mdot_kg_s_mean": mdot,
            "mdot_kg_s_std": float(c["mdot_kg_s"].std(ddof=0)),
            "cda_m2": cda,
        })

    result = pd.DataFrame(rows)
    if result.empty:
        raise ValueError("No valid characterization rows were produced.")
    return result.sort_values(["nominal_angle_deg", "direction"]).reset_index(drop=True)


def build_lookup(characterization: pd.DataFrame) -> pd.DataFrame:
    records = []
    for angle, group in characterization.groupby("nominal_angle_deg", sort=True):
        up = group.loc[group["direction"] == "up", "cda_m2"].dropna()
        down = group.loc[group["direction"] == "down", "cda_m2"].dropna()

        up_val = float(up.mean()) if len(up) else float("nan")
        down_val = float(down.mean()) if len(down) else float("nan")

        # keep up/down separate for diagnostics, then average what is actually available.
        available = [v for v in [up_val, down_val] if np.isfinite(v)]
        final = float(np.mean(available)) if available else float("nan")

        diff_pct = float("nan")
        if np.isfinite(up_val) and np.isfinite(down_val):
            # percent difference makes a bad sync or real hysteresis easy to spot.
            denom = 0.5 * (abs(up_val) + abs(down_val))
            if denom > 0:
                diff_pct = abs(up_val - down_val) / denom * 100.0

        records.append({
            "angle_deg": float(angle),
            "cda_up_m2": up_val,
            "cda_down_m2": down_val,
            "up_down_difference_percent": diff_pct,
            "cda_final_m2": final,
        })

    lookup = pd.DataFrame(records).sort_values("angle_deg").reset_index(drop=True)
    return lookup


# =============================================================================
# outputs
# =============================================================================

def format_cpp_array(name: str, values: Iterable[float], precision: int) -> str:
    formatted = []
    for value in values:
        if not np.isfinite(value):
            raise ValueError(f"Cannot emit C++ lookup table: {name} contains NaN/invalid value.")
        if name.lower() == "angles":
            formatted.append(f"{value:.1f}")
        else:
            formatted.append(f"{value:.{precision}e}")
    body = ", ".join(formatted)
    return f"constexpr double {name}[] = {{{body}}};"


def write_cpp_lookup(lookup: pd.DataFrame, out_path: Path) -> str:
    angles_line = format_cpp_array("angles", lookup["angle_deg"], 1)
    cda_line = format_cpp_array("CdA", lookup["cda_final_m2"], 10)
    # keep this output directly pasteable into the feedforward code.
    text = (
        "// generated by cda_characterization.py\n"
        f"{angles_line}\n"
        f"{cda_line}\n"
        "constexpr unsigned int CDA_TABLE_SIZE = sizeof(angles) / sizeof(angles[0]);\n"
    )
    out_path.write_text(text, encoding="utf-8")
    return text


def make_plots(arduino: pd.DataFrame, cts: pd.DataFrame, lookup: pd.DataFrame, out_dir: Path) -> None:
    # plotting is optional, so only import it when we actually need it.
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"WARNING: plots skipped because matplotlib is unavailable: {exc}", file=sys.stderr)
        return

    # compare the up sweep, down sweep, and final averaged lut.
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(lookup["angle_deg"], lookup["cda_up_m2"], marker="o", label="up")
    ax.plot(lookup["angle_deg"], lookup["cda_down_m2"], marker="o", label="down")
    ax.plot(lookup["angle_deg"], lookup["cda_final_m2"], marker="o", label="final")
    ax.set_xlabel("Valve angle (deg)")
    ax.set_ylabel("CdA (m^2)")
    ax.set_title("Throttle valve CdA characterization")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "cda_vs_angle.png", dpi=160)
    plt.close(fig)

    # normalize both dp signals so the time alignment is easy to eyeball.
    a = arduino[["time_s", "valve_dp_pa"]].dropna().copy()
    c = cts[["time_s", "cts_dp_pa"]].dropna().copy()
    if len(a) > 1 and len(c) > 1:
        def normalize_signal(s: pd.Series) -> pd.Series:
            std = s.std(ddof=0)
            if not np.isfinite(std) or std == 0:
                return s * 0.0
            return (s - s.mean()) / std

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(a["time_s"], normalize_signal(a["valve_dp_pa"]), label="Arduino valve dP (normalized)")
        ax.plot(c["time_s"], normalize_signal(c["cts_dp_pa"]), label="CTS dP (normalized)")
        ax.set_xlabel("Aligned time (s)")
        ax.set_ylabel("Normalized signal")
        ax.set_title("Time-sync sanity check")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / "sync_check.png", dpi=160)
        plt.close(fig)


def print_diagnostics(columns: Columns, characterization: pd.DataFrame, lookup: pd.DataFrame) -> None:
    print("\nCharacterized dwells:")
    for row in characterization.itertuples(index=False):
        cda_text = f"{row.cda_m2:.6e}" if np.isfinite(row.cda_m2) else "INVALID"
        print(
            f"  {row.nominal_angle_deg:5.1f} deg {row.direction:4s} | "
            f"actual={row.actual_angle_deg_mean:7.3f} deg | "
            f"mdot={row.mdot_kg_s_mean:.6f} kg/s | "
            f"valve dP={row.valve_dp_pa_mean:.1f} Pa | CdA={cda_text} m^2"
        )

    print("\nOpening / closing comparison:")
    for row in lookup.itertuples(index=False):
        if np.isfinite(row.up_down_difference_percent):
            suffix = ""
            if row.up_down_difference_percent > UP_DOWN_WARNING_PERCENT:
                suffix = "  <-- WARNING"
            print(
                f"  {row.angle_deg:5.1f} deg: "
                f"up={row.cda_up_m2:.6e}, down={row.cda_down_m2:.6e}, "
                f"difference={row.up_down_difference_percent:.1f}%{suffix}"
            )
        else:
            print(
                f"  {row.angle_deg:5.1f} deg: only one sweep direction available"
            )


def process(cts_path: Path, arduino_path: Path, offset_s: float, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    cts_raw = read_table(cts_path)
    arduino_raw = read_table(arduino_path)

    # figure out the real log column names once so the rest of the stack stays generic.
    cols = detect_columns(arduino_raw, cts_raw)

    print("\nDetected columns:")
    for key, value in cols.__dict__.items():
        print(f"  {key}: {value}")

    validate_engineering_constants(cts_has_density=cols.cts_density is not None)

    arduino = prepare_arduino(arduino_raw, cols)
    cts = prepare_cts(cts_raw, cols, offset_s)

    # from here on everything is aligned time -> dwell points -> final lut.
    dwells = detect_dwells(arduino)
    characterization = characterize(arduino, cts, dwells)
    lookup = build_lookup(characterization)

    characterization.to_csv(output_dir / "characterization.csv", index=False)
    lookup.to_csv(output_dir / "lookup_table.csv", index=False)

    print_diagnostics(cols, characterization, lookup)

    cpp_text = write_cpp_lookup(lookup, output_dir / "cda_lookup.txt")
    make_plots(arduino, cts, lookup, output_dir)

    print("\nPaste-ready C++ lookup table:\n")
    print(cpp_text)
    print(f"Outputs written to: {output_dir.resolve()}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a throttle-valve CdA lookup table from CTS + Arduino logs."
    )
    parser.add_argument("cts_log", type=Path, help="HOTV_Test_data CTS log")
    parser.add_argument("arduino_log", type=Path, help="Arduino valve-characterization log")
    parser.add_argument(
        "offset_s",
        type=float,
        help=(
            "Time offset in seconds from Yorg. Both logs are first normalized to t=0, "
            "then aligned_cts_time = normalized_cts_time + offset_s."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("cda_results"),
        help="Output directory (default: cda_results)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        process(args.cts_log, args.arduino_log, args.offset_s, args.output_dir)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
