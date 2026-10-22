"""easy-to-edit settings for the cda characterization stack."""

CONFIG = {
    # change these once if the log format changes. normal runs should not need edits here.
    "arduino": {
        "read_csv_kwargs": {"sep": None, "engine": "python"},
        "columns": {
            "time": {
                "aliases": ["timestamp_ms", "time_ms", "timestamp", "time", "t_ms", "t"],
                "unit": "ms",
                "required": True,
            },
            "angle": {
                "aliases": ["position_deg", "angle_deg", "valve_angle_deg", "valve_angle", "position", "angle"],
                "required": True,
            },
            "state": {
                "aliases": ["state", "test_state"],
                "required": False,
            },
            "p_up": {
                "aliases": ["pt_up_psi", "p_up_psi", "upstream_pressure_psi", "upstream_psi", "pt_up", "p_up", "upstream_pressure"],
                "unit": "psi",
                "required": True,
            },
            "p_down": {
                "aliases": ["pt_down_psi", "p_down_psi", "downstream_pressure_psi", "downstream_psi", "pt_down", "p_down", "downstream_pressure"],
                "unit": "psi",
                "required": True,
            },
        },
    },
    "cts": {
        "read_csv_kwargs": {"sep": None, "engine": "python"},
        "columns": {
            "time": {
                "aliases": ["timestamp_ms", "time_ms", "timestamp_s", "time_s", "timestamp", "time", "elapsed_time_s", "elapsed_time", "t"],
                "unit": "auto",
                "required": True,
            },
            # if a direct dp column exists it gets used. otherwise cfm2-cfm1 is used.
            "dp": {
                "aliases": ["delta_p_psi", "dp_psi", "differential_pressure_psi", "delta_p", "dp", "differential_pressure", "pressure_drop_psi", "pressure_drop"],
                "unit": "psi",
                "required": False,
            },
            "p_up": {
                "aliases": ["CFM2(psi)", "cfm2_psi", "cfm2", "p_up_psi", "upstream_pressure_psi", "p_up", "upstream_pressure"],
                "unit": "psi",
                "required": False,
            },
            "p_down": {
                "aliases": ["CFM1(psi)", "cfm1_psi", "cfm1", "p_down_psi", "downstream_pressure_psi", "p_down", "downstream_pressure"],
                "unit": "psi",
                "required": False,
            },
            "density": {
                "aliases": ["rho_kg_m3", "density_kg_m3", "rho", "density"],
                "required": False,
            },
            # this is only for the sync plot. ctpt is useful for matching cts events in yorg.
            "sync": {
                "aliases": ["CTPT(psi)", "ctpt_psi", "ctpt"],
                "unit": "psi",
                "required": False,
            },
        },
    },
    "hardware": {
        # fill these in once from the cts geometry. a1 should be the larger section.
        "rho_kg_m3": None,
        "a1_m2": None,
        "a2_m2": None,
    },
    "processing": {
        # only keep the middle of each dwell so timing error and settling do not bias cda.
        "window_start_after_setpoint_s": 0.50,
        "window_end_before_departure_s": 0.25,

        # all test grids get snapped onto this final lut grid. this lets 0,10,20... and
        # 5,15,25... tests combine into a 0,5,10,15... table.
        "lut_angle_increment_deg": 5.0,
        "angle_min_deg": 0.0,
        "angle_max_deg": 90.0,
        "angle_snap_warning_deg": 2.5,

        # fallback dwell detection for older logs without at_setpoint_up/down states.
        "angle_plateau_tolerance_deg": 2.5,
        "max_plateau_speed_deg_s": 3.0,
        "min_raw_dwell_duration_s": 1.0,

        # diagnostics / validation.
        "up_down_warning_percent": 15.0,
        "min_cts_dp_pa": 1.0,
        "min_valve_dp_pa": 1.0,
        "expected_arduino_rate_hz": 25.0,
        "expected_cts_rate_hz": None,
        "rate_warning_percent": 25.0,
    },
}
