"""
Flask web UI for rangesim.

Run:
    python app.py
Then open http://localhost:5000 in your browser.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from flask import Flask, render_template, request, jsonify
from aircraft import AircraftConfig
from simulation import breguet_range_nm, step_simulation, sensitivity_table
from atmosphere import ms_to_ktas, G0_M_S2
from engine import ENGINE_CONFIGS
import math
import traceback

app = Flask(__name__)

MATERIAL_FACTORS = {
    "printed_ti_al": 0.80,
    "cfrp":          0.75,
    "aluminium":     1.00,
}

MATERIAL_LABELS = {
    "printed_ti_al": "3D-printed Ti-6Al-4V + graded Al",
    "cfrp":          "CFRP composite",
    "aluminium":     "Conventional aluminium",
}

ENGINE_TYPE_LABELS = {k: v["label"] for k, v in ENGINE_CONFIGS.items()}

SENSITIVITY_PARAMS = {
    "fan_diameter_m":      ("Fan diameter",       "m",    0.8,  3.0,  7),
    "blade_sweep_deg":     ("Blade sweep",        "°",    0.0, 45.0,  7),
    "wingspan_m":          ("Wingspan",            "m",    6.0, 16.0,  6),
    "engine_power_kw":     ("Engine power",        "kW",  80.0,500.0,  7),
    "fuel_mass_kg":        ("Fuel mass",           "kg",  60.0,400.0,  7),
    "cabin_width_m":       ("Cabin width",         "m",   0.85, 1.50,  6),
    "cruise_speed_ktas":   ("Cruise speed",        "KTAS",120, 320,    7),
    "cruise_altitude_ft":  ("Cruise altitude",     "ft", 5000,60000,   7),
}


def _build_config(data: dict) -> AircraftConfig:
    sf = MATERIAL_FACTORS.get(data.get("material", "printed_ti_al"), 0.80)
    return AircraftConfig(
        fan_diameter_m     = float(data.get("fan_diameter_m",    1.60)),
        blade_sweep_deg    = float(data.get("blade_sweep_deg",    0.0)),
        engine_power_kw    = float(data.get("engine_power_kw",  220.0)),
        wingspan_m         = float(data.get("wingspan_m",        10.0)),
        cabin_width_m      = float(data.get("cabin_width_m",     1.10)),
        fuel_mass_kg       = float(data.get("fuel_mass_kg",     160.0)),
        cruise_speed_ktas  = float(data.get("cruise_speed_ktas",210.0)),
        cruise_altitude_ft = float(data.get("cruise_altitude_ft",20000)),
        stall_speed_ktas   = float(data.get("stall_speed_ktas",   65.0)),
        structural_factor  = sf,
        engine_type        = data.get("engine_type", "super_turboshaft"),
    )


@app.route("/")
def index():
    return render_template("index.html",
                           sensitivity_params=SENSITIVITY_PARAMS,
                           material_labels=MATERIAL_LABELS,
                           engine_type_labels=ENGINE_TYPE_LABELS)


@app.route("/run", methods=["POST"])
def run():
    try:
        data = request.get_json()
        cfg = _build_config(data)
        warnings = cfg.validate()

        w   = cfg.weights
        atm = cfg.atmosphere
        rho = atm["density_kg_m3"]
        sos = atm["speed_of_sound_m_s"]
        nu  = atm["kinematic_viscosity_m2_s"]
        v   = cfg.cruise_speed_ms
        W_n = cfg.mtow_kg * G0_M_S2

        aero = cfg.aero.summary(W_n, v, rho, nu)
        drag_n = cfg.aero.drag_n(W_n, v, rho, nu)
        eta_prop = cfg.propfan.efficiency(v, drag_n, rho, sos)

        strategy = data.get("strategy", "constant_altitude")
        stepped  = step_simulation(cfg, strategy=strategy)
        breguet  = breguet_range_nm(cfg)

        # Cruise fuel flow at initial MTOW
        P_cruise_kw  = cfg.propfan.shaft_power_w(v, drag_n, rho, sos) / 1000.0
        fuel_flow_gph = cfg.engine.fuel_flow_L_h(P_cruise_kw) / 3.78541

        # Fuel in gallons, MPG, trip cost
        range_miles   = stepped["range_nm"] * 1.15078
        liters_burned = stepped["fuel_burned_kg"] / cfg.engine.fuel_density_kg_l
        gal_burned    = liters_burned / 3.78541
        range_mpg     = range_miles / gal_burned if gal_burned > 0 else 0.0
        trip_cost_usd = gal_burned * 2.50

        # Cruise log for the chart (sample to ≤50 points)
        log = stepped.get("log", [])
        step = max(1, len(log) // 50)
        chart_log = log[::step]

        return jsonify({
            "ok": True,
            "warnings": warnings,
            "weights": {
                "mtow_kg":              round(w["mtow_kg"],       1),
                "dry_mass_kg":          round(w["dry_mass_kg"],   1),
                "empty_weight_kg":      round(w["empty_weight_kg"],1),
                "empty_fraction":       round(w["empty_weight_fraction"],3),
                "payload_kg":           round(w["payload_kg"],    1),
                "fuel_kg":              round(w["fuel_kg"],        1),
                "wing_kg":              round(w["wing_kg"],        1),
                "canard_kg":            round(w["canard_kg"],      1),
                "fuselage_kg":          round(w["fuselage_kg"],    1),
                "landing_gear_kg":      round(w["landing_gear_kg"],1),
                "landing_gear_leg_m":   round(w["landing_gear_leg_length_m"],2),
                "engine_kg":            round(w["engine_kg"],      1),
                "engine_install_kg":    round(w["engine_install_kg"],1),
                "propfan_kg":           round(w["propfan_kg"],     1),
                "fuel_system_kg":       round(w["fuel_system_kg"], 1),
                "titanium_mass_kg":     round(w["titanium_mass_kg"], 1),
                "aluminum_mass_kg":     round(w["aluminum_mass_kg"], 1),
                "primary_structure_kg": round(w["primary_structure_kg"], 1),
                "structural_factor":    w["structural_factor"],
                "wing_sizing_driver":   w["wing_sizing_driver"],
            },
            "geometry": {
                "wing_area_m2":    round(w["wing_area_m2"],    2),
                "aspect_ratio":    round(w["aspect_ratio"],    2),
                "wing_loading":    round(cfg.mtow_kg * G0_M_S2 / w["wing_area_m2"], 0),
                "fuselage_length": round(w["fuselage_length_m"],  2),
                "fuselage_diam":   round(w["fuselage_diameter_m"],2),
            },
            "atmosphere": {
                "temperature_c":   round(atm["temperature_k"] - 273.15, 1),
                "density":         round(rho, 4),
                "density_ratio":   round(rho / 1.225, 4),
                "mach":            round(cfg.cruise_mach, 3),
                "sos_ktas":        round(ms_to_ktas(sos), 1),
            },
            "engine": {
                "engine_type":     cfg.engine.engine_type,
                "bsfc":            round(cfg.engine.sfc_kg_per_kwh(), 3),
                "power_alt_kw":    round(cfg.engine.max_power_at_altitude_kw(rho), 1),
                "dry_weight_kg":   round(cfg.engine.dry_weight_kg(), 1),
                "thermal_eff_pct": round(cfg.engine.thermal_efficiency * 100, 0),
            },
            "propfan": {
                "rpm":             round(cfg.propfan.design_rpm(v), 0),
                "tip_mach":        round(cfg.propfan.tip_speed_ms(cfg.propfan.design_rpm(v)) / sos, 3),
                "eff_tip_mach":    round(cfg.propfan.effective_tip_mach(v, sos), 3),
                "sweep_deg":       cfg.propfan.blade_sweep_deg,
                "eta_prop":        round(eta_prop * 100, 1),
                "max_speed_ktas":  round(ms_to_ktas(cfg.propfan.max_feasible_speed_ms(sos)), 0),
                "disk_area_m2":    round(cfg.propfan.disk_area_m2, 2),
            },
            "aero": {
                "cl_cruise":   round(aero["cl_cruise"],          4),
                "cd0":         round(aero["cd0_cruise"],         5),
                "cd_total":    round(aero["cd_total_cruise"],    5),
                "ld_cruise":   round(aero["ld_cruise"],          2),
                "best_ld":     round(aero["best_ld"],            2),
                "best_ld_ktas":round(ms_to_ktas(aero["best_ld_speed_ms"]), 0),
            },
            "range": {
                "breguet_nm":     round(breguet.get("range_nm",0),  0),
                "breguet_km":     round(breguet.get("range_km",0),  0),
                "step_nm":        round(stepped["range_nm"],         0),
                "step_km":        round(stepped["range_km"],         0),
                "endurance_hr":   round(stepped["endurance_hr"],     1),
                "avg_ld":         round(stepped["avg_ld"],           2),
                "avg_eta":        round(stepped["avg_propulsive_efficiency"]*100, 1),
                "fuel_burned_gal": round(gal_burned,                  1),
                "fuel_flow_gph":  round(fuel_flow_gph,               2),
                "range_mpg":      round(range_mpg,                   1),
                "trip_cost_usd":  round(trip_cost_usd,               0),
            },
            "chart_log": chart_log,
        })
    except Exception as e:
        return jsonify({"ok": False, "error": str(e), "trace": traceback.format_exc()})


@app.route("/sensitivity", methods=["POST"])
def sensitivity():
    try:
        data   = request.get_json()
        cfg    = _build_config(data)
        param  = data.get("sens_param", "fan_diameter_m")
        if param not in SENSITIVITY_PARAMS:
            return jsonify({"ok": False, "error": f"Unknown param {param}"})

        label, unit, lo, hi, n = SENSITIVITY_PARAMS[param]
        values = [round(lo + i * (hi - lo) / (n - 1), 4) for i in range(n)]

        strategy = data.get("strategy", "constant_altitude")
        rows = sensitivity_table(cfg, param, values, strategy=strategy)

        return jsonify({
            "ok":    True,
            "param": param,
            "label": label,
            "unit":  unit,
            "rows": [
                {
                    "value":    round(r["param_value"], 4),
                    "mtow":     round(r["mtow_kg"],     1),
                    "empty_w":  round(r["empty_weight_kg"], 1),
                    "lg_leg":   round(r["landing_gear_leg_length_m"], 2),
                    "wing_s":   round(r["wing_area_m2"], 2),
                    "ar":       round(r["aspect_ratio"], 1),
                    "ld":       round(r["avg_ld"],       2),
                    "range_nm": round(r["range_nm"],     0),
                    "endur_hr": round(r["endurance_hr"], 1),
                }
                for r in rows
            ],
        })
    except Exception as e:
        return jsonify({"ok": False, "error": str(e), "trace": traceback.format_exc()})


@app.route("/optimize", methods=["POST"])
def optimize():
    """
    Grid-search optimizer: find fan_diameter_m, wingspan_m, engine_power_kw
    that maximise Breguet range.  Cruise altitude is fixed (taken from the
    request).  Also fixed: fuel load, cruise speed, cabin width, stall speed,
    material, engine type.
    """
    try:
        data = request.get_json()

        engine_type = data.get("engine_type", "super_turboshaft")
        ecfg = ENGINE_CONFIGS.get(engine_type, ENGINE_CONFIGS["super_turboshaft"])
        max_pwr = ecfg["max_power_kw"]
        min_pwr = max(30.0, max_pwr * 0.04)

        # All fixed parameters (including cruise_altitude_ft)
        fixed = {}
        for key in ("fuel_mass_kg", "cruise_speed_ktas", "cabin_width_m",
                    "stall_speed_ktas", "cruise_altitude_ft",
                    "material", "engine_type", "blade_sweep_deg", "strategy"):
            if key in data:
                fixed[key] = data[key]

        # ── Coarse grid (3-D: fan × wingspan × power) ───────────────
        fan_vals = [0.8, 1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.0]
        ws_vals  = [6.0, 7.5, 9.0, 10.5, 12.0, 13.5, 15.0, 16.0]
        # 7 power levels log-spaced min→max
        pw_vals  = [round(min_pwr * (max_pwr / min_pwr) ** (i / 6.0), 1) for i in range(7)]

        best_range  = -1.0
        best_params = None

        def _eval(fd, ws, pw):
            params = dict(fixed, fan_diameter_m=fd, wingspan_m=ws, engine_power_kw=pw)
            cfg = _build_config(params)
            atm = cfg.atmosphere
            rho = atm["density_kg_m3"]
            sos = atm["speed_of_sound_m_s"]
            nu  = atm["kinematic_viscosity_m2_s"]
            v   = cfg.cruise_speed_ms
            W_n = cfg.mtow_kg * G0_M_S2
            drag_n  = cfg.aero.drag_n(W_n, v, rho, nu)
            P_avail = cfg.engine.max_power_at_altitude_kw(rho) * 1000.0
            T_avail = cfg.propfan.max_thrust_n(v, P_avail, rho, sos)
            if T_avail < drag_n * 1.01:
                return None, None   # infeasible
            r = breguet_range_nm(cfg).get("range_nm", 0.0)
            return r, params

        for fd in fan_vals:
            for ws in ws_vals:
                for pw in pw_vals:
                    try:
                        r, p = _eval(fd, ws, pw)
                        if r is not None and r > best_range:
                            best_range, best_params = r, p
                    except Exception:
                        pass

        if best_params is None:
            return jsonify({"ok": False, "error": "No feasible configuration found in grid."})

        # ── Refinement: ±25 % around best ───────────────────────────
        fd0 = best_params["fan_diameter_m"]
        ws0 = best_params["wingspan_m"]
        pw0 = best_params["engine_power_kw"]

        def _clamp(v, lo, hi): return max(lo, min(hi, v))

        fan_vals2 = sorted({_clamp(fd0 * f, 0.8, 3.0)        for f in [0.80,0.88,0.94,1.00,1.06,1.13,1.20,1.28]})
        ws_vals2  = sorted({_clamp(ws0 * f, 6.0, 16.0)       for f in [0.80,0.88,0.94,1.00,1.06,1.13,1.20,1.28]})
        pw_vals2  = sorted({_clamp(pw0 * f, min_pwr, max_pwr) for f in [0.65,0.80,1.00,1.20,1.40]})

        for fd in fan_vals2:
            for ws in ws_vals2:
                for pw in pw_vals2:
                    try:
                        r, p = _eval(fd, ws, pw)
                        if r is not None and r > best_range:
                            best_range, best_params = r, p
                    except Exception:
                        pass

        opt = {
            "fan_diameter_m":   round(best_params["fan_diameter_m"],  2),
            "wingspan_m":       round(best_params["wingspan_m"],      2),
            "engine_power_kw":  round(best_params["engine_power_kw"], 0),
            "breguet_range_nm": round(best_range, 0),
        }

        return jsonify({"ok": True, "params": opt})

    except Exception as e:
        return jsonify({"ok": False, "error": str(e), "trace": traceback.format_exc()})


if __name__ == "__main__":
    print("rangesim UI  →  http://localhost:5000")
    app.run(debug=False, host="0.0.0.0", port=5000)
