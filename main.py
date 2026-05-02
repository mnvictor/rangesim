#!/usr/bin/env python3
"""
rangesim — max-range simulator for a two-seat canard pusher-propfan aircraft.

Quick start
-----------
    python main.py                          # default configuration
    python main.py --fan-diameter 1.8       # wider fans → longer gear, etc.
    python main.py --fuel 200 --altitude 25000
    python main.py --sensitivity fan_diameter 1.2 1.4 1.6 1.8 2.0

All dimensional inputs are in the units shown in the help text.
All outputs include auto-derived quantities that cascade from your inputs.
"""

import argparse
import sys
import math

from aircraft import AircraftConfig
from simulation import breguet_range_nm, step_simulation, sensitivity_table
from atmosphere import ms_to_ktas, G0_M_S2


# ─────────────────────────────────────────────────────────────────────────────
# Output formatting
# ─────────────────────────────────────────────────────────────────────────────

SEP = "─" * 68
DOUBLE_SEP = "═" * 68


def _hdr(title: str) -> str:
    pad = (68 - len(title) - 2) // 2
    return f"\n{'═' * pad} {title} {'═' * (68 - pad - len(title) - 2)}"


def _row(label: str, value: str, unit: str = "") -> None:
    unit_str = f"  {unit}" if unit else ""
    print(f"  {label:<38} {value}{unit_str}")


def _section(title: str) -> None:
    print(f"\n  {title}")
    print(f"  {SEP[:len(title) + 2]}")


def print_aircraft_summary(cfg: AircraftConfig) -> None:
    summ = cfg.full_summary()
    w = cfg.weights
    atm = cfg.atmosphere
    rho = atm["density_kg_m3"]
    sos = atm["speed_of_sound_m_s"]
    nu = atm["kinematic_viscosity_m2_s"]
    v = cfg.cruise_speed_ms
    W_n = cfg.mtow_kg * G0_M_S2
    drag_n = cfg.aero.drag_n(W_n, v, rho, nu)

    print(_hdr("AIRCRAFT CONFIGURATION"))

    # ── User inputs ──────────────────────────────────────────────────────────
    _section("USER INPUTS")
    material_names = {0.80: "3D-printed Ti-6Al-4V + graded Al", 0.75: "CFRP composite", 1.00: "Conventional aluminium"}
    mat_label = material_names.get(cfg.structural_factor, f"custom (factor {cfg.structural_factor:.2f})")
    _row("Primary structure",          mat_label)
    _row("  ↳ structural weight factor", f"{cfg.structural_factor:.2f}",   "× Al baseline")
    _row("Fan diameter",               f"{cfg.fan_diameter_m:.2f}",        "m")
    _row("Engine power (SL rated)",    f"{cfg.engine_power_kw:.0f}",       "kW")
    _row("Wingspan",                   f"{cfg.wingspan_m:.1f}",            "m")
    _row("Cabin width",                f"{cfg.cabin_width_m:.2f}",         "m")
    _row("Fuel loaded",                f"{cfg.fuel_mass_kg:.0f}",          "kg")
    _row("Cruise speed",               f"{cfg.cruise_speed_ktas:.0f}",     "KTAS")
    _row("Cruise altitude",            f"{cfg.cruise_altitude_ft:,.0f}",   "ft")

    # ── Atmosphere ───────────────────────────────────────────────────────────
    _section("ATMOSPHERE AT CRUISE ALTITUDE")
    _row("Temperature",                f"{atm['temperature_k'] - 273.15:.1f}",  "°C")
    _row("Density",                    f"{rho:.4f}",                            "kg/m³")
    _row("Density ratio (σ)",          f"{rho / 1.225:.4f}")
    _row("Speed of sound",             f"{ms_to_ktas(sos):.0f}",               "KTAS")
    _row("Cruise Mach number",         f"{cfg.cruise_mach:.3f}")

    # ── Engine ───────────────────────────────────────────────────────────────
    _section("ENGINE")
    eng = cfg.engine
    _row("Thermal efficiency",         f"{60:.0f}",                        "%")
    _row("Max power at altitude",      f"{eng.max_power_at_altitude_kw(rho):.0f}", "kW")
    _row("BSFC",                       f"{eng.sfc_kg_per_kwh():.3f}",      "kg/(kW·h)")
    _row("Engine dry weight",          f"{eng.dry_weight_kg():.1f}",       "kg")
    _row("Total powerplant weight",    f"{eng.total_powerplant_weight_kg():.1f}", "kg")

    # ── Propfan ──────────────────────────────────────────────────────────────
    _section("COUNTER-ROTATING PUSHER PROPFAN")
    pf = summ["propfan"]
    _row("Fan diameter",               f"{pf['fan_diameter_m']:.2f}",      "m")
    _row("Disk area",                  f"{pf['disk_area_m2']:.2f}",        "m²")
    _row("Design RPM",                 f"{pf['design_rpm']:.0f}",          "rpm")
    _row("Tip speed",                  f"{pf['tip_speed_ms']:.0f}",        "m/s")
    _row("Tip Mach",                   f"{pf['tip_mach']:.3f}")
    _row("Design advance ratio",       f"{pf['design_advance_ratio']:.2f}")
    _row("Assembly weight",            f"{pf['assembly_weight_kg']:.1f}",  "kg")
    eta_prop = summ["propulsive_efficiency_cruise"]
    _row("Propulsive efficiency (cruise)", f"{eta_prop * 100:.1f}",         "%")

    # ── Weights ──────────────────────────────────────────────────────────────
    _section("WEIGHT BREAKDOWN")
    _row("Wing structure",             f"{w['wing_kg']:.1f}",              "kg")
    _row("Canard",                     f"{w['canard_kg']:.1f}",            "kg")
    _row("Fuselage",                   f"{w['fuselage_kg']:.1f}",          "kg")
    _row("Landing gear",               f"{w['landing_gear_kg']:.1f}",      "kg")
    _row("  ↳ leg length",             f"{w['landing_gear_leg_length_m']:.2f}", "m")
    _row("Engine",                     f"{w['engine_kg']:.1f}",            "kg")
    _row("Engine installation",        f"{w['engine_install_kg']:.1f}",    "kg")
    _row("Propfan assembly",           f"{w['propfan_kg']:.1f}",           "kg")
    _row("Fuel system",                f"{w['fuel_system_kg']:.1f}",       "kg")
    _row("Avionics / electrical",      f"{w['avionics_electrical_kg']:.1f}", "kg")
    _row("Furnishings / misc",         f"{w['furnishings_kg']:.1f}",       "kg")
    print(f"  {'─' * 50}")
    _row("EMPTY WEIGHT",               f"{w['empty_weight_kg']:.1f}",      "kg")
    _row("  ↳ fraction of MTOW",       f"{w['empty_weight_fraction']:.3f}")
    _row("Payload",                    f"{w['payload_kg']:.1f}",           "kg")
    _row("Fuel",                       f"{w['fuel_kg']:.1f}",              "kg")
    print(f"  {'─' * 50}")
    _row("MTOW",                       f"{w['mtow_kg']:.1f}",              "kg")

    # ── Geometry ─────────────────────────────────────────────────────────────
    _section("DERIVED GEOMETRY")
    _row("Wing area",                  f"{w['wing_area_m2']:.2f}",         "m²")
    _row("Aspect ratio",               f"{w['aspect_ratio']:.2f}")
    _row("Wing loading",               f"{cfg.mtow_kg * G0_M_S2 / w['wing_area_m2']:.0f}", "N/m²")
    _row("Fuselage length",            f"{w['fuselage_length_m']:.2f}",    "m")
    _row("Fuselage diameter",          f"{w['fuselage_diameter_m']:.2f}",  "m")

    # ── Aerodynamics ─────────────────────────────────────────────────────────
    _section("AERODYNAMICS AT CRUISE")
    aero = summ["aerodynamics"]
    _row("CL (cruise)",                f"{aero['cl_cruise']:.4f}")
    _row("CD₀ (parasite)",             f"{aero['cd0_cruise']:.5f}")
    _row("CD (induced)",               f"{aero['cd_induced_cruise']:.5f}")
    _row("CD (total)",                 f"{aero['cd_total_cruise']:.5f}")
    _row("L/D (cruise)",               f"{aero['ld_cruise']:.2f}")
    _row("Best L/D",                   f"{aero['best_ld']:.2f}")
    _row("Speed for best L/D",         f"{ms_to_ktas(aero['best_ld_speed_ms']):.0f}", "KTAS")
    _row("Stall speed (clean, SL, target)",
         f"{cfg.stall_speed_ktas:.0f}",
         "KTAS")

    print()


def print_range_results(breguet: dict, stepped: dict) -> None:
    print(_hdr("RANGE ESTIMATES"))

    _section("BREGUET (analytical, constant conditions)")
    if "error" in breguet:
        print(f"  ERROR: {breguet['error']}")
    else:
        _row("Range",                  f"{breguet['range_nm']:.0f}",   "nm")
        _row("Range",                  f"{breguet['range_km']:.0f}",   "km")
        _row("Endurance",              f"{breguet['endurance_hr']:.1f}", "hr")
        _row("L/D (mid-cruise)",       f"{breguet['ld_mid_cruise']:.2f}")
        _row("Propulsive efficiency",  f"{breguet['propulsive_efficiency_mid']*100:.1f}", "%")

    _section("STEP INTEGRATION (constant altitude)")
    _row("Range",                      f"{stepped['range_nm']:.0f}",   "nm")
    _row("Range",                      f"{stepped['range_km']:.0f}",   "km")
    _row("Endurance",                  f"{stepped['endurance_hr']:.1f}", "hr")
    _row("Fuel burned",                f"{stepped['fuel_burned_kg']:.1f}", "kg")
    _row("Avg L/D",                    f"{stepped['avg_ld']:.2f}")
    _row("Avg propulsive efficiency",  f"{stepped['avg_propulsive_efficiency']*100:.1f}", "%")
    print()

    # Cruise log table
    log = stepped.get("log", [])
    if log:
        _section("CRUISE LOG (every 10 min)")
        hdr = f"  {'Time':>6}  {'Range':>7}  {'Alt':>7}  {'Weight':>7}  {'Fuel left':>9}  {'Speed':>6}  {'L/D':>6}  {'Power':>7}"
        print(hdr)
        print(f"  {'min':>6}  {'nm':>7}  {'ft':>7}  {'kg':>7}  {'kg':>9}  {'ktas':>6}  {'':>6}  {'kW':>7}")
        print(f"  {'-'*6}  {'-'*7}  {'-'*7}  {'-'*7}  {'-'*9}  {'-'*6}  {'-'*6}  {'-'*7}")
        for row in log:
            print(
                f"  {row['time_min']:6.0f}  "
                f"{row['range_nm']:7.0f}  "
                f"{row['altitude_ft']:7.0f}  "
                f"{row['weight_kg']:7.1f}  "
                f"{row['fuel_remaining_kg']:9.1f}  "
                f"{row['v_ktas']:6.0f}  "
                f"{row['ld']:6.2f}  "
                f"{row['power_kw']:7.1f}"
            )
    print()


def print_warnings(warnings: list[str]) -> None:
    if warnings:
        print(_hdr("WARNINGS"))
        for w in warnings:
            print(f"  ⚠  {w}")
        print()


def print_sensitivity(param: str, results: list[dict]) -> None:
    param_label = param.replace("_", " ").title()
    print(_hdr(f"SENSITIVITY: {param_label.upper()}"))
    print()
    hdr = f"  {param_label:<18}  {'MTOW':>7}  {'Empty W':>7}  {'LG leg':>6}  {'Wing S':>6}  {'AR':>5}  {'L/D':>6}  {'Range':>7}  {'Endur':>6}"
    print(hdr)
    print(f"  {'':18}  {'kg':>7}  {'kg':>7}  {'m':>6}  {'m²':>6}  {'':>5}  {'':>6}  {'nm':>7}  {'hr':>6}")
    print(f"  {'-'*18}  {'-'*7}  {'-'*7}  {'-'*6}  {'-'*6}  {'-'*5}  {'-'*6}  {'-'*7}  {'-'*6}")
    for r in results:
        lg = r.get("landing_gear_leg_length_m", float("nan"))
        ew = r.get("empty_weight_kg", float("nan"))
        sa = r.get("wing_area_m2", float("nan"))
        ar = r.get("aspect_ratio", float("nan"))
        print(
            f"  {r['param_value']:<18.3g}  "
            f"{r['weight_initial_kg']:7.1f}  "
            f"{ew:7.1f}  "
            f"{lg:6.2f}  "
            f"{sa:6.2f}  "
            f"{ar:5.1f}  "
            f"{r['avg_ld']:6.2f}  "
            f"{r['range_nm']:7.0f}  "
            f"{r['endurance_hr']:6.1f}"
        )
    print()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rangesim",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Propulsion
    g_prop = p.add_argument_group("Propulsion")
    g_prop.add_argument("--fan-diameter", type=float, default=1.60, metavar="M",
                        help="Counter-rotating fan tip-to-tip diameter (m) [default: 1.60]")
    g_prop.add_argument("--power", type=float, default=220.0, metavar="KW",
                        help="Engine max shaft power at sea level (kW) [default: 220]")

    # Airframe
    g_air = p.add_argument_group("Airframe")
    g_air.add_argument("--wingspan", type=float, default=10.0, metavar="M",
                       help="Wing tip-to-tip span (m) [default: 10.0]")
    g_air.add_argument("--cabin-width", type=float, default=1.10, metavar="M",
                       help="Interior cabin width (m) [default: 1.10]")

    # Loading
    g_load = p.add_argument_group("Loading")
    g_load.add_argument("--fuel", type=float, default=160.0, metavar="KG",
                        help="Usable fuel loaded (kg) [default: 160]")

    # Cruise
    g_cruise = p.add_argument_group("Cruise conditions")
    g_cruise.add_argument("--speed", type=float, default=210.0, metavar="KTAS",
                          help="Cruise true airspeed (knots) [default: 210]")
    g_cruise.add_argument("--altitude", type=float, default=20_000.0, metavar="FT",
                          help="Cruise pressure altitude (feet) [default: 20000]")
    g_cruise.add_argument("--stall-speed", type=float, default=65.0, metavar="KTAS",
                          help="Target clean stall speed at SL, MTOW (knots) — drives wing sizing [default: 65]")

    # Construction material
    g_mat = p.add_argument_group("Construction material")
    g_mat.add_argument(
        "--material",
        choices=["printed_ti_al", "cfrp", "aluminium"],
        default="printed_ti_al",
        help=(
            "Primary structural material: "
            "printed_ti_al = 3D-printed Ti-6Al-4V + graded Al (default, factor 0.80); "
            "cfrp = carbon-fibre composite (factor 0.75); "
            "aluminium = conventional machined Al (factor 1.00)"
        ),
    )

    # Simulation options
    g_sim = p.add_argument_group("Simulation options")
    g_sim.add_argument("--strategy", choices=["constant_altitude", "cruise_climb"],
                       default="constant_altitude",
                       help="Cruise strategy [default: constant_altitude]")
    g_sim.add_argument("--dt", type=float, default=60.0, metavar="SEC",
                       help="Integration time step (s) [default: 60]")

    # Sensitivity sweep
    g_sens = p.add_argument_group("Sensitivity sweep")
    g_sens.add_argument("--sensitivity", nargs="+", metavar="PARAM VAL ...",
                        help=(
                            "Run a sensitivity sweep. First argument is the "
                            "parameter name (fan_diameter_m, wingspan_m, "
                            "engine_power_kw, fuel_mass_kg, cabin_width_m, "
                            "cruise_speed_ktas, cruise_altitude_ft); "
                            "remaining arguments are numeric values to test.\n"
                            "Example: --sensitivity fan_diameter_m 1.2 1.4 1.6 1.8 2.0"
                        ))
    g_sens.add_argument("--no-log", action="store_true",
                        help="Suppress the per-step cruise log table")

    return p


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    # Build configuration
    material_factors = {"printed_ti_al": 0.80, "cfrp": 0.75, "aluminium": 1.00}
    structural_factor = material_factors[args.material]

    cfg = AircraftConfig(
        fan_diameter_m=args.fan_diameter,
        engine_power_kw=args.power,
        wingspan_m=args.wingspan,
        cabin_width_m=args.cabin_width,
        fuel_mass_kg=args.fuel,
        cruise_speed_ktas=args.speed,
        cruise_altitude_ft=args.altitude,
        stall_speed_ktas=args.stall_speed,
        structural_factor=structural_factor,
    )

    print()
    print(DOUBLE_SEP)
    print("  rangesim — Two-Seat Canard Counter-Rotating Pusher-Propfan")
    print(DOUBLE_SEP)

    # Aircraft summary
    print_aircraft_summary(cfg)

    # Warnings
    warnings = cfg.validate()
    print_warnings(warnings)

    # Range simulation
    breguet = breguet_range_nm(cfg)
    stepped = step_simulation(cfg, strategy=args.strategy, dt_s=args.dt)

    if args.no_log:
        stepped.pop("log", None)

    print_range_results(breguet, stepped)

    # Sensitivity analysis
    if args.sensitivity:
        tokens = args.sensitivity
        param = tokens[0]
        valid_params = {
            "fan_diameter_m", "wingspan_m", "engine_power_kw",
            "fuel_mass_kg", "cabin_width_m", "cruise_speed_ktas",
            "cruise_altitude_ft",
        }
        if param not in valid_params:
            print(f"Unknown sensitivity parameter '{param}'.")
            print(f"Valid options: {', '.join(sorted(valid_params))}")
            return 1
        try:
            values = [float(v) for v in tokens[1:]]
        except ValueError as e:
            print(f"Non-numeric value in sensitivity list: {e}")
            return 1
        if len(values) < 2:
            print("Provide at least two values for a sensitivity sweep.")
            return 1

        sens_results = sensitivity_table(cfg, param, values, strategy=args.strategy)
        print_sensitivity(param, sens_results)

    print(DOUBLE_SEP)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
