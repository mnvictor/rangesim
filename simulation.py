"""
Range simulation — Breguet estimate + step integration.

Two methods are provided:

1. breguet_range()
   Closed-form Breguet range equation.  Assumes constant altitude, speed,
   L/D and propulsive efficiency.  Very fast; useful as a sanity check.

       R = (η_prop · LHV · η_th / g) · (L/D) · ln(W_initial / W_final)

   Units: R in metres (converted to nm for display).

2. step_simulation()
   Numeric integration in small time steps (dt ≈ 60 s).  At each step:
     a. Compute current weight → lift required → drag → thrust required
     b. Look up engine power needed for that thrust via propfan model
     c. Compute fuel burn in dt
     d. Deduct fuel; advance position
   Continues until fuel is exhausted or max range/time exceeded.

   This naturally captures:
     - Gradual weight decrease → rising L/D → improving specific range
     - Altitude and speed changes if desired (constant IAS cruise-climb)
     - Engine power limiting at altitude

Cruise strategy options
-----------------------
  "constant_altitude"  : hold altitude and speed; power reduces as weight falls
  "cruise_climb"       : hold constant IAS (CAS ≈ IAS for moderate altitudes);
                         as weight drops, altitude rises slightly keeping Vcas
                         constant — more realistic for long-range cruise
"""

import math
from atmosphere import (
    get_atmosphere, ft_to_m, m_to_ft, ktas_to_ms, ms_to_ktas, G0_M_S2
)

NM_PER_METRE = 1.0 / 1852.0
KM_PER_METRE = 1.0 / 1000.0


def breguet_range_nm(aircraft_config) -> dict:
    """
    Breguet range equation evaluated at the cruise condition.

    Returns a dict with range (nm, km), endurance (hr), and supporting data.
    """
    cfg = aircraft_config
    atm = cfg.atmosphere
    rho = atm["density_kg_m3"]
    sos = atm["speed_of_sound_m_s"]
    nu = atm["kinematic_viscosity_m2_s"]
    v = cfg.cruise_speed_ms

    W_initial_n = cfg.mtow_kg * G0_M_S2
    W_final_n = (cfg.mtow_kg - cfg.fuel_mass_kg) * G0_M_S2
    if W_final_n <= 0:
        return {"error": "fuel mass exceeds MTOW — impossible configuration"}

    # Use mid-cruise weight for a single-point estimate
    W_mid_n = 0.5 * (W_initial_n + W_final_n)
    drag_n = cfg.aero.drag_n(W_mid_n, v, rho, nu)

    eta_prop = cfg.propfan.efficiency(v, drag_n, rho, sos)
    ld = cfg.aero.ld_ratio(W_mid_n, v, rho, nu)

    from engine import THERMAL_EFFICIENCY, FUEL_LHV_J_KG
    # Breguet numerator: specific air range factor
    # R = (η_prop · η_th · LHV / g) · (L/D) · ln(Wi/Wf)
    range_factor = eta_prop * THERMAL_EFFICIENCY * FUEL_LHV_J_KG / G0_M_S2
    range_m = range_factor * ld * math.log(W_initial_n / W_final_n)

    # Endurance: E = R / V  (approximate; constant speed assumed)
    endurance_s = range_m / v
    endurance_hr = endurance_s / 3600.0

    # Specific air range (nm per kg of fuel, at mid-cruise weight)
    fuel_flow_kg_s = cfg.engine.fuel_flow_kg_s(
        cfg.engine.max_power_at_altitude_kw(rho)  # will be less at cruise; approximate here
    )
    sar_nm_per_kg = (v * NM_PER_METRE) / (fuel_flow_kg_s + 1e-12)

    return {
        "method": "Breguet",
        "range_nm": range_m * NM_PER_METRE,
        "range_km": range_m * KM_PER_METRE,
        "endurance_hr": endurance_hr,
        "ld_mid_cruise": ld,
        "propulsive_efficiency_mid": eta_prop,
        "weight_initial_kg": cfg.mtow_kg,
        "weight_final_kg": cfg.mtow_kg - cfg.fuel_mass_kg,
        "fuel_fraction": cfg.fuel_mass_kg / cfg.mtow_kg,
    }


# ─────────────────────────────────────────────────────────────────────────────


def step_simulation(
    aircraft_config,
    strategy: str = "constant_altitude",
    dt_s: float = 60.0,
    max_time_hr: float = 30.0,
) -> dict:
    """
    Numeric range integration.

    Parameters
    ----------
    strategy     : "constant_altitude" | "cruise_climb"
    dt_s         : time step (seconds)
    max_time_hr  : safety limit on simulation duration

    Returns
    -------
    dict with range/endurance, step log, and performance profile.
    """
    cfg = aircraft_config
    v_ktas = cfg.cruise_speed_ktas
    alt_ft = cfg.cruise_altitude_ft

    # Initial state
    fuel_remaining_kg = cfg.fuel_mass_kg
    weight_kg = cfg.mtow_kg
    range_m = 0.0
    time_s = 0.0
    max_time_s = max_time_hr * 3600.0

    # For cruise-climb we track IAS (calibrated, approximately = IAS at moderate alt)
    v_ms_initial = ktas_to_ms(v_ktas)

    # Logging (every 10 minutes)
    log_interval_s = 600.0
    last_log_s = -log_interval_s
    log: list[dict] = []

    # Track performance stats
    ld_values = []
    eta_values = []
    power_fractions = []

    while fuel_remaining_kg > 0.0 and time_s < max_time_s:
        alt_m = ft_to_m(alt_ft)
        atm = get_atmosphere(alt_m)
        rho = atm["density_kg_m3"]
        sos = atm["speed_of_sound_m_s"]
        nu = atm["kinematic_viscosity_m2_s"]

        if strategy == "cruise_climb":
            # Maintain constant dynamic pressure (constant IAS) → TAS rises with altitude
            # At initial condition: q0 = 0.5 * rho0 * v0²
            rho0_initial = get_atmosphere(ft_to_m(cfg.cruise_altitude_ft))["density_kg_m3"]
            q0 = 0.5 * rho0_initial * v_ms_initial ** 2
            v_ms = math.sqrt(2.0 * q0 / rho)
        else:
            v_ms = v_ms_initial

        W_n = weight_kg * G0_M_S2
        drag_n = cfg.aero.drag_n(W_n, v_ms, rho, nu)

        eta_prop = cfg.propfan.efficiency(v_ms, drag_n, rho, sos)
        # Required shaft power
        P_required_kw = cfg.propfan.shaft_power_w(v_ms, drag_n, rho, sos) / 1000.0
        P_available_kw = cfg.engine.max_power_at_altitude_kw(rho)

        if P_required_kw > P_available_kw * 1.02:
            # Cannot maintain level flight — end simulation
            break

        # Actual power (throttle to what's needed)
        P_actual_kw = min(P_required_kw, P_available_kw)
        fuel_flow_kg_s = cfg.engine.fuel_flow_kg_s(P_actual_kw)

        # Advance
        fuel_burn_kg = fuel_flow_kg_s * dt_s
        if fuel_burn_kg > fuel_remaining_kg:
            # Last partial step — burn whatever is left
            dt_s_final = fuel_remaining_kg / fuel_flow_kg_s
            range_m += v_ms * dt_s_final
            time_s += dt_s_final
            weight_kg -= fuel_remaining_kg
            fuel_remaining_kg = 0.0
            break

        range_m += v_ms * dt_s
        time_s += dt_s
        fuel_remaining_kg -= fuel_burn_kg
        weight_kg -= fuel_burn_kg

        ld = cfg.aero.ld_ratio(W_n, v_ms, rho, nu)
        ld_values.append(ld)
        eta_values.append(eta_prop)
        power_fractions.append(P_actual_kw / P_available_kw)

        # For cruise-climb: rise altitude to stay at same IAS / CL
        if strategy == "cruise_climb":
            # Adjust altitude so CL stays roughly constant as weight drops
            # CL = W / (q · S) → constant CL cruise: q ∝ W → ρ ∝ W
            # New density: ρ_new = ρ_old * (W_new / W_old)
            rho_new = rho * (weight_kg / (weight_kg + fuel_burn_kg))
            rho_new = max(rho_new, 0.3)   # floor for stratosphere limit
            # Invert ISA to find new altitude
            alt_m = _altitude_for_density(rho_new)
            alt_ft = m_to_ft(alt_m)

        # Logging
        if time_s - last_log_s >= log_interval_s:
            last_log_s = time_s
            log.append({
                "time_min": time_s / 60.0,
                "range_nm": range_m * NM_PER_METRE,
                "altitude_ft": alt_ft,
                "weight_kg": weight_kg,
                "fuel_remaining_kg": fuel_remaining_kg,
                "v_ktas": ms_to_ktas(v_ms),
                "ld": ld,
                "eta_prop": eta_prop,
                "power_kw": P_actual_kw,
                "fuel_flow_kg_h": fuel_flow_kg_s * 3600.0,
            })

    endurance_hr = time_s / 3600.0
    avg_ld = sum(ld_values) / len(ld_values) if ld_values else 0.0
    avg_eta = sum(eta_values) / len(eta_values) if eta_values else 0.0

    return {
        "method": f"step_integration ({strategy})",
        "range_nm": range_m * NM_PER_METRE,
        "range_km": range_m * KM_PER_METRE,
        "endurance_hr": endurance_hr,
        "fuel_burned_kg": cfg.fuel_mass_kg - fuel_remaining_kg,
        "fuel_remaining_kg": fuel_remaining_kg,
        "final_altitude_ft": alt_ft,
        "avg_ld": avg_ld,
        "avg_propulsive_efficiency": avg_eta,
        "weight_initial_kg": cfg.mtow_kg,
        "weight_final_kg": weight_kg,
        "log": log,
    }


def _altitude_for_density(target_rho: float, tol: float = 0.0001) -> float:
    """
    Binary search for altitude (m) that yields the given air density.
    Valid from 0 to 20 000 m.
    """
    lo, hi = 0.0, 20_000.0
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        rho = get_atmosphere(mid)["density_kg_m3"]
        if abs(rho - target_rho) < tol:
            return mid
        if rho > target_rho:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def sensitivity_table(
    aircraft_config,
    param_name: str,
    values: list,
    strategy: str = "constant_altitude",
) -> list[dict]:
    """
    Run the step simulation for a list of parameter values and return results.

    param_name : one of the AircraftConfig field names
    values     : list of values to test
    """
    import copy
    results = []
    for val in values:
        cfg_copy = copy.deepcopy(aircraft_config)
        setattr(cfg_copy, param_name, val)
        cfg_copy.reconfigure()
        res = step_simulation(cfg_copy, strategy=strategy)
        res["param_value"] = val
        res.pop("log", None)   # keep table compact
        # Attach weight/geometry details from the config
        w = cfg_copy.weights
        res["empty_weight_kg"] = w["empty_weight_kg"]
        res["mtow_kg"] = w["mtow_kg"]
        res["landing_gear_leg_length_m"] = w["landing_gear_leg_length_m"]
        res["wing_area_m2"] = w["wing_area_m2"]
        res["aspect_ratio"] = w["aspect_ratio"]
        results.append(res)
    return results
