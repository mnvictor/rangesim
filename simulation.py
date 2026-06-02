"""
Range simulation — Breguet estimate + step integration + full mission sim.

Three methods:

1. breguet_range_nm()   Closed-form Breguet equation (no reserves).
2. step_simulation()    Numeric cruise integration (no reserves, max range).
3. mission_simulation() Full mission: takeoff → climb → cruise → descent.
                        Holds a 45-min fuel reserve and models all phases.

Cruise strategy options (applies to step_simulation and cruise phase of mission)
-----------------------
  "constant_altitude"  : hold altitude and speed; power reduces as weight falls
  "cruise_climb"       : hold constant IAS; altitude rises as aircraft lightens
"""

# ── Mission simulation constants ────────────────────────────────────────────
RESERVE_MIN         = 45.0   # FAR 91 IFR: 45-min cruise fuel reserve
TAKEOFF_MIN         = 3.0    # taxi + ground roll + climbout to 400 ft AGL
CLIMB_POWER_FRAC    = 0.90   # fraction of SL max power used for climb
_LOG_PHASE_INTERVAL = 120.0  # log every 2 min during climb / descent (s)

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

    # Breguet numerator: specific air range factor
    # R = (η_prop · η_th · LHV / g) · (L/D) · ln(Wi/Wf)
    range_m = cfg.engine.range_factor(eta_prop) * ld * math.log(W_initial_n / W_final_n)

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


def mission_simulation(
    aircraft_config,
    strategy: str = "constant_altitude",
    dt_cruise_s: float = 60.0,
    dt_phase_s: float = 30.0,
) -> dict:
    """
    Full-mission range simulation.

    Phases
    ------
    0. Takeoff  : TAKEOFF_MIN min at full SL power, no horizontal distance.
    1. Climb    : constant-IAS climb at CLIMB_POWER_FRAC of SL power.
    2. Cruise   : step integration on (total_fuel - takeoff - climb - reserve).
    3. Descent  : idle-power descent back to sea level; range credit counted.
    4. Landing  : 1 min idle at sea level (taxiing in).

    Returns dict with phase fuel breakdown, range, and chart log.
    """
    cfg = aircraft_config

    atm_sl  = get_atmosphere(0.0)
    rho_sl  = atm_sl["density_kg_m3"]

    cruise_alt_m = ft_to_m(cfg.cruise_altitude_ft)
    atm_cr  = get_atmosphere(cruise_alt_m)
    rho_cr  = atm_cr["density_kg_m3"]
    sos_cr  = atm_cr["speed_of_sound_m_s"]
    nu_cr   = atm_cr["kinematic_viscosity_m2_s"]

    # Calibrated airspeed used for climb / descent
    # (same IAS as cruise; TAS varies with density during climb)
    v_ias_ms = cfg.cruise_speed_ms * math.sqrt(rho_cr / rho_sl)
    v_ias_ms = max(v_ias_ms, ktas_to_ms(70.0))   # 70 KIAS floor

    fuel_remaining = cfg.fuel_mass_kg
    weight_kg      = cfg.mtow_kg
    time_s         = 0.0
    range_m        = 0.0
    log: list[dict] = []

    def _log(phase: str, alt_m: float, v_ms: float) -> None:
        log.append({
            "time_min":    round(time_s / 60.0, 2),
            "range_nm":    round(range_m * NM_PER_METRE, 2),
            "altitude_ft": round(m_to_ft(alt_m), 0),
            "v_ktas":      round(ms_to_ktas(v_ms), 1),
            "phase":       phase,
        })

    # ── Phase 0: Takeoff ────────────────────────────────────────────────────
    _log("takeoff", 0.0, v_ias_ms)
    ff_to      = cfg.engine.fuel_flow_kg_s(cfg.engine.max_power_kw)
    fuel_to    = min(ff_to * TAKEOFF_MIN * 60.0, fuel_remaining)
    fuel_remaining -= fuel_to
    weight_kg      -= fuel_to
    time_s         += TAKEOFF_MIN * 60.0
    _log("takeoff", ft_to_m(400.0), v_ias_ms)

    # ── Phase 1: Climb ──────────────────────────────────────────────────────
    alt_m           = ft_to_m(400.0)
    last_phase_log  = time_s
    fuel_after_takeoff = fuel_remaining          # checkpoint

    while alt_m < cruise_alt_m - 1.0 and fuel_remaining > 0.0:
        atm  = get_atmosphere(alt_m)
        rho  = atm["density_kg_m3"]
        sos  = atm["speed_of_sound_m_s"]
        nu   = atm["kinematic_viscosity_m2_s"]

        # TAS increases with altitude at constant IAS
        v_ms   = v_ias_ms * math.sqrt(rho_sl / rho)
        W_n    = weight_kg * G0_M_S2
        drag_n = cfg.aero.drag_n(W_n, v_ms, rho, nu)

        p_climb = min(
            cfg.engine.max_power_kw * CLIMB_POWER_FRAC,
            cfg.engine.max_power_at_altitude_kw(rho),
        )
        thrust_n = cfg.propfan.max_thrust_n(v_ms, p_climb * 1000.0, rho, sos)

        # Rate of climb from excess thrust
        roc_ms = v_ms * max(0.0, thrust_n - drag_n) / W_n
        roc_ms = min(roc_ms, 15.0)      # cap at ~3 000 fpm
        if roc_ms < 0.25:               # service ceiling
            alt_m = cruise_alt_m
            break

        dh_left  = cruise_alt_m - alt_m
        dt_step  = min(dt_phase_s, dh_left / roc_ms)
        fuel_step = min(cfg.engine.fuel_flow_kg_s(p_climb) * dt_step, fuel_remaining)
        horiz_v  = math.sqrt(max(0.0, v_ms**2 - roc_ms**2))

        range_m        += horiz_v  * dt_step
        time_s         += dt_step
        alt_m          += roc_ms   * dt_step
        fuel_remaining -= fuel_step
        weight_kg      -= fuel_step

        if time_s - last_phase_log >= _LOG_PHASE_INTERVAL or alt_m >= cruise_alt_m - 1.0:
            _log("climb", min(alt_m, cruise_alt_m), v_ms)
            last_phase_log = time_s

    alt_m = cruise_alt_m
    fuel_after_climb   = fuel_remaining           # checkpoint
    climb_fuel_kg      = fuel_after_takeoff - fuel_after_climb

    # ── Reserve fuel (sized at post-climb weight / cruise conditions) ────────
    W_n_cr   = weight_kg * G0_M_S2
    drag_cr  = cfg.aero.drag_n(W_n_cr, cfg.cruise_speed_ms, rho_cr, nu_cr)
    p_cr_kw  = cfg.propfan.shaft_power_w(cfg.cruise_speed_ms, drag_cr, rho_cr, sos_cr) / 1000.0
    p_cr_kw  = min(p_cr_kw, cfg.engine.max_power_at_altitude_kw(rho_cr))
    reserve_kg = cfg.engine.fuel_flow_kg_s(p_cr_kw) * RESERVE_MIN * 60.0

    fuel_for_cruise = max(0.0, fuel_remaining - reserve_kg)

    # ── Phase 2: Cruise ─────────────────────────────────────────────────────
    alt_ft          = cfg.cruise_altitude_ft
    fuel_cruise_burned = 0.0
    ld_values:  list[float] = []
    eta_values: list[float] = []
    last_cruise_log = time_s - 600.0   # force first cruise log immediately
    v_ms = cfg.cruise_speed_ms         # fallback if loop body never runs

    _log("cruise", cruise_alt_m, v_ms)

    while fuel_cruise_burned < fuel_for_cruise:
        alt_m_c = ft_to_m(alt_ft)
        atm_c   = get_atmosphere(alt_m_c)
        rho_c   = atm_c["density_kg_m3"]
        sos_c   = atm_c["speed_of_sound_m_s"]
        nu_c    = atm_c["kinematic_viscosity_m2_s"]

        if strategy == "cruise_climb":
            q0   = 0.5 * rho_cr * cfg.cruise_speed_ms**2
            v_ms = math.sqrt(2.0 * q0 / rho_c)
        else:
            v_ms = cfg.cruise_speed_ms

        W_n    = weight_kg * G0_M_S2
        drag_n = cfg.aero.drag_n(W_n, v_ms, rho_c, nu_c)
        p_req  = cfg.propfan.shaft_power_w(v_ms, drag_n, rho_c, sos_c) / 1000.0
        p_avail = cfg.engine.max_power_at_altitude_kw(rho_c)
        if p_req > p_avail * 1.02:
            break

        p_act = min(p_req, p_avail)
        ff    = cfg.engine.fuel_flow_kg_s(p_act)
        ld    = cfg.aero.ld_ratio(W_n, v_ms, rho_c, nu_c)
        eta   = cfg.propfan.efficiency(v_ms, drag_n, rho_c, sos_c)
        ld_values.append(ld)
        eta_values.append(eta)

        remaining_cr = fuel_for_cruise - fuel_cruise_burned
        dt_step = min(dt_cruise_s, remaining_cr / max(ff, 1e-9))

        range_m        += v_ms * dt_step
        time_s         += dt_step
        weight_kg      -= ff   * dt_step
        fuel_remaining -= ff   * dt_step
        fuel_cruise_burned += ff * dt_step

        if strategy == "cruise_climb":
            rho_new  = rho_c * (weight_kg / (weight_kg + ff * dt_step))
            rho_new  = max(rho_new, 0.3)
            alt_ft   = m_to_ft(_altitude_for_density(rho_new))

        if time_s - last_cruise_log >= 600.0:
            _log("cruise", ft_to_m(alt_ft), v_ms)
            last_cruise_log = time_s

    _log("cruise", ft_to_m(alt_ft), v_ms)

    # ── Phase 3: Descent ────────────────────────────────────────────────────
    alt_m          = ft_to_m(alt_ft)
    last_phase_log = time_s

    while alt_m > 10.0 and fuel_remaining > 0.0:
        atm  = get_atmosphere(alt_m)
        rho  = atm["density_kg_m3"]
        sos  = atm["speed_of_sound_m_s"]
        nu   = atm["kinematic_viscosity_m2_s"]

        v_ms   = v_ias_ms * math.sqrt(rho_sl / rho)
        W_n    = weight_kg * G0_M_S2
        drag_n = cfg.aero.drag_n(W_n, v_ms, rho, nu)

        p_idle   = cfg.engine.idle_power_kw(rho)
        thrust_i = cfg.propfan.max_thrust_n(v_ms, p_idle * 1000.0, rho, sos)

        # Descent rate: excess drag over idle thrust drives the sink
        rod_ms = v_ms * max(0.0, drag_n - thrust_i) / W_n
        rod_ms = max(rod_ms, max(4.0, v_ms * 0.05))  # ~800 fpm min (spoiler/pitch authority)

        dt_step   = min(dt_phase_s, alt_m / rod_ms)
        fuel_step = min(cfg.engine.fuel_flow_kg_s(p_idle) * dt_step, fuel_remaining)

        range_m        += v_ms   * dt_step
        time_s         += dt_step
        alt_m          -= rod_ms * dt_step
        alt_m           = max(alt_m, 0.0)
        fuel_remaining -= fuel_step
        weight_kg      -= fuel_step

        if time_s - last_phase_log >= _LOG_PHASE_INTERVAL or alt_m <= 10.0:
            _log("descent", alt_m, v_ms)
            last_phase_log = time_s

    _log("landing", 0.0, v_ias_ms)

    avg_ld  = sum(ld_values)  / len(ld_values)  if ld_values  else 0.0
    avg_eta = sum(eta_values) / len(eta_values) if eta_values else 0.0

    return {
        "method":            "mission_simulation",
        "range_nm":          range_m * NM_PER_METRE,
        "range_km":          range_m * KM_PER_METRE,
        "endurance_hr":      time_s / 3600.0,
        "fuel_burned_kg":    cfg.fuel_mass_kg - fuel_remaining,
        "fuel_remaining_kg": fuel_remaining,
        "reserve_kg":        reserve_kg,
        "reserve_min":       RESERVE_MIN,
        "takeoff_fuel_kg":   fuel_to,
        "climb_fuel_kg":     climb_fuel_kg,
        "cruise_fuel_kg":    fuel_cruise_burned,
        "avg_ld":            avg_ld,
        "avg_propulsive_efficiency": avg_eta,
        "log":               log,
    }


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
