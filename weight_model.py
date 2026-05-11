"""
Component weight estimation for the canard pusher-propfan aircraft.

Construction materials
----------------------
Primary structure: 3D-printed (additive manufactured) titanium alloy Ti-6Al-4V
with topology-optimised internal lattice, transitioning to graded aluminium
alloys (e.g. 7075 → 6061) in lower-stress regions and at lower temperatures.

Weight impact vs. baselines:
  - Conventional aluminium machined:  factor 1.00  (Torenbeek calibration basis)
  - CFRP composite lay-up:            factor 0.75  (−25 %; high labour, UV-stable)
  - 3D-printed Ti + graded Al:        factor 0.80  (−20 %; topology-optimised Ti
    has specific strength ≈ CFRP in compression; Al fills cooler zones)
    Ti-6Al-4V yield 880 MPa / 4507 kg/m³ → specific strength ≈ Al 7075, but
    additive topology optimisation saves ~35 % mass over machined Al equivalent;
    blending in Al where Ti is over-specified recovers ≈ 5 % more → net 0.80.

MTOW iteration
--------------
Landing-gear weight and wing sizing depend on MTOW, so we iterate:

    MTOW_0 = initial guess
    loop:
        compute all component weights
        MTOW_new = W_empty + W_payload + W_fuel
        if |MTOW_new - MTOW_0| < tol: break
        MTOW_0 = MTOW_new
"""

import math


# ── Fixed system weights ──────────────────────────────────────────────────
AVIONICS_KG = 25.0        # IFR glass cockpit for two-seater
ELECTRICAL_KG = 15.0      # wiring, alternator, battery
FURNISHINGS_KG = 20.0     # seats, upholstery, belts, misc
PITOT_STATIC_ETC_KG = 5.0

# ── Occupant / payload model ───────────────────────────────────────────────
OCCUPANT_MASS_KG = 90.0          # FAA standard (pilot + passenger)
BAGGAGE_PER_OCCUPANT_KG = 20.0   # behind seats

# Minimum cabin width to fit two abreast:
MIN_CABIN_WIDTH_M = 0.85
# Reference cabin width for two occupants (standard two-seater):
REF_CABIN_WIDTH_M = 1.10

# ── Wing / canard parameters ───────────────────────────────────────────────
# Canard is ~25 % of wing area; share of lifting-surface weight accordingly.
CANARD_WING_WEIGHT_FRACTION = 0.30
# Wing weight uses the Torenbeek formula (calibrated to GA/light aircraft) with
# a composite-construction discount factor.
#   W_wing_lb = 0.0051 × (W_dg·n_z)^0.557 × S_w^0.649 × AR^0.5
#               × (t/c)^-0.4 × (1+λ)^0.1 × S_csw^0.1
# Composite discount vs. aluminium baseline: 0.75
N_ULT = 5.7           # ultimate load factor (3.8 g limit × 1.5 safety)
TC_RATIO = 0.14        # wing thickness-to-chord ratio
TAPER_RATIO = 0.40     # planform taper ratio
# Structural weight factor relative to the Torenbeek conventional-Al baseline.
# Applied to all primary airframe structure (wing, canard, fuselage, landing gear).
# Engine, propfan, avionics, fuel system weights are material-independent.
STRUCTURAL_FACTOR_CFRP       = 0.75   # carbon-fibre composite lay-up
STRUCTURAL_FACTOR_PRINTED_TI = 0.80   # 3D-printed Ti-6Al-4V + graded Al (default)
STRUCTURAL_FACTOR_ALUMINIUM  = 1.00   # conventional machined aluminium baseline

# Default: 3D-printed titanium with graded aluminium
DEFAULT_STRUCTURAL_FACTOR = STRUCTURAL_FACTOR_PRINTED_TI

# Ti / Al mass split within primary airframe structure for each material choice.
# Primary structure = wing + canard + fuselage + landing gear.
# "printed_ti_al": mostly Ti (high-stress spars, frames, gear) with graded Al skins/brackets.
# Key: (ti_fraction, al_fraction)  — must sum to ≤ 1.0; remainder is other (adhesive, seals…)
TI_AL_SPLIT = {
    STRUCTURAL_FACTOR_PRINTED_TI: (0.65, 0.35),   # 65 % Ti-6Al-4V, 35 % graded Al
    STRUCTURAL_FACTOR_CFRP:       (0.00, 0.00),   # CFRP — no Ti or Al in primary structure
    STRUCTURAL_FACTOR_ALUMINIUM:  (0.00, 1.00),   # all Al
}

# Wing is sized to meet a sea-level stall speed requirement (CL_max canard-limited)
CL_MAX_CANARD = 1.00   # canard stalls first; main wing reference CL_max ≈ 1.0

# ── Fuselage ─────────────────────────────────────────────────────────────────
# W_fuse = k * l_fuse * (d_fuse)^1.5  (semi-empirical)
# Calibrated to VariEze-class: l=5.5m, d=1.0m → ~90 kg fuselage structure
K_FUSELAGE = 14.0    # kg / (m × m^1.5)

# ── Landing gear ────────────────────────────────────────────────────────────
# Base fraction of MTOW for fixed composite gear (no retraction mechanism)
LG_BASE_FRACTION = 0.030
# Extra mass per metre of extra gear leg length beyond the reference height
LG_EXTRA_KG_PER_M = 12.0      # per leg; aircraft has 3 legs (nose + 2 main)
LG_NUM_LEGS = 3
# Reference gear leg length (short/conventional canard): 0.30 m
LG_REF_LEG_LENGTH_M = 0.30
# Minimum prop ground clearance required (FAA 14 CFR 23.925: 7 in for landplanes)
PROP_GROUND_CLEARANCE_M = 0.20   # we use 20 cm — slightly generous

# ── Fuel system ──────────────────────────────────────────────────────────────
# Tank structure, sealant, caps, fuel lines, pumps
FUEL_SYSTEM_FRACTION = 0.055    # fraction of fuel mass

# ── Thrust frame ────────────────────────────────────────────────────────────────
# Composite aft-fuselage frame that carries propfan reaction loads into the
# wing box.  Max static thrust from actuator-disk: T = (2·ρ₀·A·P²)^(1/3).
# Scales from ~7 kg at 220 kW to ~49 kg at 3 MW (2 m fan).
K_THRUST_FRAME_KG_N = 0.0012   # kg per N of max static thrust


# ═══════════════════════════════════════════════════════════════════════════════


def _wing_weight_kg(
    wing_area_m2: float, aspect_ratio: float, mtow_kg: float
) -> float:
    """
    Torenbeek wing structural weight, corrected for composite construction.
    Calibrated to data from Long-EZ / Velocity / similar composite canards.
    """
    mtow_lb = mtow_kg * 2.20462
    S_ft2 = wing_area_m2 * 10.7639
    S_csw_ft2 = S_ft2 * 0.20   # control surfaces ≈ 20 % of wing area
    W_lb = (
        0.0051
        * (mtow_lb * N_ULT) ** 0.557
        * S_ft2 ** 0.649
        * aspect_ratio ** 0.5
        * TC_RATIO ** -0.4
        * (1.0 + TAPER_RATIO) ** 0.1
        * S_csw_ft2 ** 0.1
    )
    return W_lb * 0.453592   # → kg, material factor applied by caller


def _fuselage_weight_kg(fuselage_length_m: float, cabin_width_m: float) -> float:
    """Fuselage structural weight."""
    # Effective fuselage diameter is slightly larger than cabin width
    d_fuse = cabin_width_m * 1.20   # 20 % for structure around the cabin
    return K_FUSELAGE * fuselage_length_m * d_fuse ** 1.5


def _landing_gear_weight_kg(
    mtow_kg: float,
    fan_diameter_m: float,
    fuselage_bottom_height_m: float,
) -> float:
    """
    Landing gear weight accounting for extended leg length when the pusher
    propfan diameter exceeds the fuselage clearance.

    The propfan sits at the tail of the fuselage.  When the aircraft is on
    the ground in the three-point attitude the lowest blade tip must clear
    the ground by PROP_GROUND_CLEARANCE_M.

    fan_center_height_m ≈ fuselage_bottom_height_m + d_fuse/2
    required_height = fan_diameter_m/2 + PROP_GROUND_CLEARANCE_M
    extra = max(0, required_height - fan_center_height_m)
    """
    fan_radius = fan_diameter_m / 2.0
    # With a pusher at the fuselage tail, fan center is roughly at the
    # fuselage centreline height above ground
    fan_center_h = fuselage_bottom_height_m    # approx centreplane height
    required_fan_h = fan_radius + PROP_GROUND_CLEARANCE_M
    extra_leg_length = max(0.0, required_fan_h - fan_center_h - LG_REF_LEG_LENGTH_M)

    base_weight = mtow_kg * LG_BASE_FRACTION
    extra_weight = extra_leg_length * LG_EXTRA_KG_PER_M * LG_NUM_LEGS
    return base_weight + extra_weight


def _landing_gear_leg_length_m(
    fan_diameter_m: float,
    fuselage_bottom_height_m: float,
) -> float:
    fan_radius = fan_diameter_m / 2.0
    fan_center_h = fuselage_bottom_height_m
    required_fan_h = fan_radius + PROP_GROUND_CLEARANCE_M
    extra = max(0.0, required_fan_h - fan_center_h - LG_REF_LEG_LENGTH_M)
    return LG_REF_LEG_LENGTH_M + extra


def _payload_kg(cabin_width_m: float) -> float:
    """
    Two-seater: payload capacity scales with cabin width relative to reference.
    Wider cabin → larger seats, potentially heavier passengers + more baggage.
    Below MIN_CABIN_WIDTH_M the second seat is impractical.
    """
    if cabin_width_m < MIN_CABIN_WIDTH_M:
        # Single-seat configuration
        n_occupants = 1
    else:
        n_occupants = 2
    # Width factor: slightly more payload per occupant in a wider cabin
    width_scale = min(1.0 + 0.5 * (cabin_width_m - REF_CABIN_WIDTH_M), 1.15)
    return n_occupants * (OCCUPANT_MASS_KG + BAGGAGE_PER_OCCUPANT_KG) * width_scale


# ═══════════════════════════════════════════════════════════════════════════════


class WeightModel:
    """
    Iterative component weight build-up.

    Parameters come from AircraftConfig; call `.compute()` which returns
    a WeightBreakdown named dict.
    """

    def __init__(
        self,
        # Geometry
        wingspan_m: float,
        cabin_width_m: float,
        fan_diameter_m: float,
        # Performance / sizing
        stall_speed_ktas: float,   # target sea-level stall speed (clean)
        cruise_speed_ms: float,    # cruise TAS — used for altitude CL constraint
        # Masses (user-set)
        fuel_mass_kg: float,
        # Sub-models
        engine_model,
        propfan_model,
        # Material
        structural_factor: float = DEFAULT_STRUCTURAL_FACTOR,
    ):
        self.wingspan_m = wingspan_m
        self.cabin_width_m = cabin_width_m
        self.fan_diameter_m = fan_diameter_m
        self.stall_speed_ktas = stall_speed_ktas
        self.cruise_speed_ms = cruise_speed_ms
        self.fuel_mass_kg = fuel_mass_kg
        self.engine = engine_model
        self.propfan = propfan_model
        self.structural_factor = structural_factor

    def compute(
        self,
        density_kg_m3: float,
        max_iter: int = 40,
        tol_kg: float = 0.5,
    ) -> dict:
        """
        Iterate until MTOW converges.  Returns a full weight breakdown dict.
        """
        from atmosphere import G0_M_S2

        # Derived geometry (independent of MTOW)
        payload_kg = _payload_kg(self.cabin_width_m)

        # Fuselage geometry — depends on cabin width
        d_fuse = self.cabin_width_m * 1.20
        # Fuselage length: 2-seat canard is compact, ~4.5–6 m
        l_fuse = max(4.5, 2.5 + self.cabin_width_m * 2.8 + self.fan_diameter_m * 0.8)
        fuselage_bottom_h = d_fuse / 2.0   # centreline ≈ radius above ground

        sf = self.structural_factor  # applied to all primary airframe structure

        # Fixed weight items (structural ones scaled by material factor)
        w_engine = self.engine.total_powerplant_weight_kg()
        w_propfan = self.propfan.weight_kg(shaft_power_kw=self.engine.max_power_kw)
        w_fuse = _fuselage_weight_kg(l_fuse, self.cabin_width_m) * sf
        w_fuel_sys = self.fuel_mass_kg * FUEL_SYSTEM_FRACTION
        w_fixed = (
            AVIONICS_KG + ELECTRICAL_KG + FURNISHINGS_KG + PITOT_STATIC_ETC_KG
        )

        from atmosphere import RHO0_KG_M3, ktas_to_ms
        V_stall_ms = ktas_to_ms(self.stall_speed_ktas)

        # Thrust frame: aft-fuselage composite frame reacting propfan thrust.
        # T_static from actuator-disk theory: T = (2·ρ₀·A_disk·P²)^(1/3)
        P_w = self.engine.max_power_kw * 1000.0
        T_static = (2.0 * RHO0_KG_M3 * self.propfan.disk_area_m2 * P_w ** 2) ** (1.0 / 3.0)
        w_thrust_frame = K_THRUST_FRAME_KG_N * T_static

        # Cruise CL limit: never exceed 85 % of CL_max at cruise altitude.
        # This sizes the wing larger when flying high and slow.
        CL_CRUISE_LIMIT = CL_MAX_CANARD * 0.85
        V_cruise = self.cruise_speed_ms

        # Initial MTOW guess (typical for this class)
        mtow_kg = 750.0
        converged = False

        for _ in range(max_iter):
            # Wing sized by the LARGER of two constraints:
            #   1. Sea-level stall speed
            #   2. Cruise CL margin at cruise altitude (CL_cruise ≤ 0.85 × CL_max)
            S_stall  = mtow_kg * G0_M_S2 / (0.5 * RHO0_KG_M3 * V_stall_ms ** 2 * CL_MAX_CANARD)
            S_cruise = mtow_kg * G0_M_S2 / (0.5 * density_kg_m3 * V_cruise ** 2 * CL_CRUISE_LIMIT)
            S_wing   = max(S_stall, S_cruise)
            sizing_driver = "altitude" if S_cruise > S_stall else "stall"
            AR = self.wingspan_m ** 2 / S_wing

            w_wing = _wing_weight_kg(S_wing, AR, mtow_kg) * sf
            w_canard = w_wing * CANARD_WING_WEIGHT_FRACTION
            w_lg = _landing_gear_weight_kg(
                mtow_kg, self.fan_diameter_m, fuselage_bottom_h
            ) * sf

            w_empty = (
                w_wing + w_canard + w_fuse + w_lg +
                w_engine + w_thrust_frame + w_propfan + w_fuel_sys + w_fixed
            )
            mtow_new = w_empty + payload_kg + self.fuel_mass_kg

            if abs(mtow_new - mtow_kg) < tol_kg:
                mtow_kg = mtow_new
                converged = True
                break
            mtow_kg = 0.6 * mtow_new + 0.4 * mtow_kg  # damped update

        leg_length = _landing_gear_leg_length_m(self.fan_diameter_m, fuselage_bottom_h)

        # ── Ti / Al mass breakdown ───────────────────────────────
        w_primary_structure = w_wing + w_canard + w_fuse + w_lg
        ti_frac, al_frac = TI_AL_SPLIT.get(self.structural_factor, (0.0, 0.0))
        ti_mass_kg = w_primary_structure * ti_frac
        al_mass_kg = w_primary_structure * al_frac

        # ── Dry mass (no fuel, no payload) ───────────────────────────
        dry_mass_kg = w_empty   # alias for clarity

        return {
            # ── Component breakdown ──────────────────────────────────────────────
            "wing_kg": w_wing,
            "canard_kg": w_canard,
            "fuselage_kg": w_fuse,
            "landing_gear_kg": w_lg,
            "engine_kg": self.engine.dry_weight_kg(),
            "engine_install_kg": self.engine.installation_weight_kg() + w_thrust_frame,
            "propfan_kg": w_propfan,
            "fuel_system_kg": w_fuel_sys,
            "avionics_electrical_kg": AVIONICS_KG + ELECTRICAL_KG,
            "furnishings_kg": FURNISHINGS_KG + PITOT_STATIC_ETC_KG,
            "empty_weight_kg": w_empty,
            "dry_mass_kg": dry_mass_kg,
            # ── Material breakdown ──────────────────────────────────────────────
            "primary_structure_kg": w_primary_structure,
            "titanium_mass_kg": ti_mass_kg,
            "aluminum_mass_kg": al_mass_kg,
            # ── Useful load ─────────────────────────────────────────────────────────────
            "payload_kg": payload_kg,
            "fuel_kg": self.fuel_mass_kg,
            # ── Totals ───────────────────────────────────────────────────────────────────
            "mtow_kg": mtow_kg,
            # ── Geometry ─────────────────────────────────────────────────────────────────
            "wing_area_m2": S_wing,
            "aspect_ratio": AR,
            "fuselage_length_m": l_fuse,
            "fuselage_diameter_m": d_fuse,
            "landing_gear_leg_length_m": leg_length,
            # ── Diagnostics ─────────────────────────────────────────────────────────────
            "converged": converged,
            "empty_weight_fraction": w_empty / mtow_kg,
            "structural_factor": self.structural_factor,
            "wing_sizing_driver": sizing_driver,   # "stall" or "altitude"
        }
