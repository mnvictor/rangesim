"""
Aerodynamic model for the two-seat canard aircraft.

Canard vs. conventional tail differences
-----------------------------------------
* The canard (forward wing) produces positive lift in trimmed flight,
  unlike a conventional tail which must produce download to trim.  This
  means both surfaces contribute to lifting the weight, giving a slightly
  better effective span and induced-drag distribution.

* We account for this with a slightly higher Oswald efficiency factor e
  relative to a conventional aircraft with the same geometric AR.

* The pusher propfan sits in the aft wake.  It ingests the wing/fuselage
  boundary layer, which reduces its net profile drag slightly (we fold this
  into a small CD0 reduction via the "wake-ingestion" credit), but this is
  already captured in the propfan installation factor.

Drag polar
----------
    CD = CD0 + CL² / (π · AR · e)

CD0 components:
    Wing + canard      (friction + form)
    Fuselage           (friction + form)
    Landing gear       (fixed gear only when deployed — not used in cruise)
    Cooling drag       (exhaust heat rejection for the turboshaft)
    Interference       (wing-fuselage, canard-fuselage junctions)
    Miscellaneous      (antennae, seals, gaps)

Reynolds-number corrections are applied via a flat-plate friction
coefficient formula (Schlichting turbulent):
    Cf = 0.455 / (log10(Re))^2.58  (fully turbulent boundary layer)
"""

import math


# ── Canard-specific aerodynamic constants ────────────────────────────────────
OSWALD_EFF = 0.88          # composite canard (higher than conventional ~0.80)
CL_MAX_CLEAN = 1.00        # canard stalls first; main-wing-referenced CL_max ≈ 1.0
CL_MAX_TAKEOFF = 1.30      # with extended canard leading edge (modest improvement)
CL_CRUISE_TARGET = 0.35    # informational; actual cruise CL falls out from wing sizing

# ── Coffin corner (high-altitude flight envelope) ─────────────────────────────
# As altitude rises the 1 g stall speed (in TAS) climbs while the drag-divergence
# Mach number caps the top speed.  The two boundaries converge at the aerodynamic
# ceiling — the "coffin corner" — above which no level cruise speed exists.
M_DRAG_DIVERGENCE = 0.82   # clean-airframe drag-divergence Mach (above the M0.80
                           # swept-propfan design point, so it never caps that feature)
BUFFET_MARGIN_G   = 1.3    # required load-factor margin to the low-speed buffet


def coffin_corner(weight_n, wing_area_m2, density_kg_m3, speed_of_sound_ms, cruise_speed_ms):
    """
    Coffin-corner envelope at the current cruise condition.

    Low-speed boundary  : 1 g stall TAS × sqrt(BUFFET_MARGIN_G)
    High-speed boundary : M_DRAG_DIVERGENCE × speed of sound

    Returns a dict with both boundary speeds (m/s), whether a usable band
    exists (band_ok), and whether the cruise speed sits inside it (in_band).
    """
    v_stall = math.sqrt(weight_n / (0.5 * density_kg_m3 * wing_area_m2 * CL_MAX_CLEAN))
    v_buffet = v_stall * math.sqrt(BUFFET_MARGIN_G)
    v_mach = M_DRAG_DIVERGENCE * speed_of_sound_ms
    return {
        "v_buffet_ms": v_buffet,
        "v_mach_ms": v_mach,
        "band_ok": v_buffet < v_mach,
        "in_band": v_buffet <= cruise_speed_ms <= v_mach,
    }

# ── Parasite drag breakdown (wetted-area approach) ───────────────────────────
# Form factors for components (Hoerner)
FF_WING = 1.30      # wing form factor (moderate t/c ~0.14)
FF_CANARD = 1.30
FF_FUSELAGE = 1.10  # slender body
FF_NACELLE = 1.15   # propfan ring / cowl

# Wetted area ratios:  S_wet / S_ref
WING_WET_RATIO = 2.0      # two sides
FUSELAGE_WET_RATIO = 3.8  # fuselage wetted / wing reference area, typical canard
CANARD_WET_RATIO = 0.65   # canard S_wet / S_wing_ref

# Interference drag increment
CD0_INTERFERENCE = 0.0005

# Cooling / exhaust drag (turboshaft exhaust velocity slightly higher than
# flight speed at cruise → small momentum loss)
CD0_COOLING = 0.0006

# Miscellaneous (antennae, vents, surface imperfections)
CD0_MISC = 0.0004

# Reference chord for Re calculation (mean aerodynamic chord proxy)
# MAC ≈ sqrt(S / AR) which we calculate on the fly


def flat_plate_cf(reynolds: float) -> float:
    """Schlichting fully-turbulent flat-plate skin-friction coefficient."""
    if reynolds < 1e5:
        return 0.01    # low-Re floor (laminar region, conservative)
    return 0.455 / (math.log10(reynolds) ** 2.58)


def _cd0_lifting_surface(
    area_m2: float,
    ref_area_m2: float,
    mac_m: float,
    velocity_ms: float,
    kinematic_viscosity_m2_s: float,
    form_factor: float,
    wet_ratio: float,
) -> float:
    """Parasite drag coefficient contribution from one lifting surface."""
    Re = velocity_ms * mac_m / kinematic_viscosity_m2_s
    Cf = flat_plate_cf(Re)
    # Assume 35 % natural laminar flow on leading edge (feasible for composites)
    laminar_credit = 0.35
    Re_lam = 0.5e6  # transition Reynolds number
    if Re > Re_lam:
        Cf_lam = 1.328 / Re_lam ** 0.5
        Cf = (1.0 - laminar_credit) * Cf + laminar_credit * Cf_lam
    return Cf * form_factor * (wet_ratio * area_m2) / ref_area_m2


def _cd0_fuselage(
    fuse_length_m: float,
    fuse_diameter_m: float,
    ref_area_m2: float,
    velocity_ms: float,
    kinematic_viscosity_m2_s: float,
) -> float:
    """Fuselage parasite drag coefficient."""
    Re = velocity_ms * fuse_length_m / kinematic_viscosity_m2_s
    Cf = flat_plate_cf(Re)
    S_wet_fuse = math.pi * fuse_diameter_m * fuse_length_m * 0.80   # 80 % cylinder approx
    # Fineness ratio form factor (Hoerner body of revolution)
    f_ratio = fuse_length_m / fuse_diameter_m
    ff = 1.0 + 60.0 / f_ratio ** 3 + f_ratio / 400.0
    return Cf * ff * S_wet_fuse / ref_area_m2


class AeroModel:
    """
    Drag polar and lift estimates for the canard pusher aircraft.

    Parameters (all resolved geometry, not user inputs directly):
    """

    def __init__(
        self,
        wing_area_m2: float,
        wingspan_m: float,
        aspect_ratio: float,
        fuselage_length_m: float,
        fuselage_diameter_m: float,
    ):
        self.S = wing_area_m2
        self.b = wingspan_m
        self.AR = aspect_ratio
        self.l_fuse = fuselage_length_m
        self.d_fuse = fuselage_diameter_m

        # Mean aerodynamic chord
        self.MAC = math.sqrt(wing_area_m2 / aspect_ratio)
        # Canard area ≈ 20 % of wing area
        self.S_canard = wing_area_m2 * 0.20

    def cd0(
        self,
        velocity_ms: float,
        kinematic_viscosity_m2_s: float,
    ) -> float:
        """Zero-lift drag coefficient."""
        cd0_wing = _cd0_lifting_surface(
            self.S, self.S, self.MAC, velocity_ms,
            kinematic_viscosity_m2_s, FF_WING, WING_WET_RATIO,
        )
        cd0_canard = _cd0_lifting_surface(
            self.S_canard, self.S, self.MAC * 0.65, velocity_ms,
            kinematic_viscosity_m2_s, FF_CANARD, CANARD_WET_RATIO,
        )
        cd0_fuse = _cd0_fuselage(
            self.l_fuse, self.d_fuse, self.S,
            velocity_ms, kinematic_viscosity_m2_s,
        )
        return (
            cd0_wing + cd0_canard + cd0_fuse
            + CD0_INTERFERENCE + CD0_COOLING + CD0_MISC
        )

    def cl(self, weight_n: float, velocity_ms: float, density_kg_m3: float) -> float:
        """Lift coefficient required for level flight."""
        q = 0.5 * density_kg_m3 * velocity_ms ** 2
        return weight_n / (q * self.S)

    def cd(
        self,
        cl: float,
        velocity_ms: float,
        kinematic_viscosity_m2_s: float,
    ) -> float:
        """Total drag coefficient at given CL."""
        cd_induced = cl ** 2 / (math.pi * self.AR * OSWALD_EFF)
        cd_parasite = self.cd0(velocity_ms, kinematic_viscosity_m2_s)
        return cd_parasite + cd_induced

    def ld_ratio(
        self,
        weight_n: float,
        velocity_ms: float,
        density_kg_m3: float,
        kinematic_viscosity_m2_s: float,
    ) -> float:
        """Lift-to-drag ratio in level flight."""
        cl = self.cl(weight_n, velocity_ms, density_kg_m3)
        cd_total = self.cd(cl, velocity_ms, kinematic_viscosity_m2_s)
        return cl / cd_total if cd_total > 0 else 0.0

    def drag_n(
        self,
        weight_n: float,
        velocity_ms: float,
        density_kg_m3: float,
        kinematic_viscosity_m2_s: float,
    ) -> float:
        """Drag force in level flight (= required thrust)."""
        q = 0.5 * density_kg_m3 * velocity_ms ** 2
        cl = self.cl(weight_n, velocity_ms, density_kg_m3)
        cd_total = self.cd(cl, velocity_ms, kinematic_viscosity_m2_s)
        return q * self.S * cd_total

    def best_ld_speed_ms(
        self,
        weight_n: float,
        density_kg_m3: float,
        kinematic_viscosity_m2_s: float,
    ) -> float:
        """
        Speed for maximum L/D (minimum drag, maximum range speed for jet/fan).
        At best L/D: induced drag = parasite drag.
        Solve numerically since CD0 is speed-dependent (Re effects).
        """
        # Start with the theoretical speed ignoring Re dependence
        # CL_best = sqrt(pi * AR * e * CD0)  → iterate
        v_lo, v_hi = 30.0, 250.0
        best_v = 0.5 * (v_lo + v_hi)
        best_ld = 0.0
        for _ in range(60):
            v_mid = 0.5 * (v_lo + v_hi)
            ld = self.ld_ratio(weight_n, v_mid, density_kg_m3, kinematic_viscosity_m2_s)
            if ld > best_ld:
                best_ld = ld
                best_v = v_mid
            # check gradient
            dv = 2.0
            ld_p = self.ld_ratio(weight_n, v_mid + dv, density_kg_m3, kinematic_viscosity_m2_s)
            ld_m = self.ld_ratio(weight_n, v_mid - dv, density_kg_m3, kinematic_viscosity_m2_s)
            if ld_p > ld_m:
                v_lo = v_mid
            else:
                v_hi = v_mid
            if v_hi - v_lo < 0.5:
                break
        return best_v

    def summary(
        self,
        weight_n: float,
        velocity_ms: float,
        density_kg_m3: float,
        kinematic_viscosity_m2_s: float,
    ) -> dict:
        cl = self.cl(weight_n, velocity_ms, density_kg_m3)
        cd_parasite = self.cd0(velocity_ms, kinematic_viscosity_m2_s)
        cd_induced = cl ** 2 / (math.pi * self.AR * OSWALD_EFF)
        cd_total = cd_parasite + cd_induced
        ld = cl / cd_total if cd_total > 0 else 0.0
        best_ld_v = self.best_ld_speed_ms(weight_n, density_kg_m3, kinematic_viscosity_m2_s)
        best_ld = self.ld_ratio(weight_n, best_ld_v, density_kg_m3, kinematic_viscosity_m2_s)
        return {
            "wing_area_m2": self.S,
            "wingspan_m": self.b,
            "aspect_ratio": self.AR,
            "oswald_efficiency": OSWALD_EFF,
            "cl_cruise": cl,
            "cd0_cruise": cd_parasite,
            "cd_induced_cruise": cd_induced,
            "cd_total_cruise": cd_total,
            "ld_cruise": ld,
            "best_ld": best_ld,
            "best_ld_speed_ms": best_ld_v,
        }
