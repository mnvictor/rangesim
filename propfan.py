"""
Counter-rotating pusher propfan model.

Physical basis
--------------
A propfan (also called an unducted fan or open rotor) achieves higher
propulsive efficiency than a conventional propeller at cruise speeds up to
Mach ~0.8 by using swept, thin, high-solidity blades. Counter-rotation
(two contra-rotating stages) eliminates residual swirl, recovering ~3-5 %
additional efficiency versus a single-rotation design.

Blade sweep and high-speed capability
--------------------------------------
The key limit on propfan cruise speed is blade tip compressibility.
Tip speed is independent of diameter:

    tip_speed = π · V_∞ / J

For a straight blade, compressibility sets in when tip_mach approaches 0.85.
Swept (scimitar) blades work like a swept wing: only the velocity component
*normal* to the leading edge drives compressibility.  For sweep angle Λ:

    M_eff = M_tip · cos(Λ)

Modern open-rotor concepts (CFM RISE, GE36 follow-ons) use 35–40° of sweep
to achieve M0.80 cruise.  At 35° sweep, a tip Mach of 1.04 produces an
effective tip Mach of only 0.85 — right at the compressibility onset.

The swept blade also carries a slight weight penalty (~0.5 % per degree)
due to the more complex structural load path, and a small off-design profile
efficiency reduction from 3-D spanwise flow.

Efficiency model
----------------
1. Ideal (actuator-disk) efficiency given disk loading:
       η_ideal = 2 / (1 + sqrt(1 + CT_disk))
2. Technology factors:
       - blade profile losses (viscous drag)
       - tip compressibility on *effective* Mach (post-sweep correction)
       - 3-D profile penalty for swept blades
       - swirl recovery (counter-rotation eliminates most swirl loss)
       - installation / blockage (pusher location in fuselage wake)
"""

import math


# Propfan technology constants
ETA_PROFILE      = 0.94    # blade profile (viscous) efficiency factor
ETA_SWIRL_CR     = 0.985   # swirl-recovery factor for counter-rotation
ETA_INSTALLATION = 0.97    # pusher installation factor (wake ingestion)
J_DESIGN         = 2.4     # design-point advance ratio (typical range 2.0–3.0)
TIP_MACH_LIMIT   = 0.85    # effective tip Mach above which compressibility penalty starts
TIP_MACH_MAX     = 1.05    # effective tip Mach at which efficiency reaches minimum


def _compressibility_factor(effective_tip_mach: float) -> float:
    """
    Penalty on efficiency due to blade tip compressibility.
    Applied to the sweep-corrected effective tip Mach, not raw tip Mach.
    """
    if effective_tip_mach <= TIP_MACH_LIMIT:
        return 1.0
    elif effective_tip_mach >= TIP_MACH_MAX:
        return 0.82
    fraction = (effective_tip_mach - TIP_MACH_LIMIT) / (TIP_MACH_MAX - TIP_MACH_LIMIT)
    return 1.0 - 0.18 * fraction


def _advance_ratio_efficiency_factor(J: float) -> float:
    """
    Correction factor as a function of advance ratio relative to design point.
    Peaks at J_DESIGN; drops off to either side.  Fitted to published propfan
    wind-tunnel data (Hamilton Standard / NASA Lewis propfan test results).
    """
    x = (J - J_DESIGN) / J_DESIGN
    if J <= J_DESIGN:
        return max(0.70, 1.0 - 1.8 * x ** 2)
    else:
        return max(0.70, 1.0 - 1.2 * x ** 2)


class PropfanModel:
    """
    Two-stage counter-rotating pusher propfan with optional blade sweep.

    Parameters
    ----------
    diameter_m       : tip-to-tip diameter of each fan stage (m)
    num_blades_each  : number of blades per stage (default 8+8)
    blade_sweep_deg  : leading-edge sweep angle (degrees, 0 = straight blade).
                       Modern M0.80 open-rotor designs (CFM RISE) use 35–40°.
    """

    def __init__(
        self,
        diameter_m: float,
        num_blades_each: int = 8,
        blade_sweep_deg: float = 0.0,
    ):
        self.diameter_m      = diameter_m
        self.num_blades_each = num_blades_each
        self.blade_sweep_deg = blade_sweep_deg
        self._sweep_rad      = math.radians(blade_sweep_deg)
        self.disk_area_m2    = math.pi * diameter_m ** 2 / 4.0

    # ------------------------------------------------------------------
    # Core calculations
    # ------------------------------------------------------------------

    def design_rpm(self, cruise_speed_ms: float) -> float:
        """Shaft speed (RPM) for the design advance ratio at cruise speed."""
        n_rps = cruise_speed_ms / (J_DESIGN * self.diameter_m)
        return n_rps * 60.0

    def tip_speed_ms(self, shaft_rpm: float) -> float:
        return math.pi * self.diameter_m * shaft_rpm / 60.0

    def advance_ratio(self, airspeed_ms: float, shaft_rpm: float) -> float:
        n_rps = shaft_rpm / 60.0
        return airspeed_ms / (n_rps * self.diameter_m) if n_rps > 0 else 0.0

    def effective_tip_mach(self, airspeed_ms: float, speed_of_sound_ms: float) -> float:
        """
        Tip Mach component normal to the blade leading edge (after sweep correction).
        This is what drives compressibility losses.
        """
        rpm = self.design_rpm(airspeed_ms)
        raw = self.tip_speed_ms(rpm) / speed_of_sound_ms
        return raw * math.cos(self._sweep_rad)

    # ------------------------------------------------------------------
    # Efficiency at a given flight condition
    # ------------------------------------------------------------------

    def efficiency(
        self,
        airspeed_ms: float,
        thrust_n: float,
        density_kg_m3: float,
        speed_of_sound_ms: float,
    ) -> float:
        """
        Propulsive efficiency  η = T·V / P_shaft.

        Sweep reduces compressibility losses but adds a small 3-D profile
        penalty from spanwise flow (~0.06 % per degree above 5°).
        """
        if airspeed_ms < 1.0 or thrust_n <= 0.0:
            return 0.60

        # Actuator-disk ideal efficiency based on disk loading
        q_inf = 0.5 * density_kg_m3 * airspeed_ms ** 2
        CT_disk = thrust_n / (q_inf * self.disk_area_m2 + 1e-9)
        eta_ideal = 2.0 / (1.0 + math.sqrt(max(1.0, 1.0 + CT_disk)))

        # Compressibility correction using sweep-corrected effective tip Mach
        eff_mach = self.effective_tip_mach(airspeed_ms, speed_of_sound_ms)
        eta_comp = _compressibility_factor(eff_mach)

        # 3-D spanwise-flow profile penalty for swept blades
        eta_sweep_3d = max(0.965, 1.0 - 0.0006 * self.blade_sweep_deg)

        rpm_design = self.design_rpm(airspeed_ms)
        J = self.advance_ratio(airspeed_ms, rpm_design)
        eta_J = _advance_ratio_efficiency_factor(J)

        eta = (eta_ideal * ETA_PROFILE * eta_sweep_3d * ETA_SWIRL_CR
               * ETA_INSTALLATION * eta_comp * eta_J)
        return min(eta, 0.935)

    # ------------------------------------------------------------------
    # Derived quantities
    # ------------------------------------------------------------------

    def shaft_power_w(
        self,
        airspeed_ms: float,
        thrust_n: float,
        density_kg_m3: float,
        speed_of_sound_ms: float,
    ) -> float:
        """Shaft power required to produce `thrust_n` at `airspeed_ms`."""
        eta = self.efficiency(airspeed_ms, thrust_n, density_kg_m3, speed_of_sound_ms)
        return thrust_n * airspeed_ms / eta

    def max_thrust_n(
        self,
        airspeed_ms: float,
        shaft_power_w: float,
        density_kg_m3: float,
        speed_of_sound_ms: float,
        tol: float = 0.1,
    ) -> float:
        """Thrust delivered when `shaft_power_w` is applied at `airspeed_ms`."""
        if airspeed_ms < 1.0:
            T_guess = (2.0 * density_kg_m3 * self.disk_area_m2 * shaft_power_w ** 2) ** (1.0 / 3.0)
            return T_guess

        T = shaft_power_w * 0.85 / airspeed_ms
        for _ in range(20):
            eta = self.efficiency(airspeed_ms, T, density_kg_m3, speed_of_sound_ms)
            T_new = shaft_power_w * eta / airspeed_ms
            if abs(T_new - T) < tol:
                return T_new
            T = T_new
        return T

    # ------------------------------------------------------------------
    # Weight estimate
    # ------------------------------------------------------------------

    def weight_kg(self) -> float:
        """
        Structural mass of the two-stage counter-rotating propfan assembly.

        Base: W ≈ 18 · D^2.5  (calibrated to Hamilton Standard SR-7L).
        Sweep penalty: ~0.5 % per degree — swept scimitar blades carry
        complex 3-D bending/torsion loads requiring heavier root structure.
        """
        base = 18.0 * self.diameter_m ** 2.5
        sweep_factor = 1.0 + 0.005 * self.blade_sweep_deg
        return base * sweep_factor

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def max_feasible_speed_ms(self, speed_of_sound_ms: float) -> float:
        """
        Maximum cruise speed before effective tip Mach reaches TIP_MACH_LIMIT.

        Sweep increases this by 1/cos(Λ):
            V_max = TIP_MACH_LIMIT · J · sos / (π · cos(Λ))

        At 35° sweep this is ~22 % faster than a straight blade,
        enabling M0.80 cruise for the first time.
        """
        cos_sweep = math.cos(self._sweep_rad)
        return TIP_MACH_LIMIT * J_DESIGN * speed_of_sound_ms / (math.pi * cos_sweep)

    def summary(self, cruise_speed_ms: float, speed_of_sound_ms: float) -> dict:
        rpm = self.design_rpm(cruise_speed_ms)
        tip_v = self.tip_speed_ms(rpm)
        raw_tip_mach = tip_v / speed_of_sound_ms
        eff_mach = raw_tip_mach * math.cos(self._sweep_rad)
        return {
            "fan_diameter_m":       self.diameter_m,
            "blade_sweep_deg":      self.blade_sweep_deg,
            "disk_area_m2":         self.disk_area_m2,
            "design_rpm":           rpm,
            "tip_speed_ms":         tip_v,
            "tip_mach":             raw_tip_mach,
            "eff_tip_mach":         eff_mach,
            "design_advance_ratio": J_DESIGN,
            "assembly_weight_kg":   self.weight_kg(),
        }
