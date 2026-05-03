"""
AircraftConfig — the central configuration object.

All user-adjustable parameters live here.  When any parameter changes,
call `.reconfigure()` to rebuild all derived quantities.  This ensures
the cascade of interdependencies (fan diameter → gear length → gear weight
→ MTOW → wing area → range) is always consistent.

Typical usage
-------------
    cfg = AircraftConfig()                  # defaults
    cfg.fan_diameter_m = 1.8               # change one parameter …
    cfg.reconfigure()                       # … then update everything
    sim = RangeSimulation(cfg)
    result = sim.run()
"""

from dataclasses import dataclass, field
from atmosphere import (
    get_atmosphere, ft_to_m, ktas_to_ms, ms_to_ktas, ms_to_mach, G0_M_S2
)
from engine import EngineModel
from propfan import PropfanModel
from weight_model import WeightModel
from aerodynamics import AeroModel


@dataclass
class AircraftConfig:
    """
    User-adjustable top-level parameters.
    All others are computed by `.reconfigure()`.
    """

    # ── Propulsion ────────────────────────────────────────────────────────────
    fan_diameter_m: float = 1.60
    """Tip-to-tip diameter of each counter-rotating fan stage (m)."""

    engine_power_kw: float = 220.0
    """Maximum shaft power at sea level (kW)."""

    # ── Airframe geometry ─────────────────────────────────────────────────────
    wingspan_m: float = 10.0
    """Wing tip-to-tip span (m)."""

    cabin_width_m: float = 1.10
    """Interior cabin width at shoulder level (m)."""

    # ── Mission / loading ─────────────────────────────────────────────────────
    fuel_mass_kg: float = 160.0
    """Usable fuel loaded (kg)."""

    # ── Cruise conditions ─────────────────────────────────────────────────────
    cruise_speed_ktas: float = 210.0
    """True airspeed at cruise (knots)."""

    cruise_altitude_ft: float = 20_000.0
    """Cruise pressure altitude (feet)."""

    # ── Handling / stall ──────────────────────────────────────────────────────
    stall_speed_ktas: float = 65.0
    """
    Target clean stall speed at sea level, MTOW (knots).
    Drives wing area sizing.  Canard stall (CL_max ≈ 1.0) is the constraint.
    """

    # ── Engine type ───────────────────────────────────────────────────────────
    engine_type: str = "super_turboshaft"
    """
    One of "super_turboshaft" | "turboshaft" | "piston".
    Controls thermal efficiency, power-to-weight, and fuel properties.
    """

    # ── Construction material ─────────────────────────────────────────────────
    structural_factor: float = 0.80
    """
    Structural weight factor relative to the Torenbeek conventional-aluminium
    baseline.  Applied to wing, canard, fuselage, and landing gear.
      0.80  → 3D-printed Ti-6Al-4V with graded aluminium (topology-optimised)
      0.75  → CFRP composite lay-up
      1.00  → conventional machined aluminium
    Default is 3D-printed titanium / graded aluminium.
    """

    # ── Computed / derived fields (populated by reconfigure) ──────────────────
    # (Not user inputs — do not set these manually)
    _atmosphere: dict = field(default_factory=dict, init=False, repr=False)
    _engine: EngineModel = field(default=None, init=False, repr=False)
    _propfan: PropfanModel = field(default=None, init=False, repr=False)
    _weights: dict = field(default_factory=dict, init=False, repr=False)
    _aero: AeroModel = field(default=None, init=False, repr=False)

    def __post_init__(self):
        self.reconfigure()

    # ──────────────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────────────

    def reconfigure(self) -> None:
        """Recompute all derived quantities from user parameters."""
        # 1. Atmosphere at cruise altitude
        alt_m = ft_to_m(self.cruise_altitude_ft)
        self._atmosphere = get_atmosphere(alt_m)

        rho = self._atmosphere["density_kg_m3"]
        sos = self._atmosphere["speed_of_sound_m_s"]
        nu = self._atmosphere["kinematic_viscosity_m2_s"]

        # 2. Sub-models
        self._engine = EngineModel(max_power_kw=self.engine_power_kw, engine_type=self.engine_type)
        self._propfan = PropfanModel(diameter_m=self.fan_diameter_m)

        # 3. Cruise speed in m/s
        v_ms = ktas_to_ms(self.cruise_speed_ktas)

        # 4. Weight iteration
        wm = WeightModel(
            wingspan_m=self.wingspan_m,
            cabin_width_m=self.cabin_width_m,
            fan_diameter_m=self.fan_diameter_m,
            stall_speed_ktas=self.stall_speed_ktas,
            cruise_speed_ms=v_ms,
            fuel_mass_kg=self.fuel_mass_kg,
            engine_model=self._engine,
            propfan_model=self._propfan,
            structural_factor=self.structural_factor,
        )
        self._weights = wm.compute(density_kg_m3=rho)

        # 5. Aero model (needs converged geometry)
        self._aero = AeroModel(
            wing_area_m2=self._weights["wing_area_m2"],
            wingspan_m=self.wingspan_m,
            aspect_ratio=self._weights["aspect_ratio"],
            fuselage_length_m=self._weights["fuselage_length_m"],
            fuselage_diameter_m=self._weights["fuselage_diameter_m"],
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Convenience accessors
    # ──────────────────────────────────────────────────────────────────────────

    @property
    def cruise_altitude_m(self) -> float:
        return ft_to_m(self.cruise_altitude_ft)

    @property
    def cruise_speed_ms(self) -> float:
        return ktas_to_ms(self.cruise_speed_ktas)

    @property
    def cruise_mach(self) -> float:
        return ms_to_mach(self.cruise_speed_ms, self.cruise_altitude_m)

    @property
    def mtow_kg(self) -> float:
        return self._weights["mtow_kg"]

    @property
    def empty_weight_kg(self) -> float:
        return self._weights["empty_weight_kg"]

    @property
    def payload_kg(self) -> float:
        return self._weights["payload_kg"]

    @property
    def wing_area_m2(self) -> float:
        return self._weights["wing_area_m2"]

    @property
    def aspect_ratio(self) -> float:
        return self._weights["aspect_ratio"]

    @property
    def atmosphere(self) -> dict:
        return self._atmosphere

    @property
    def engine(self) -> EngineModel:
        return self._engine

    @property
    def propfan(self) -> PropfanModel:
        return self._propfan

    @property
    def weights(self) -> dict:
        return self._weights

    @property
    def aero(self) -> AeroModel:
        return self._aero

    # ──────────────────────────────────────────────────────────────────────────
    # Validation helpers
    # ──────────────────────────────────────────────────────────────────────────

    def validate(self) -> list[str]:
        """
        Return a list of warning strings for potentially problematic
        configurations.  Empty list = no warnings.
        """
        from aerodynamics import CL_MAX_CLEAN
        from atmosphere import ktas_to_ms

        warnings = []
        atm = self._atmosphere
        v = self.cruise_speed_ms
        rho = atm["density_kg_m3"]
        sos = atm["speed_of_sound_m_s"]
        nu = atm["kinematic_viscosity_m2_s"]
        W = self.mtow_kg * G0_M_S2

        # CL at cruise vs CL_max check
        cl_cruise = self.aero.cl(W, v, rho)
        if cl_cruise > CL_MAX_CLEAN * 0.95:
            warnings.append(
                f"Cruise CL {cl_cruise:.3f} is ≥ 95 % of CL_max ({CL_MAX_CLEAN:.2f}) — "
                f"aircraft is near stall at cruise altitude/speed."
            )
        elif cl_cruise > CL_MAX_CLEAN * 0.85:
            warnings.append(
                f"Cruise CL {cl_cruise:.3f} is high (> 85 % of CL_max {CL_MAX_CLEAN:.2f}) — "
                f"very little stall margin at cruise."
            )

        # Stall margin check (cruise TAS vs. sea-level stall speed — conservative)
        v_stall_sl = ktas_to_ms(self.stall_speed_ktas)
        stall_margin = v / v_stall_sl
        if stall_margin < 1.30:
            warnings.append(
                f"Low stall margin: {stall_margin:.2f}× (recommend ≥ 1.30×)."
            )

        # Mach limit
        if self.cruise_mach > 0.75:
            warnings.append(
                f"Cruise Mach {self.cruise_mach:.3f} may cause compressibility drag rise."
            )

        # Engine power vs. drag check
        drag_n = self.aero.drag_n(W, v, rho, nu)
        thrust_avail = self.propfan.max_thrust_n(
            v, self.engine.max_power_at_altitude_kw(rho) * 1000.0, rho, sos
        )
        if thrust_avail < drag_n * 1.05:
            warnings.append(
                f"Thrust margin thin: available {thrust_avail:.0f} N vs. "
                f"required {drag_n:.0f} N at cruise (need ≥5 % margin)."
            )

        # Wing loading
        wing_loading = self.mtow_kg * G0_M_S2 / self.wing_area_m2
        if wing_loading > 1500:
            warnings.append(f"High wing loading: {wing_loading:.0f} N/m².")

        # Propfan tip Mach
        rpm = self.propfan.design_rpm(v)
        tip_v = self.propfan.tip_speed_ms(rpm)
        tip_mach = tip_v / sos
        if tip_mach > 0.95:
            warnings.append(
                f"Propfan tip Mach {tip_mach:.2f} is very high — consider "
                f"larger diameter or lower RPM."
            )

        # Fuel fraction
        fuel_fraction = self.fuel_mass_kg / self.mtow_kg
        if fuel_fraction > 0.40:
            warnings.append(
                f"Fuel fraction {fuel_fraction:.2f} is very high "
                f"({self.fuel_mass_kg:.0f} kg / {self.mtow_kg:.0f} kg)."
            )

        if not self._weights["converged"]:
            warnings.append("MTOW iteration did not converge — results may be unreliable.")

        return warnings

    # ──────────────────────────────────────────────────────────────────────────
    # Display helper
    # ──────────────────────────────────────────────────────────────────────────

    def full_summary(self) -> dict:
        """Merged summary of all sub-models for reporting."""
        atm = self._atmosphere
        v = self.cruise_speed_ms
        rho = atm["density_kg_m3"]
        sos = atm["speed_of_sound_m_s"]
        nu = atm["kinematic_viscosity_m2_s"]
        W = self.mtow_kg * G0_M_S2

        propfan_sum = self.propfan.summary(v, sos)
        engine_sum = self.engine.summary()
        aero_sum = self.aero.summary(W, v, rho, nu)

        # Effective propulsive efficiency at cruise
        drag_n = self.aero.drag_n(W, v, rho, nu)   # thrust required
        eta_prop = self.propfan.efficiency(v, drag_n, rho, sos)

        return {
            "user_inputs": {
                "fan_diameter_m": self.fan_diameter_m,
                "engine_power_kw": self.engine_power_kw,
                "wingspan_m": self.wingspan_m,
                "cabin_width_m": self.cabin_width_m,
                "fuel_mass_kg": self.fuel_mass_kg,
                "cruise_speed_ktas": self.cruise_speed_ktas,
                "cruise_altitude_ft": self.cruise_altitude_ft,
            },
            "atmosphere": {
                "altitude_ft": self.cruise_altitude_ft,
                "altitude_m": self.cruise_altitude_m,
                "temperature_c": atm["temperature_k"] - 273.15,
                "pressure_pa": atm["pressure_pa"],
                "density_kg_m3": rho,
                "density_ratio": rho / 1.225,
                "speed_of_sound_ktas": ms_to_ktas(sos),
                "cruise_mach": self.cruise_mach,
            },
            "propfan": propfan_sum,
            "engine": engine_sum,
            "weights": self._weights,
            "aerodynamics": aero_sum,
            "propulsive_efficiency_cruise": eta_prop,
        }
