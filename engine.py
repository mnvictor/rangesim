"""
Engine models for the canard pusher-propfan aircraft.

Three engine types are supported:

  super_turboshaft
    Advanced recuperated / intercooled turboshaft.  η_th = 60 %, P/W = 11 kW/kg.
    Fuel: Jet-A (LHV = 43.2 MJ/kg, ρ = 0.800 kg/L).

  turboshaft
    Conventional turboshaft (e.g. Pratt & Whitney PT6A class).
    η_th ≈ 38 %, P/W ≈ 5.5 kW/kg.  Fuel: Jet-A.

  piston
    Conventional piston aero engine (e.g. Lycoming IO-540 class).
    η_th ≈ 28 %, P/W ≈ 1.6 kW/kg.  Fuel: AVGAS 100LL (LHV ≈ 43.5 MJ/kg, ρ = 0.720 kg/L).

Altitude / temperature de-rating
---------------------------------
    P_available(h) = P_max_sl · min(1.0, (ρ(h)/ρ_sl)^0.9)

The exponent 0.9 reflects that temperature recovery partially offsets density
loss (consistent with published turboprop altitude lapse curves).  Piston engines
use the same model as a first approximation (naturally-aspirated engines lapse
slightly more steeply, but 0.9 is a reasonable average).
"""

from atmosphere import RHO0_KG_M3

ALTITUDE_LAPSE_EXP = 0.90
IDLE_FRACTION      = 0.10

ENGINE_CONFIGS: dict = {
    "super_turboshaft": {
        "label":                 "Super turboshaft (60% eff., 2× P/W)",
        "thermal_efficiency":    0.60,
        "power_to_weight_kw_kg": 11.0,
        "fuel_lhv_j_kg":         43.2e6,
        "fuel_density_kg_l":     0.800,
        "max_power_kw":          3000.0,
    },
    "turboshaft": {
        "label":                 "Conventional turboshaft (~38% eff.)",
        "thermal_efficiency":    0.38,
        "power_to_weight_kw_kg": 5.5,
        "fuel_lhv_j_kg":         43.2e6,
        "fuel_density_kg_l":     0.800,
        "max_power_kw":          447.4,    # 600 HP
    },
    "piston": {
        "label":                 "Conventional piston aero engine (~28% eff.)",
        "thermal_efficiency":    0.28,
        "power_to_weight_kw_kg": 1.6,
        "fuel_lhv_j_kg":         43.5e6,   # AVGAS 100LL
        "fuel_density_kg_l":     0.720,
        "max_power_kw":          260.99,   # 350 HP
    },
}

# Module-level constants kept for any code that still imports them directly.
_DEFAULT = ENGINE_CONFIGS["super_turboshaft"]
FUEL_LHV_J_KG         = _DEFAULT["fuel_lhv_j_kg"]
THERMAL_EFFICIENCY     = _DEFAULT["thermal_efficiency"]
POWER_TO_WEIGHT_KW_KG  = _DEFAULT["power_to_weight_kw_kg"]


class EngineModel:
    """
    Generic shaft-power engine driving the counter-rotating propfan.

    Parameters
    ----------
    max_power_kw  : rated sea-level shaft power output (kW)
    engine_type   : one of "super_turboshaft" | "turboshaft" | "piston"
    """

    def __init__(self, max_power_kw: float, engine_type: str = "super_turboshaft"):
        ecfg = ENGINE_CONFIGS.get(engine_type, ENGINE_CONFIGS["super_turboshaft"])
        self.max_power_kw           = max_power_kw
        self.engine_type            = engine_type
        self.thermal_efficiency     = ecfg["thermal_efficiency"]
        self._power_to_weight       = ecfg["power_to_weight_kw_kg"]
        self.fuel_lhv_j_kg          = ecfg["fuel_lhv_j_kg"]
        self.fuel_density_kg_l      = ecfg["fuel_density_kg_l"]
        self.bsfc_kg_per_kwh        = 1.0 / (self.thermal_efficiency * self.fuel_lhv_j_kg / 3_600_000)

    # ------------------------------------------------------------------
    # Available power
    # ------------------------------------------------------------------

    def max_power_at_altitude_kw(self, density_kg_m3: float) -> float:
        """Sea-level rated power derated for altitude."""
        sigma = density_kg_m3 / RHO0_KG_M3
        return self.max_power_kw * min(1.0, sigma ** ALTITUDE_LAPSE_EXP)

    def idle_power_kw(self, density_kg_m3: float) -> float:
        return self.max_power_at_altitude_kw(density_kg_m3) * IDLE_FRACTION

    # ------------------------------------------------------------------
    # Fuel consumption
    # ------------------------------------------------------------------

    def fuel_flow_kg_s(self, shaft_power_kw: float) -> float:
        """Mass flow of fuel for a given shaft power output (kg/s)."""
        return shaft_power_kw * 1000.0 / (self.thermal_efficiency * self.fuel_lhv_j_kg)

    def fuel_flow_kg_h(self, shaft_power_kw: float) -> float:
        return self.fuel_flow_kg_s(shaft_power_kw) * 3600.0

    def fuel_flow_L_h(self, shaft_power_kw: float) -> float:
        """Volumetric fuel flow in litres per hour."""
        return self.fuel_flow_kg_h(shaft_power_kw) / self.fuel_density_kg_l

    def sfc_kg_per_kwh(self) -> float:
        """Brake-specific fuel consumption — constant (efficiency is fixed)."""
        return self.bsfc_kg_per_kwh

    def range_factor(self, eta_prop: float) -> float:
        """
        Overall range factor = η_prop · η_th · LHV / g  (m·N/N = m).
        Multiply by (L/D) × ln(Wi/Wf) for Breguet range in metres.
        """
        from atmosphere import G0_M_S2
        return eta_prop * self.thermal_efficiency * self.fuel_lhv_j_kg / G0_M_S2

    # ------------------------------------------------------------------
    # Weight
    # ------------------------------------------------------------------

    def dry_weight_kg(self) -> float:
        """Engine dry mass (without fuel, oil, or accessories)."""
        return self.max_power_kw / self._power_to_weight

    def installation_weight_kg(self) -> float:
        """Accessories, mounts, exhaust, oil system — ~18 % of dry weight."""
        return self.dry_weight_kg() * 0.18

    def total_powerplant_weight_kg(self) -> float:
        return self.dry_weight_kg() + self.installation_weight_kg()

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def summary(self) -> dict:
        return {
            "engine_type":             self.engine_type,
            "max_power_sl_kw":         self.max_power_kw,
            "thermal_efficiency":      self.thermal_efficiency,
            "bsfc_kg_per_kwh":         self.bsfc_kg_per_kwh,
            "power_to_weight_kw_kg":   self._power_to_weight,
            "dry_weight_kg":           self.dry_weight_kg(),
            "total_powerplant_weight_kg": self.total_powerplant_weight_kg(),
            "fuel_lhv_mj_kg":          self.fuel_lhv_j_kg / 1e6,
            "fuel_density_kg_l":       self.fuel_density_kg_l,
        }
