"""
High-efficiency turboshaft engine model.

Specification
-------------
* Architecture  : turboshaft (gas-generator + free power turbine)
* Thermal efficiency η_th = 0.60  (exceptional; Brayton cycle with recuperation
  or advanced intercooled-recuperated cycle; compare ~0.32-0.40 for a
  conventional turboprop, ~0.30 for a piston engine)
* Power-to-weight : 2 × conventional turboprop baseline
  Conventional turboprop reference: 5.5 kW/kg (e.g. Pratt & Whitney PT6A-65)
  This engine: 11 kW/kg
* Fuel           : Jet-A (or sustainable aviation fuel with equivalent LHV)
  LHV = 43.2 MJ/kg

Fuel consumption
----------------
The shaft power delivered to the propfan is:

    P_shaft = η_th · ṁ_fuel · LHV

Therefore:

    ṁ_fuel [kg/s] = P_shaft / (η_th · LHV)

Brake-specific fuel consumption (BSFC):

    BSFC = ṁ_fuel / P_shaft = 1 / (η_th · LHV)
         = 1 / (0.60 × 43 200 000) ≈ 3.86 × 10⁻⁸ kg/(W·s)
         = 0.139 kg/(kW·h)

Compare: ~0.25–0.35 kg/(kW·h) for a conventional turboprop.

Altitude / temperature de-rating
---------------------------------
Turboshaft power output is proportional to air mass flow, which is
proportional to density.  We apply a standard flat-rated / lapse model:

    P_available(h) = P_max_sl · min(1.0, (ρ(h)/ρ_sl)^0.9)

The exponent 0.9 reflects that temperature recovery partially offsets the
density loss (consistent with published turboprop altitude lapse curves).

Power setting
-------------
The engine can be throttled from idle (10 % power) to max (100 %).  In
cruise the power setting is the minimum needed to sustain level flight.
"""

from atmosphere import RHO0_KG_M3

FUEL_LHV_J_KG = 43.2e6       # J/kg  lower heating value of Jet-A
THERMAL_EFFICIENCY = 0.60
POWER_TO_WEIGHT_KW_KG = 11.0  # kW/kg  (2× conventional turboprop)
ALTITUDE_LAPSE_EXP = 0.90
IDLE_FRACTION = 0.10


class EngineModel:
    """
    Turboshaft driving the counter-rotating propfan.

    Parameters
    ----------
    max_power_kw : rated sea-level shaft power output (kW)
    """

    def __init__(self, max_power_kw: float):
        self.max_power_kw = max_power_kw
        self.bsfc_kg_per_kwh = 1.0 / (THERMAL_EFFICIENCY * FUEL_LHV_J_KG / 3_600_000)

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
        """Mass flow of fuel for a given shaft power output."""
        return shaft_power_kw * 1000.0 / (THERMAL_EFFICIENCY * FUEL_LHV_J_KG)

    def fuel_flow_kg_h(self, shaft_power_kw: float) -> float:
        return self.fuel_flow_kg_s(shaft_power_kw) * 3600.0

    def sfc_kg_per_kwh(self) -> float:
        """Brake-specific fuel consumption — constant (efficiency is fixed)."""
        return self.bsfc_kg_per_kwh

    def range_factor(self, eta_prop: float) -> float:
        """
        Overall range factor  = η_prop · η_th · LHV / g
        (numerator of the Breguet range equation, in m/N × kg).
        Equivalent to (L/D) × ln(Wi/Wf) gives range in metres.
        """
        import math
        from atmosphere import G0_M_S2
        return eta_prop * THERMAL_EFFICIENCY * FUEL_LHV_J_KG / G0_M_S2

    # ------------------------------------------------------------------
    # Weight
    # ------------------------------------------------------------------

    def dry_weight_kg(self) -> float:
        """Engine dry mass (without fuel, oil, or accessories)."""
        return self.max_power_kw / POWER_TO_WEIGHT_KW_KG

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
            "max_power_sl_kw": self.max_power_kw,
            "thermal_efficiency": THERMAL_EFFICIENCY,
            "bsfc_kg_per_kwh": self.bsfc_kg_per_kwh,
            "power_to_weight_kw_kg": POWER_TO_WEIGHT_KW_KG,
            "dry_weight_kg": self.dry_weight_kg(),
            "total_powerplant_weight_kg": self.total_powerplant_weight_kg(),
            "fuel_lhv_mj_kg": FUEL_LHV_J_KG / 1e6,
        }
