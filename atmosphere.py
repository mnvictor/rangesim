"""
ISA standard atmosphere model (ICAO Doc 7488).
Valid from sea level to 20 km (troposphere + lower stratosphere).
"""

import math

# Sea-level constants
T0_K = 288.15        # K
P0_PA = 101325.0     # Pa
RHO0_KG_M3 = 1.225  # kg/m³
A0_M_S = 340.294    # m/s  (speed of sound at SL)

# Physical constants
R_J_KGK = 287.058   # J/(kg·K)  specific gas constant for air
GAMMA = 1.4          # heat capacity ratio
G0_M_S2 = 9.80665   # m/s²

# Lapse rates and layer boundaries
TROPOPAUSE_M = 11000.0   # m
STRATOPAUSE_M = 20000.0  # m
L_K_M = 0.0065          # K/m  lapse rate in troposphere


def get_atmosphere(altitude_m: float) -> dict:
    """
    Return ISA atmosphere properties at the given geometric altitude (m).

    Returns a dict with keys:
        temperature_k, pressure_pa, density_kg_m3, speed_of_sound_m_s,
        dynamic_viscosity_pa_s, kinematic_viscosity_m2_s
    """
    if altitude_m < 0:
        altitude_m = 0.0

    if altitude_m <= TROPOPAUSE_M:
        # Troposphere: linear temperature decrease
        T = T0_K - L_K_M * altitude_m
        P = P0_PA * (T / T0_K) ** (G0_M_S2 / (R_J_KGK * L_K_M))
    elif altitude_m <= STRATOPAUSE_M:
        # Lower stratosphere: isothermal at tropopause temperature
        T_trop = T0_K - L_K_M * TROPOPAUSE_M
        P_trop = P0_PA * (T_trop / T0_K) ** (G0_M_S2 / (R_J_KGK * L_K_M))
        T = T_trop
        P = P_trop * math.exp(-G0_M_S2 * (altitude_m - TROPOPAUSE_M) / (R_J_KGK * T_trop))
    else:
        raise ValueError(f"Altitude {altitude_m:.0f} m exceeds model limit of {STRATOPAUSE_M:.0f} m")

    rho = P / (R_J_KGK * T)
    a = math.sqrt(GAMMA * R_J_KGK * T)

    # Sutherland's law for dynamic viscosity
    mu = 1.458e-6 * T**1.5 / (T + 110.4)
    nu = mu / rho

    return {
        "temperature_k": T,
        "pressure_pa": P,
        "density_kg_m3": rho,
        "speed_of_sound_m_s": a,
        "dynamic_viscosity_pa_s": mu,
        "kinematic_viscosity_m2_s": nu,
    }


def density_ratio(altitude_m: float) -> float:
    return get_atmosphere(altitude_m)["density_kg_m3"] / RHO0_KG_M3


def ft_to_m(feet: float) -> float:
    return feet * 0.3048


def m_to_ft(metres: float) -> float:
    return metres / 0.3048


def ktas_to_ms(ktas: float) -> float:
    return ktas * 1852.0 / 3600.0


def ms_to_ktas(ms: float) -> float:
    return ms * 3600.0 / 1852.0


def ms_to_mach(ms: float, altitude_m: float) -> float:
    return ms / get_atmosphere(altitude_m)["speed_of_sound_m_s"]
