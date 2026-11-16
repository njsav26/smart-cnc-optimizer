# -*- coding: utf-8 -*-
"""
cnc_physics.py - physics-based model of CNC spindle.

Runs in BOTH Python 3 and Ignition's Jython 2.7

Cause -> Effect Chain:

cutting speed + tool diameter   -> spindle RPM
feed per tooth + flutes + RPM   -> table feed (how fast tool travels)
feed + radial width of the cut  -> average chip thickness
chip thickness + material       -> specific cutting force, kc (Kienzle)
kc + volume removed per minute  -> cutting power
power + RPM                     -> spindle torque
power + motor losses            -> motor current
torque                          -> small drop in spindle speed (speed droop)
"""

import math, random


# ===============================================================================
# 1. PARAMETERS
#
# Sources:
#   [1] machiningdoctor.com - KC1/MC chart for raw materials, rake angle default
#   [2] IMCO catalog (M223) - 6061 cutting speed and chip load, 3-flute carbide
#   [3] ISCAR milling calculations - idle motor power, 7-12% of machine power
#   [A] Assumption - typical engineering value, to be verified
#
# Untagged values are design choices (the tool and machine being modeled).
# ===============================================================================

MATERIALS = {
    # kc1: specific cutting force for a chip 1 mm wide and 1 mm thick [N/mm^2]
    # mc: Kienzle exponent -  how strongly kc rises as chips get thinner
    # kc_variation_frac: max slow drift of kc in normal operation, bar-to-bar hardness variation

    "aluminum_6061_t6": {
        "kc1_n_mm2": 800.0,          # [1] 6061 (AlMgSiCu, 3.3211)
        "mc": 0.25,                  # [1]
        "kc_variation_frac": 0.05,   # [A] bar-to-bar hardness variation
    },
}

TOOLS = {
    # diameter_mm: cutting diameter [mm]
    # flutes: number of cutting edges (teeth) [count]
    # tool_material: links to CUTTING_DATA (speeds/feeds depend on it)
    # stiffness_n_m: force needed to deflect the tool tip 1 m [N/m]
    # natural_freq_hz: frequency the tool naturally vibrates at [Hz]
    # damping_ratio: how fast vibration dies out
    # rake_deg: angle between cutting face and virtual line perpendicular to machined surface

    "em_10mm_3fl_carbide": {
        "diameter_mm": 10.0,
        "flutes": 3,
        "tool_material": "carbide",
        "stiffness_n_m": 5.0e7,      # [A] typical short carbide end mill, real value from tap test
        "natural_freq_hz": 1500.0,   # [A]
        "damping_ratio": 0.04,       # [A]
        "rake_deg": 7.0,             # [1] default when supplier data is unknown
    },
}

CUTTING_DATA = {
    # vc: cutting speed, how fast the cutting edge moves
    # fz: feed per tooth, how far tool advances each time one flute cuts [mm]

    ("aluminum_6061_t6", "carbide"): {
        "vc_m_min": 244.0,           # [2] 800 SFM
        "fz_mm": 0.114,              # [2] 0.0045 in/tooth (3/8" tool)
    }
}

SPINDLE= {
    # max_rpm: spindle max speed [rev/min]
    # rated_power_kw: continuous motor power at rated torque [kW]
    # rated_torque_nm: continuous torque at rated torque [N*m]
    # idle_power_kw: power drawn to spin without load [kW]
    # efficiency: fraction of power doing work, ie: cutting material
    # voltage_v: 3-phase line voltage [V]
    # power_factor: fraction of current doing work
    # droop_frac_at_rated: speed drop at max torque

    "max_rpm": 12000.0,
    "rated_power_kw": 11.0,
    "rated_torque_nm": 70.0,
    "idle_power_kw": 1.0,            # [3] ~9% of 11 kW
    "efficiency": 0.85,              # [A]
    "voltage_v": 230.0,
    "power_factor": 0.85,            # [A] full-load value, real PF drops at light load
    "droop_frac_at_rated": 0.005,    # [A] modern vector drives hold speed tightly
}


# ===============================================================================
# 2. PHYSICS FUNCTIONS
#    Each function is one equation.
# ===============================================================================

def spindle_kinematics(tool, cut, feed_frac, max_rpm):
    """ Convert recommended cutting data into machine settings.

    Inputs:
        tool        : dict from TOOLS
        cut         : dict from CUTTING_DATA
        feed_frac   : feed rate override as a fraction [0-1], 1.0 = programmed feed
        max_rpm     : max spindle speed [rev/min]

    Outputs:
        rpm         : spindle speed [rev/min]
        vc          : actual cutting speed [m/min]
        fz          : actual feed per tooth after override [mm/tooth]
        vf          : table feed = tool travel speed [mm/min]
    """
    # n = vc * 1000 / (pi * D)
    # pi * D is tool circumference [mm], cicumference * RPM is edge speed
    # vc [m/min] * 1000 [mm/m] converts to [mm/min]
    rpm = cut["vc_m_min"] * 1000.0 / (math.pi * tool["diameter_mm"])

    # machine cannot exceed max RPM
    rpm = min(rpm, max_rpm)

    # if RPM got capped, cutting speed is less than recommended
    vc = math.pi * tool["diameter_mm"] * rpm / 1000.0

    # feed override adjust feed per tooth
    fz = cut["fz_mm"] * feed_frac

    # vf = fz * z * n
    # fz            = tool advance per tooth [mm/tooth]
    # fz * z        = tool advance revolution [mm/rev]
    # fz * z * n    = tool advance per minute [mm/min]
    vf = fz * tool["flutes"] * rpm

    return rpm, vc, fz, vf


def chip_geometry(fz, ae, d):
    """ Thickness of chip that each tooth cuts.

    Tooth enters material, sweep an arc, exits.
    Chip thickness at tooth angle phi: h(phi) = fz * sin(phi)
    Comma shaped chips. Thin at entry, thick at exit.

    Inputs:
        fz  : feed per tooth [mm]
        ae  : radial width of cut, how far into part [mm]
        d   : tool diameter [mm]

    Outputs:
        phi_e   : engagement angle, arc of the tooth's cut [rad]
        hm      : average chip thickness over that arc [mm]
    """
    # Tool cannot cut wider that itself
    ae = min(ae, d)

    # Engagement angle from geometry: cos(phi_e) = 1 - 2 * ae / D
    # ae = D / 2 -> 90 deg (half the circle), ae = D -> 180 deg (full circle)
    phi_e = math.acos(1.0 - 2.0 * ae / d)

    # Average of fz * sin(phi) from 0 to phi_e
    # (1/phi_e) * integral(fz * sin(phi)) = fz * (1 - cos(phi_e)) / phi_e
    hm = fz * (1.0 - math.cos(phi_e)) / phi_e

    return phi_e, hm


def specific_cutting_force(mat, hm, rake_deg=0.0, wear_factor=1.0, kc_mult=1.0):
    """ Kienzle equation with rake correction: force per mm^2 of chip cross-section [N/mm^2]

        kc = kc1 * hm^(-mc) * (1 - rake/100)

    Negative exponent is size effect: thinner chips need more energy per unit of material,
    edge's roundness and friction matter more when chip is thin.

    kc1 is defined at 0 deg rake. Each degree of positive rake lowers kc by ~1%
    (sharper edge cuts more easily). Source: Sandvik rake correction.

    [N/mm^2] = [mJ/mm^3]: kc also equals the energy (in mJ) to remove 1 mm^3

    Inputs:
        mat         : dict from MATERIALS
        hm          : average chip thickness [mm]
        rake_deg    : tool top rake angle [deg], from TOOLS
        wear_factor : >1 when tool is worn (rubbing)
        kc_mult     : material variation / chipped edge multiplier
    """
    rake_factor = 1.0 - rake_deg / 100.0
    return mat["kc1_n_mm2"] * hm ** (-mat["mc"]) * rake_factor * wear_factor * kc_mult


def cutting_power_kw(ap, ae, vf, kc):
    """ Net cutting power [kW]

        Pc = ap * ae * vf * kc / (60 * 10^6)

    ap * ae           = chip cross-section [mm^2]
    ap * ae * vf      = volume removed per minute [mm^3/min]
    * kc  [N/mm^2]    = force * distance per minute [N*mm/min] = [mJ/min]
    / 60              -> per second: [mJ/s] = [mW]
    / 10^6            -> [mW] to [kW]
    """
    return ap * ae * vf *kc / 60.0e6


def torque_nm(pc_kw, rpm):
    """ Spindle torque [N*m]

        Mc = Pc * 30 * 10^3 / (pi * n)

    Power = torque * angular speed -> torque = power / omega
    omega [rad/s] = 2 * pi * n / 60 = pi * n / 30
    * 10^3 converts kW to W
    """
    return pc_kw * 30.0e3 / (math.pi * rpm)


def motor_current_a(pc_kw, sp):
    """ 3-phase motor current [A]

    Electrical power drawn = cutting power / efficiency + idle power
    / efficiency: losses in drive, motor, and belts
    + idle power: power needed to spin without cutting

    3-phase power:  P = sqrt(3) * V * I * PF
    rearranged:     I = P / (sqrt(3) * V * PF)
    * 1000          -> convert kW to W
    """
    p_elec_kw = pc_kw / sp["efficiency"] + sp["idle_power_kw"]
    return p_elec_kw * 1000.0 / (math.sqrt(3.0) * sp["voltage_v"] * sp["power_factor"])


def spindle_speed(rpm_set, torque, sp):
    """ Actual spindle speed under load [rev/min]

    The drive's speed controller lets speed sag slightly in proportion to load torque
    (proportional speed droop)

        n_actual = n_set * (1 - droop% * (torque / rated_torque))
    """
    return rpm_set * (1.0 - sp["droop_frac_at_rated"] * torque / sp["rated_torque_nm"])


# ===============================================================================
# 3. TEST
#    Only runs when file directly executed.
#       python cnc_physics.py
# ===============================================================================

if __name__ == "__main__":
    # Setup: material, tool, cutting data
    mat = MATERIALS["aluminum_6061_t6"]
    tool = TOOLS["em_10mm_3fl_carbide"]
    cut = CUTTING_DATA[("aluminum_6061_t6", "carbide")]

    # Cut geometry: side milling, 5 mm deep, 3 mm into side (30% of D)
    ap = 5.0    # axial depth of cut
    ae = 3.0    # radial width of cut

    # Calculate, 100% feed override
    rpm, vc, fz, vf = spindle_kinematics(tool, cut, 1.0, SPINDLE["max_rpm"])
    phi_e, hm = chip_geometry(fz, ae, tool["diameter_mm"])
    kc = specific_cutting_force(mat, hm, tool["rake_deg"])
    pc = cutting_power_kw(ap, ae, vf, kc)
    mc = torque_nm(pc, rpm)
    current = motor_current_a(pc, SPINDLE)
    rpm_actual = spindle_speed(rpm, mc, SPINDLE)

    # Print each step:
    print("rpm %.0f | vc %.0f m/min | fz %.3f mm | vf %.0f mm/min" % (rpm, vc, fz, vf))
    print("phi_e %.3f rad (%.1f deg) | hm %.4f mm" % (phi_e, math.degrees(phi_e), hm))
    print("kc %.0f N/mm2 | Pc %.3f kW | torque %.3f Nm" % (kc, pc, mc))
    print("current %.2f A | rpm under load %.1f" % (current, rpm_actual))
