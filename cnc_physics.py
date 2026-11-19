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
#   [4] Machinery's Handbook, 31st ed. - Taylor tool life, cumulative tool-life fraction
#   [A] Assumption - typical engineering value, to be verified
#
# Untagged values are design choices (the tool and machine being modeled).
# Naming: suffix = unit. _frac = fraction 0-1, _pct = percent 0-100 (UI/tags only)
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
    # --- GEOMETRY ---
    # diameter_mm: cutting diameter [mm]
    # flutes: number of cutting edges (teeth) [count]
    # tool_material: links to CUTTING_DATA (speeds/feeds depend on it)
    # rake_deg: angle between cutting face and virtual line perpendicular to machined surface

    # --- DYNAMICS ---
    # stiffness_n_m: force needed to deflect the tool tip 1 m [N/m]
    # natural_freq_hz: frequency the tool naturally vibrates at [Hz]
    # damping_ratio: how fast vibration dies out

    # --- TOOL LIFE ---
    # vb_criterion_mm: flank wear width that define end of tool life [mm]
    # life_scatter_sigma: life spread between identical tools [lognormal sigma]

    "em_10mm_3fl_carbide": {
        # geometry
        "diameter_mm": 10.0,
        "flutes": 3,
        "tool_material": "carbide",
        "rake_deg": 7.0,             # [1] default when supplier data is unknown
        # dynamics
        "stiffness_n_m": 5.0e7,      # [A] typical short carbide end mill, real value from tap test
        "natural_freq_hz": 1500.0,   # [A]
        "damping_ratio": 0.04,       # [A]
        # tool life
        "vb_criterion_mm": 0.3,       # [A] common end-of-life criterion for carbide end mills
        "life_scatter_sigma": 0.25,   # [A] identical tools vary ~+/-25%
    },
}

CUTTING_DATA = {
    # keyed by (material, specific tool): how this tool behaves in the material

    # --- SPEEDS AND FEEDS ---
    # vc_m_min: cutting speed, how fast the cutting edge moves through material [m/min]
    # fz_per_diameter: chip load as a fraction of tool diameter [mm/mm]
    #                  fz_ref = fz_per_diameter * D
    # fz_min_mm: minimum chip load (below this value, the edge rubs instead of cuts) [mm]

    # --- TOOL LIFE (from extended Taylor) ---
    # taylor_n: speed exponent, how strongly speed shortens tool life
    # taylor_a: feed exponent, how strongly feed shortens tool life
    # life_ref_min: tool life at vc_m_min and fz_min, until VB = criterion [min]

    # --- WEAR AND PROCESS BEHAVIOR ---
    # rub_gain: tool life penalty strength below fz_min
    # wear_force_gain: cutting force increase at VB = criterion [fraction]
    # coolant_loss_kc_factor: kc multiplier when coolant flow is lost

    ("aluminum_6061_t6", "carbide"): {
        # speeds and feeds
        "vc_m_min": 244.0,                 # [2] 800 SFM
        "fz_per_diameter": 0.012,          # [2] IPT/D ratio, 3/8"-1" tools (e.g. .0045"/.375")
        "fz_min_mm": 0.05,                 # [A] common ~0.002 in guideline
        # tool life
        "taylor_n": 0.25,                  # [A] carbide range 0.2-0.4, equation form [4]
        "taylor_a": 0.5,                   # [A] feed affects life less than speed
        "life_ref_min": 240.0,             # [A] carbide in aluminum, long life
        # wear and process behavior
        "rub_gain": 3.0,                   # [A]
        "wear_force_gain": 0.30,           # [A] worn flank rubs, ~+30% force at end of life
        "coolant_loss_kc_factor": 1.10,    # [A]
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

def ref_feed_per_tooth(cut, tool):
    """ Catalog chip load for this tool's diameter [mm/tooth].
    Chip load scales with diameter: fz_ref = (fz/D ratio) * D  [2]"""
    return cut["fz_per_diameter"] * tool["diameter_mm"]


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
    fz = ref_feed_per_tooth(cut, tool) * feed_frac

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


def rubbing_factor(fz, fz_min, rub_gain):
    """ Penalty when chip is too thin to cut cleanly.

    Below minimum chip load, tool edge rubs causing more friction, heat, and wear.

        k_rub = 1                                   if fz >= fz_min
        K_rub = 1 + rub_gain * (fz_min / fz - 1)     if fz <  fz_min

    k_rub = 1.29 -> tool life shortened by 29%
    """
    if fz >= fz_min:
        return 1.0
    elif fz < fz_min:
        return 1.0 + rub_gain * (fz_min / fz - 1.0)


def tool_life_min(cut, vc, fz, fz_ref, life_mult=1.0, k_rub=1.0):
    """ Extended Taylor equation: minutes pf cutting until VB reaches criterion [min]

    Original form [4]: vc * T^n * fz^a = C
    Relative to known reference point:

        T = T_ref * (vc_ref / vc)^(1/n) * (fz_ref / fz)^(a/n)

    life_mult: this particular tool's random life scatter
    k_rub: rubbing penalty below minimum chip load
    """
    n, a = cut["taylor_n"], cut["taylor_a"]
    t = (cut["life_ref_min"]
         * (cut["vc_m_min"] / vc) ** (1 / n)
         * (fz_ref / fz) ** (a / n))

    return t * life_mult / k_rub


def new_tool_life_mult(sigma):
    """ Random life multiplier generated once per tool.

    Lognormal: always positive and skewed so tools last normal than average.
    With sigma = 0.25 -> tools within +/- 25% of nominal life
    """
    return random.lognormvariate(0.0, sigma)


def update_life_frac(life_frac, dt_min, life_min):
    """ Fraction of tool life used so far [0 = new, 1 = end of life].

    Cumulative tool-life fraction [4]: each time step uses up dt/T of the
    tool's life at current conditions.  If feed drops, T gets longer, so
    each minute uses less life.

        f(t + dt) = f(t) + dt / T
    """
    return life_frac + dt_min / life_min


def flank_wear_mm(life_frac, vb_criterion_mm):
    """ Flank wear width, VB, from fraction of life used [mm].

    3-Stage Wear Curve (scale so VB = criterion at f = 1):
    1) Break-In:    (f < 0.1)   Fast initial wear. Rounds off sharp edge.
    2) Steady:      (0.1-0.8)   Slow, linear wear. Most of tool's life.
    3 Accelerated:  (f > 0.8)   Wear speeds up until failure.

    shape = fraction of criterion met.
    F(f, shape) -> (0.1, 0.25), (0.8, 0.75), (1.0, 1.00)
    """
    f = max(0.0, life_frac)
    if f < 0.1:
        shape = 0.25 * (f / 0.1) ** 0.5
    elif f < 0.8:
        shape = 0.25 + 0.50 * (f - 0.1) / 0.7
    else:
        shape = 0.75 + 0.25 * ((f - 0.7) / 0.2) ** 2.5
    return vb_criterion_mm * shape


def wear_force_factor(vb, vb_criterion_mm, wear_force_gain):
    """ How muich flank wear increases cutting force.

    Worn flank rubs on cut surface, adding friction.

        K_wear = 1 + gain * VB / VB_criterion
    """
    return 1.0 + wear_force_gain * vb / vb_criterion_mm


# ===============================================================================
# 3. TEST
#    Only runs when file directly executed.
#       python cnc_physics.py
# ===============================================================================

if __name__ == "__main__":
    # Setup: material, tool, cutting data
    material_key = "aluminum_6061_t6"
    tool_key = "em_10mm_3fl_carbide"


    mat = MATERIALS[material_key]
    tool = TOOLS[tool_key]
    cut = CUTTING_DATA[(material_key, tool["tool_material"])]

    # Cut geometry: side milling, 5 mm deep, 3 mm into side (30% of D)
    ap = 5.0    # axial depth of cut
    ae = 3.0    # radial width of cut

    # --- Spindle Physics ---
    fz_ref = ref_feed_per_tooth(cut, tool)
    rpm, vc, fz, vf = spindle_kinematics(tool, cut, 1.0, SPINDLE["max_rpm"])
    phi_e, hm = chip_geometry(fz, ae, tool["diameter_mm"])
    kc = specific_cutting_force(mat, hm, tool["rake_deg"])
    pc = cutting_power_kw(ap, ae, vf, kc)
    mc = torque_nm(pc, rpm)
    current = motor_current_a(pc, SPINDLE)
    rpm_actual = spindle_speed(rpm, mc, SPINDLE)

    print("rpm %.0f | vc %.0f m/min | fz %.3f mm | vf %.0f mm/min" % (rpm, vc, fz, vf))
    print("phi_e %.3f rad (%.1f deg) | hm %.4f mm" % (phi_e, math.degrees(phi_e), hm))
    print("kc %.0f N/mm2 | Pc %.3f kW | torque %.3f Nm" % (kc, pc, mc))
    print("current %.2f A | rpm under load %.1f" % (current, rpm_actual))

    # --- Tool Wear ---
    print("--- Tool life vs feed override ---")
    for feed in (1.0, 0.85, 0.7, 0.5, 0.4):
        rpm_f, vc_f, fz_f, vf_f = spindle_kinematics(tool, cut, feed, SPINDLE["max_rpm"])
        k_rub = rubbing_factor(fz_f, cut["fz_min_mm"], cut["rub_gain"])
        life = tool_life_min(cut, vc_f, fz_f, fz_ref, k_rub=k_rub)
        vol = ap * ae * vf_f * life / 1000.0      # material removed per tool [cm3]
        print("feed %3.0f%% | fz %.4f mm | k_rub %.2f | tool life %5.0f min | removed per tool %6.0f cm3"
              % (feed * 100, fz_f, k_rub, life, vol))

    print("--- Wear progression at 100% feed ---")
    for f in (0.0, 0.1, 0.5, 0.8, 0.9, 1.0, 1.1):
        vb = flank_wear_mm(f, tool["vb_criterion_mm"])
        k_wear = wear_force_factor(vb, tool["vb_criterion_mm"], cut["wear_force_gain"])
        kc_w = specific_cutting_force(mat, hm, tool["rake_deg"], wear_factor=k_wear)
        pc_w = cutting_power_kw(ap, ae, vf, kc_w)
        print("life used %3.0f%% | VB %.3f mm | K_wear %.3f | kc %4.0f N/mm2 | Pc %.3f kW | I %.2f A"
              % (f * 100, vb, k_wear, kc_w, pc_w, motor_current_a(pc_w, SPINDLE)))

    print("--- Coolant flow loss (fresh tool) ---")
    kc_c = specific_cutting_force(mat, hm, tool["rake_deg"], wear_factor=cut["coolant_loss_kc_factor"])
    pc_c = cutting_power_kw(ap, ae, vf, kc_c)
    print("kc %4.0f N/mm2 | Pc %.3f kW | I %.2f A" % (kc_c, pc_c, motor_current_a(pc_c, SPINDLE)))

    print("--- Simulated tool run until worn out (dt = 1 min) ---")
    for feed in (1.0, 0.7):
        rpm_f, vc_f, fz_f, vf_f = spindle_kinematics(tool, cut, feed, SPINDLE["max_rpm"])
        k_rub = rubbing_factor(fz_f, cut["fz_min_mm"], cut["rub_gain"])
        life = tool_life_min(cut, vc_f, fz_f, fz_ref, k_rub=k_rub)
        f, minutes = 0.0, 0
        # small tolerance: never compare floats for exact equality
        while flank_wear_mm(f, tool["vb_criterion_mm"]) < tool["vb_criterion_mm"] - 1e-9:
            f = update_life_frac(f, 1.0, life)
            minutes += 1
        print("feed %3.0f%% | VB reaches 0.3 mm after %d min | material removed %.0f cm3"
              % (feed * 100, minutes, ap * ae * vf_f * minutes / 1000.0))

    print("--- Tool life scatter: 5 new tools (seed 42) ---")
    random.seed(42)    # same "random" tools every run, for repeatable results
    for k in range(5):
        m = new_tool_life_mult(tool["life_scatter_sigma"])
        print("tool %d | life_mult %.2f | life %4.0f min" % (k + 1, m, tool_life_min(cut, vc, fz, fz_ref, life_mult=m)))
