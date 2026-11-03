"""Avionics bay: IMU, flight controller, data processing unit, battery, power, ESC, radio, GNSS rx, tail servos."""
import math
import random
from mathutils import Vector
import dl
import interior
from dl import PB, part_props, sx, zc, wh, half_width_at

ZF = -0.072              # avionics floor top
ZT = -0.048              # aft tray top
PORTS = {}


def jst(pb, loc, n=4, rot=0):
    w = 0.0025 * n + 0.002
    pb.box((0.006, w, 0.005) if rot == 0 else (w, 0.006, 0.005), loc=loc, mat="beige")


def chip(pb, loc, size, h=0.0015, mat="chip"):
    pb.box((size[0], size[1], h), loc=(loc[0], loc[1], loc[2] + h / 2), mat=mat)


def build_imu(root):
    x, z = sx(0.205), -0.050
    pb = PB("IMU")
    # carbon shelf on two posts
    pb.box((0.085, 0.070, 0.003), loc=(x, 0, z - 0.0015), mat="carbon")
    for sy in (-1, 1):
        pb.cyl(0.003, 0.026, loc=(x - 0.03, sy * 0.028, z - 0.016), mat="alu", seg=8)
        pb.cyl(0.003, 0.026, loc=(x + 0.03, sy * 0.028, z - 0.016), mat="alu", seg=8)
    # gold anodised INS housing with ears
    pb.box((0.055, 0.046, 0.034), loc=(x, 0, z + 0.017), mat="gold", bevel=0.003, seg=1)
    pb.box((0.070, 0.012, 0.003), loc=(x, 0.012, z + 0.0015), mat="gold")
    pb.box((0.070, 0.012, 0.003), loc=(x, -0.012, z + 0.0015), mat="gold")
    for sx_ in (-1, 1):
        for sy in (-1, 1):
            pb.cyl(0.002, 0.003, loc=(x + sx_ * 0.031, sy * 0.012, z + 0.004), mat="dark", seg=6)
    # lid screws + label plate
    pb.box((0.040, 0.030, 0.0006), loc=(x, 0, z + 0.0343), mat="dark")
    pb.cyl(0.0065, 0.012, loc=(x + 0.033, 0, z + 0.017), rot=(0, 90, 0), mat="alu", seg=12)
    PORTS["imu"] = Vector((x + 0.040, 0, z + 0.017))
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("IMU / INS", "Avionics", (-0.05, 0, 0.36),
                                      "Tactical-grade inertial measurement unit fused with GNSS for precise "
                                      "trajectory (attitude for LiDAR georeferencing).", 41))


def build_fc(root):
    x, y = -0.40, 0.0
    z = ZF
    pb = PB("Flight_Controller")
    pb.box((0.105, 0.075, 0.004), loc=(x, y, z + 0.002), mat="alu", bevel=0.002)
    for sx_ in (-1, 1):
        for sy in (-1, 1):
            px, py = x + sx_ * 0.042, y + sy * 0.028
            pb.cyl(0.0045, 0.004, loc=(px, py, z + 0.006), mat="orange", seg=8)
            pb.cyl(0.0022, 0.034, loc=(px, py, z + 0.025), mat="gold", seg=6)
    # main board
    zb = z + 0.024
    pb.box((0.095, 0.066, 0.0016), loc=(x, y, zb), mat="pcb")
    t = zb + 0.0008
    chip(pb, (x - 0.005, y, t), (0.016, 0.016), 0.0018)
    chip(pb, (x + 0.020, y + 0.016, t), (0.008, 0.008), 0.0012, "alu")       # IMU sensor can
    chip(pb, (x + 0.020, y - 0.014, t), (0.010, 0.006), 0.0012)
    chip(pb, (x - 0.028, y + 0.018, t), (0.008, 0.008), 0.0012)
    chip(pb, (x - 0.028, y - 0.016, t), (0.012, 0.006), 0.0012)
    random.seed(4)
    for k in range(14):
        px = x + random.uniform(-0.04, 0.04)
        py = y + random.uniform(-0.027, 0.027)
        if abs(px - x + 0.005) < 0.012 and abs(py - y) < 0.012:
            continue
        pb.cyl(0.0018, 0.003, loc=(px, py, t + 0.0015), mat="alu", seg=6)
    for k in range(6):
        jst(pb, (x + 0.044, y - 0.025 + k * 0.010, t + 0.0025), 3)
    for k in range(4):
        jst(pb, (x - 0.044, y - 0.0195 + k * 0.013, t + 0.0025), 4)
    pb.box((0.010, 0.008, 0.004), loc=(x + 0.010, y + 0.031, t + 0.002), mat="alu")   # USB
    pb.box((0.014, 0.012, 0.0015), loc=(x - 0.010, y - 0.026, t + 0.0008), mat="alu")  # SD slot
    for k, col in enumerate(("red", "wblue", "yellow")):
        pb.box((0.002, 0.0015, 0.0008), loc=(x + 0.030, y - 0.004 + k * 0.004, t + 0.0004), mat=col)
    # top board (companion / IO) with heatsink
    zb2 = z + 0.042
    pb.box((0.080, 0.060, 0.0016), loc=(x, y, zb2), mat="pcb")
    pb.box((0.030, 0.030, 0.003), loc=(x - 0.012, y, zb2 + 0.0023), mat="alu")
    for k in range(7):
        pb.box((0.030, 0.0012, 0.008), loc=(x - 0.012, y - 0.0135 + k * 0.0045, zb2 + 0.0083), mat="alu")
    chip(pb, (x + 0.022, y + 0.012, zb2 + 0.0008), (0.010, 0.010), 0.0015)
    for k in range(5):
        jst(pb, (x + 0.034, y - 0.020 + k * 0.010, zb2 + 0.0033), 3)
    pb.box((0.004, 0.030, 0.012), loc=(x - 0.036, y, (zb + zb2) / 2), mat="dark")    # board-to-board header
    PORTS["fc_l"] = Vector((x + 0.050, y - 0.02, t + 0.003))
    PORTS["fc_r"] = Vector((x + 0.050, y + 0.02, t + 0.003))
    PORTS["fc_f"] = Vector((x - 0.050, y, t + 0.003))
    PORTS["fc_top"] = Vector((x + 0.036, y, zb2 + 0.004))
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Flight Controller", "Avionics", (0.0, 0, 0.44),
                                      "Autopilot: triple-redundant IMUs, barometer, STM32H7 MCU, mounted on "
                                      "vibration grommets with a companion I/O board.", 42))


def build_dpu(root):
    x, y = -0.17, 0.0
    z = ZF
    L, W, H = 0.130, 0.096, 0.036
    pb = PB("Data_Processing_Unit")
    for sy in (-1, 1):
        pb.box((L + 0.016, 0.012, 0.003), loc=(x, sy * (W / 2 - 0.004), z + 0.0015), mat="alu")
        for sx_ in (-1, 1):
            pb.cyl(0.0025, 0.002, loc=(x + sx_ * (L / 2 + 0.004), sy * (W / 2 - 0.004), z + 0.004), mat="dark", seg=6)
    pb.box((L, W, H), loc=(x, y, z + H / 2 + 0.003), mat="alu", bevel=0.003)
    nf = 15
    for k in range(nf):
        pb.box((L - 0.012, 0.0018, 0.014), loc=(x, y - W / 2 + 0.006 + k * (W - 0.012) / (nf - 1), z + H + 0.003 + 0.007),
               mat="alu")
    # ports on the front face (-X)
    fx = x - L / 2 - 0.0008
    for k, (w, h, m) in enumerate(((0.016, 0.013, "dark"), (0.016, 0.013, "dark"), (0.012, 0.006, "alu"),
                                    (0.010, 0.010, "dark"), (0.006, 0.006, "alu"))):
        pb.box((0.002, w, h), loc=(fx, y - 0.036 + k * 0.018, z + 0.003 + H * 0.45), mat=m)
    pb.box((0.002, 0.004, 0.002), loc=(fx, y + 0.042, z + 0.003 + H * 0.8), mat="wblue")
    PORTS["dpu_f"] = Vector((fx - 0.002, y - 0.027, z + 0.003 + H * 0.45))
    PORTS["dpu_f2"] = Vector((fx - 0.002, y - 0.009, z + 0.003 + H * 0.45))
    PORTS["dpu_pwr"] = Vector((fx - 0.002, y + 0.027, z + 0.003 + H * 0.45))
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Data Processing Unit", "Avionics", (0.0, 0, 0.56),
                                      "Onboard computer: synchronises LiDAR, camera and INS data, records to "
                                      "SSD and runs real-time point-cloud preview. Passive finned cooling.", 43))


def build_battery(root):
    """Two hull-conforming packs filling the centre bay under the wing carry-through."""
    x0, x1 = interior.B3
    z0, z1 = ZF + 0.001, 0.040
    pb = PB("Battery_Pack")
    for sy in (-1, 1):
        rect = (0.003, 1.0, z0, z1) if sy > 0 else (-1.0, -0.003, z0, z1)
        interior.conform(pb, x0, x1, rect, "cell")
        for k in range(1, 4):
            interior.strap(pb, x0 + (x1 - x0) * k / 4, 0.010, rect, "dark")
        pb.box((0.004, 0.030, 0.016), loc=(x1 + 0.002, sy * 0.040, -0.030), mat="dark")
        PORTS[f"bat_{'r' if sy > 0 else 'l'}"] = Vector((x1 + 0.0025, sy * 0.030, -0.030))
    # velcro straps across both packs
    for cx in (x0 + 0.05, x1 - 0.05):
        interior.strap(pb, cx, 0.020, (-1.0, 1.0, z0, z1), "dark", grow=0.0025)
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Battery Pack", "Power", (0.05, 0, 0.40),
                                      "2 x 6S 44 Ah Li-ion packs, hull-conforming, ~2 kWh: 4 h+ endurance. "
                                      "Located on the centre of gravity under the wing.", 44))


def build_power(root):
    """Power distribution board + ESC on the aft tray."""
    pb = PB("Power_Distribution")
    x = 0.27
    z = ZT
    pb.box((0.050, 0.050, 0.0018), loc=(x, 0, z + 0.006), mat="pcb")
    for sx_ in (-1, 1):
        for sy in (-1, 1):
            pb.cyl(0.002, 0.006, loc=(x + sx_ * 0.020, sy * 0.020, z + 0.003), mat="gold", seg=6)
    pb.box((0.040, 0.010, 0.0012), loc=(x, 0.012, z + 0.0075), mat="copper")
    pb.box((0.040, 0.010, 0.0012), loc=(x, -0.012, z + 0.0075), mat="copper")
    pb.cyl(0.006, 0.014, loc=(x - 0.012, 0, z + 0.0145), mat="dark", seg=12)
    pb.cyl(0.006, 0.014, loc=(x + 0.004, 0, z + 0.0145), mat="dark", seg=12)
    pb.box((0.008, 0.016, 0.007), loc=(x - 0.031, 0, z + 0.010), mat="yellow")   # XT90 input
    PORTS["pdb_in"] = Vector((x - 0.036, 0, z + 0.010))
    PORTS["pdb_out"] = Vector((x + 0.024, 0, z + 0.009))
    PORTS["pdb_fwd"] = Vector((x - 0.024, 0.02, z + 0.009))
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Power Distribution", "Power", (0.0, 0, 0.30),
                                      "Power distribution board with current/voltage sensor, 5 V/12 V BECs "
                                      "and main XT90 connector.", 45))


def build_esc(root):
    pb = PB("ESC")
    x = 0.355
    z = ZT
    pb.box((0.075, 0.034, 0.012), loc=(x, 0, z + 0.006), mat="wblue", bevel=0.003)
    pb.box((0.060, 0.030, 0.003), loc=(x, 0, z + 0.0135), mat="alu")
    for k in range(9):
        pb.box((0.060, 0.0012, 0.008), loc=(x, -0.013 + k * 0.00325, z + 0.019), mat="alu")
    for k in range(2):
        pb.cyl(0.005, 0.016, loc=(x - 0.044, -0.008 + k * 0.016, z + 0.006), rot=(0, 90, 0), mat="dark", seg=10)
    PORTS["esc_in"] = Vector((x - 0.040, 0, z + 0.006))
    PORTS["esc_out"] = Vector((x + 0.038, 0, z + 0.006))
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("ESC", "Propulsion", (0.0, 0, 0.26),
                                      "80 A electronic speed controller with aluminium heat-sink.", 53))


def build_servos(root):
    pb = PB("Tail_Servos")
    x = 0.445
    z = ZT
    for sy in (-1, 1):
        y = sy * 0.0135
        pb.box((0.032, 0.012, 0.0025), loc=(x, y, z + 0.016), mat="dark")
        pb.box((0.023, 0.012, 0.024), loc=(x, y, z + 0.012), mat="dark", bevel=0.0015)
        pb.cyl(0.0035, 0.004, loc=(x + 0.006, y, z + 0.026), mat="alu", seg=8)
        pb.box((0.004, 0.018, 0.0015), loc=(x + 0.006, y + sy * 0.004, z + 0.0285), mat="white")
    # pushrods to the tail
    zax = dl.zc(0.86)
    rods = [((x + 0.006, 0.0215, ZT + 0.029), (0.60, 0.010, zax), (0.78, 0.004, zax - 0.004)),     # elevator
            ((x + 0.006, -0.0215, ZT + 0.029), (0.60, -0.006, zax + 0.010), (0.78, 0.0, zax + 0.034))]  # rudder
    for p0, pm, p1 in rods:
        pb.tube([p0, pm, p1], 0.0015, mat="carbon", seg=5, steps=4)
        pb.box((0.008, 0.003, 0.004), loc=p1, mat="dark")
    PORTS["servo_t"] = Vector((x - 0.012, 0, z + 0.012))
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Tail Servos", "Avionics", (0.0, 0, 0.24),
                                      "Elevator and rudder servos with carbon pushrods running through the tail boom.",
                                      46))


def build_bridge_units(root):
    """Telemetry radio + GNSS receiver hung under a carbon bridge below the hatch."""
    zb = 0.080
    pb = PB("Datalink_Radio")
    x = -0.235
    pb.box((0.062, 0.042, 0.014), loc=(x, 0, zb - 0.010), mat="dark", bevel=0.002)
    for k in range(6):
        pb.box((0.050, 0.0015, 0.006), loc=(x, -0.016 + k * 0.0064, zb - 0.020), mat="alu")
    pb.cyl(0.004, 0.012, loc=(x - 0.034, 0.012, zb - 0.010), rot=(0, 90, 0), mat="gold", seg=8)
    PORTS["radio_rf"] = Vector((x - 0.040, 0.012, zb - 0.010))
    PORTS["radio_io"] = Vector((x + 0.032, 0, zb - 0.012))
    radio = pb.finish(parent=root, smooth_angle=35,
                      props=part_props("Datalink Radio", "Avionics", (0.0, 0, 0.30),
                                       "Encrypted long-range telemetry / command radio modem (up to 30 km).", 47))
    pb = PB("GNSS_Receiver")
    x = -0.355
    pb.box((0.050, 0.040, 0.0016), loc=(x, 0, zb - 0.006), mat="pcb")
    pb.box((0.030, 0.026, 0.006), loc=(x, 0, zb - 0.0098), mat="alu")
    pb.cyl(0.003, 0.010, loc=(x - 0.022, 0.012, zb - 0.010), rot=(0, 90, 0), mat="gold", seg=8)
    for k in range(3):
        jst(pb, (x + 0.022, -0.012 + k * 0.010, zb - 0.0095), 3)
    # carbon bridge plate with standoffs (belongs to this unit)
    pb.box((0.25, 0.098, 0.003), loc=(-0.30, 0, zb + 0.0015), mat="carbon")
    for px in (-0.41, -0.19):
        for sy in (-1, 1):
            pb.box((0.010, 0.006, 0.020), loc=(px, sy * 0.046, zb - 0.008), mat="carbon")
    PORTS["gnss_io"] = Vector((x + 0.026, 0, zb - 0.0095))
    PORTS["gnss_rf"] = Vector((x - 0.026, 0.012, zb - 0.010))
    rx = pb.finish(parent=root, smooth_angle=35,
                   props=part_props("GNSS Receiver", "Avionics", (0.0, 0, 0.30),
                                    "RTK/PPK GNSS receiver board on the avionics bridge, directly under its antenna.",
                                    48))
    return [radio, rx]


def build_aux(root):
    """Nose equipment shelf (SSD, airspeed sensor, BEC, buzzer), RC receiver and safety switch."""
    pb = PB("Aux_Electronics")
    zs = -0.002
    x0, x1 = sx(0.075), sx(0.135)
    xm = 0.5 * (x0 + x1)
    hw = half_width_at(0.10, zs, inset=dl.SHELL_T + 0.004)
    pb.box((x1 - x0, 2 * hw, 0.003), loc=(xm, 0, zs - 0.0015), mat="carbon")
    # SSD
    pb.box((0.055, 0.038, 0.008), loc=(xm - 0.008, -0.030, zs + 0.004), mat="silver", bevel=0.0015)
    pb.box((0.030, 0.020, 0.0006), loc=(xm - 0.008, -0.030, zs + 0.0083), mat="dark")
    # airspeed sensor board with two barbs
    pb.box((0.030, 0.026, 0.0016), loc=(xm + 0.010, 0.032, zs + 0.0008), mat="pcb")
    chip(pb, (xm + 0.010, 0.032, zs + 0.0016), (0.010, 0.008), 0.004, "dark")
    for k in (-1, 1):
        pb.cyl(0.0018, 0.010, loc=(xm + 0.022, 0.032 + k * 0.004, zs + 0.004), rot=(0, 90, 0), mat="white", seg=8)
    # BEC (blue heat-shrink) + buzzer
    pb.box((0.030, 0.016, 0.010), loc=(xm + 0.014, 0.0, zs + 0.005), mat="wblue", bevel=0.002)
    pb.cyl(0.007, 0.008, loc=(xm - 0.030, 0.026, zs + 0.004), mat="dark", seg=14)
    PORTS["shelf_pwr"] = Vector((xm + 0.030, 0.0, zs + 0.005))
    PORTS["shelf_ssd"] = Vector((xm + 0.020, -0.030, zs + 0.004))
    PORTS["shelf_as"] = Vector((xm + 0.027, 0.032, zs + 0.004))
    # RC receiver on top of the avionics bridge, with its two antenna leads
    zb = 0.0815
    xr = -0.27
    pb.box((0.036, 0.024, 0.010), loc=(xr, 0.0, zb + 0.005), mat="dark", bevel=0.0015)
    # antenna leads run forward along the centre lane above the bridge, whips spread in a shallow V
    for sy in (1, -1):
        pb.tube([(xr - 0.018, sy * 0.006, zb + 0.008), (xr - 0.030, sy * 0.009, zb + 0.016), (xr - 0.048, sy * 0.011, zb + 0.022)],
                0.0008, mat="dark", seg=4, steps=3)
        pb.cyl(0.0012, 0.026, loc=(xr - 0.061, sy * 0.012, zb + 0.023), rot=(0, 90, sy * 4), mat="white", seg=6)
    PORTS["rc"] = Vector((xr + 0.018, 0.0, zb + 0.005))
    # safety switch on the spine, just aft of the hatch
    xs_ = -0.155
    s = dl.s_of_x(xs_)
    zt = zc(s) + wh(s)[1]
    pb.cyl(0.011, 0.006, loc=(xs_, 0, zt - 0.001), mat="dark", seg=16)
    pb.cyl(0.0085, 0.003, loc=(xs_, 0, zt + 0.0035), mat="red", seg=16)
    pb.cyl(0.010, 0.010, loc=(xs_, 0, zt - 0.009), mat="dark", seg=12)
    PORTS["safety"] = Vector((xs_, 0, zt - 0.014))
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Auxiliary Electronics", "Avionics", (-0.10, 0, 0.30),
                                      "Nose shelf with SSD storage, differential airspeed sensor, 5 V BEC and buzzer; "
                                      "backup RC receiver; arming safety switch on the spine.", 49))


def build(root):
    return [build_imu(root), build_fc(root), build_dpu(root), build_battery(root), build_power(root),
            build_esc(root), build_servos(root), *build_bridge_units(root), build_aux(root)]
