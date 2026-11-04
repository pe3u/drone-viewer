"""Packed interior: hull-conforming modules filling the bays between bulkheads.

Deck layout (avionics bays, z in metres):
  belly    hull bottom .. -0.078   range-extender battery modules under the floor
  lower    floor .. -0.016         existing units + foam cradles; cable ducts along the floor corners
  tray G1  -0.0155 .. -0.0055      harness tray (horizontal cable runs)
  mid      -0.0045 .. 0.050        payload computer, storage
  gap G2   0.049 .. 0.057          cable runs under the bridge units
  upper    0.051 .. hull top       satcom, fan unit, parachute, payload battery, foam
Cable risers stay open at x = -0.345..-0.333 (A), -0.170..-0.143 (B, centre lane) and -0.068..-0.057 (C).
Measure the result with fill_check.py.
"""
import math
from mathutils import Vector
import dl
from dl import PB, part_props, sx

# bulkhead-limited bays (1 mm clear of each bulkhead face)
NOSE = (-0.893, -0.651)
B1 = (-0.645, -0.291)
B2 = (-0.285, -0.057)
B3 = (-0.051, 0.231)
B4 = (0.237, 0.501)
TAIL = (0.507, 0.86)

MARGIN = 0.001            # clearance to the inner skin
BUS = (0.046, -0.054)     # floor-corner cable ducts: |y| > 0.046 and z < -0.054
TRAY = (-0.0155, -0.0055)
INF = 1.0


def longerons(s):
    out = []
    for side in (1, -1):
        p = dl.fus_point(s, math.radians(40 if side > 0 else 140), inset=dl.SHELL_T + 0.004)
        out.append((p.y, p.z))
    return out


def bus_keep(x):
    """Floor-corner cable channels exist along the floor and the aft tray."""
    return -0.47 < x < 0.30


def free_fn(s, x, keep, margin, bus, longeron_r):
    lon = longerons(s) if 0.10 <= s <= 0.86 else []
    use_bus = bus and bus_keep(x)

    def free(y, z):
        if abs(y) >= dl.half_width_at(s, z, inset=dl.SHELL_T + margin):
            return False
        for ly, lz in lon:
            if (y - ly) ** 2 + (z - lz) ** 2 < longeron_r ** 2:
                return False
        if use_bus and abs(y) > BUS[0] and z < BUS[1]:
            return False
        for y0, y1, z0, z1 in keep:
            if y0 < y < y1 and z0 < z < z1:
                return False
        return True
    return free


def clamp_rect(s, rect, margin):
    y0, y1, z0, z1 = rect
    w, h = dl.wh(s)
    w -= dl.SHELL_T + margin
    h -= dl.SHELL_T + margin
    zc = dl.zc(s)
    return max(y0, -w), min(y1, w), max(z0, zc - h), min(z1, zc + h)


def radial(free, c, d, rmax):
    """Distance from c along d to the first point that is not free (capped at rmax)."""
    step = 0.0025
    r = 0.0
    while r < rmax:
        r2 = min(r + step, rmax)
        p = c + d * r2
        if not free(p.x, p.y):
            lo, hi = r, r2
            for _ in range(10):
                m = 0.5 * (lo + hi)
                q = c + d * m
                if free(q.x, q.y):
                    lo = m
                else:
                    hi = m
            return lo
        r = r2
    return rmax


def rect_reach(rect, c, d):
    y0, y1, z0, z1 = rect
    t = 1e9
    if d.x > 1e-9:
        t = min(t, (y1 - c.x) / d.x)
    elif d.x < -1e-9:
        t = min(t, (y0 - c.x) / d.x)
    if d.y > 1e-9:
        t = min(t, (z1 - c.y) / d.y)
    elif d.y < -1e-9:
        t = min(t, (z0 - c.y) / d.y)
    return max(t, 0.0)


def conform(pb, x0, x1, rect, mat, keep=(), margin=MARGIN, bus=True, centre=None, n=32, longeron_r=0.013,
            step=0.03):
    """Solid filling rect (y0, y1, z0, z1) ∩ hull interior minus keep-out rects, extruded from x0 to x1.
    Sections are star-shaped around `centre` (y, z); stations follow the hull taper."""
    xs = [x0, x1]
    s0, s1 = dl.s_of_x(x0), dl.s_of_x(x1)
    tapered = min(s0, s1) < 0.2 or max(s0, s1) > 0.55
    if tapered:
        k = max(1, int(math.ceil(abs(x1 - x0) / (step / max(dl.DETAIL, 0.25)))))
        xs = [x0 + (x1 - x0) * i / k for i in range(k + 1)]
    sm = dl.s_of_x(0.5 * (x0 + x1))
    cr = clamp_rect(sm, rect, margin)
    c = Vector(centre) if centre else Vector((0.5 * (cr[0] + cr[1]), 0.5 * (cr[2] + cr[3])))
    nn = dl.segs(n, 12)
    angs = [2 * math.pi * i / nn for i in range(nn)]
    for cy, cz in ((cr[0], cr[2]), (cr[1], cr[2]), (cr[1], cr[3]), (cr[0], cr[3])):
        a = math.atan2(cz - c.y, cy - c.x) % (2 * math.pi)
        if all(abs(a - b) > 0.02 for b in angs):
            angs.append(a)
    angs.sort()
    secs = []
    for x in xs:
        s = dl.s_of_x(x)
        # the section must also fit the neighbouring stations (hull tapers between them)
        s_in = s
        if tapered:
            s_in = min((dl.s_of_x(min(max(x + dx, x0), x1)) for dx in (-0.5 * step, 0.0, 0.5 * step)),
                       key=lambda q: dl.wh(q)[0])
        free = free_fn(s_in, x, keep, margin, bus, longeron_r)
        rc = clamp_rect(s_in, rect, margin)
        if rc[1] - rc[0] < 0.002 or rc[3] - rc[2] < 0.002:
            continue                      # hull too narrow here: drop the station
        cs = c
        if centre is None and tapered:   # follow the tapering hull so the centre stays inside
            q = Vector((0.5 * (rc[0] + rc[1]), 0.5 * (rc[2] + rc[3])))
            cs = q if free(q.x, q.y) else c
        sec = []
        for a in angs:
            d = Vector((math.cos(a), math.sin(a)))
            r = radial(free, cs, d, rect_reach(rc, cs, d))
            p = cs + d * r
            sec.append((x, p.x, p.y))
        if max((Vector(p[1:]) - cs).length for p in sec) < 0.001:
            continue
        secs.append(sec)
    if len(secs) < 2:
        return []
    faces, rows = pb.loft(secs, mat, closed=True, cap0=True, cap1=True)
    for f in faces:
        f.smooth = False
    return faces


def strap(pb, x, w, rect, mat="dark", keep=(), centre=None, grow=0.001, bus=True):
    """Thin band wrapped around a conform() block (cell dividers, velcro straps)."""
    return conform(pb, x - w / 2, x + w / 2, (rect[0] - grow, rect[1] + grow, rect[2] - grow, rect[3] + grow), mat,
                   keep=keep, margin=MARGIN - grow, centre=centre, bus=bus)


def face_ports(pb, x, y, z, kinds, dx=-1):
    """Row of connectors on an end face at x (dx: outward direction)."""
    for k, (dy, w, h, mat) in enumerate(kinds):
        pb.box((0.004, w, h), loc=(x + dx * 0.0018, y + dy, z), mat=mat)


def label(pb, x, y, z, w, h, dx=-1, mat="white"):
    pb.box((0.0008, w, h), loc=(x + dx * 0.0004, y, z), mat=mat)


def zrange(s):
    w, h = dl.wh(s)
    return dl.zc(s) - h, dl.zc(s) + h


# ---------------------------------------------------------------------------
def sides(y0, z0, z1, y1=INF):
    """Right and left rects mirrored about the centreline."""
    return [(y0, y1, z0, z1), (-y1, -y0, z0, z1)]


def build_nose_bumper(root):
    pb = PB("Nose_Bumper")
    conform(pb, NOSE[0], -0.7955, (-INF, INF, -INF, INF), "foam", bus=False, margin=0.001, step=0.012, n=36,
            centre=(0.0, -0.002))
    # tungsten ballast slugs pressed into the back face
    for y in (-0.028, 0.0, 0.028):
        pb.cyl(0.010, 0.020, loc=(-0.806, y, -0.030), rot=(0, 90, 0), mat="silver", seg=14)
    return pb.finish(parent=root, smooth_angle=40,
                     props=part_props("Nose Bumper & Ballast", "Airframe", (-0.30, 0, 0),
                                      "Crushable EPP foam impact absorber for belly / nose-in landings, "
                                      "with tungsten trim ballast slugs.", 6))


def build_lidar_cu(root):
    pb = PB("LiDAR_Control_Unit")
    x0, x1 = -0.7945, NOSE[1]
    rect = (-INF, INF, 0.0135, INF)
    conform(pb, x0, x1, rect, "silver", bus=False, step=0.02)
    for x in (-0.770, -0.678):
        strap(pb, x, 0.006, rect, "dark", bus=False)
    face_ports(pb, x1, 0.0, 0.032, [(-0.036, 0.014, 0.010, "dark"), (-0.018, 0.012, 0.010, "dark"),
                                     (0.004, 0.016, 0.012, "dark"), (0.026, 0.008, 0.008, "gold"),
                                     (0.040, 0.006, 0.006, "alu")], dx=1)
    label(pb, x1, 0.0, 0.062, 0.050, 0.022, dx=1)
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("LiDAR Control Unit", "Payload", (-0.10, 0, 0.42),
                                      "Scanner control and timing electronics: laser driver supply, PPS / NMEA "
                                      "time-sync and Ethernet bridge to the data processing unit.", 62))


def build_payload_computer(root):
    pb = PB("Payload_Computer")
    x0, x1 = B1[0], -0.346
    rect = (-INF, INF, TRAY[1] + 0.001, 0.0505)
    conform(pb, x0, x1, rect, "alu")
    # fin block on the aft face (riser gap) and fan grille
    for k in range(9):
        y = -0.064 + k * 0.016
        pb.box((0.006, 0.003, 0.040), loc=(x1 + 0.0015, y, 0.022), mat="alu")
    pb.cyl(0.018, 0.003, loc=(x1 + 0.0018, 0.0, 0.022), rot=(0, 90, 0), mat="dark", seg=20)
    for x in (x0 + 0.06, x0 + 0.16, x0 + 0.24):
        strap(pb, x, 0.004, rect, "dark")
    face_ports(pb, x0, 0.0, 0.020, [(-0.05, 0.016, 0.012, "dark"), (-0.03, 0.016, 0.012, "dark"),
                                     (0.03, 0.010, 0.006, "alu"), (0.05, 0.012, 0.010, "wblue")], dx=-1)
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Payload Computer", "Avionics", (0.0, 0, 0.72),
                                      "Edge-AI payload computer (GPU module) for on-board point-cloud "
                                      "classification and image tagging, with forced-air cooling.", 50))


def build_satcom(root):
    pb = PB("Satcom_Modem")
    x0, x1 = B1[0], -0.431
    rect = (-INF, INF, 0.0515, INF)
    conform(pb, x0, x1, rect, "dark", step=0.02)
    strap(pb, x0 + 0.05, 0.010, rect, "wblue")
    strap(pb, x1 - 0.05, 0.010, rect, "wblue")
    for y in (-0.030, 0.0, 0.030):
        pb.cyl(0.0045, 0.010, loc=(x1 + 0.005, y, 0.072), rot=(0, 90, 0), mat="gold", seg=10)
    label(pb, x1, 0.0, 0.092, 0.060, 0.014, dx=1)
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Satcom / LTE Modem", "Avionics", (-0.06, 0, 0.95),
                                      "Beyond-line-of-sight link: Iridium Certus + LTE modem for telemetry, "
                                      "RTK corrections and remote tasking.", 51))


def build_storage(root):
    pb = PB("Storage_Array")
    z0 = TRAY[1] + 0.001
    conform(pb, -0.333, B1[1], (-INF, INF, z0, 0.0485), "silver")
    x0, x1 = B2[0], -0.068
    conform(pb, x0, -0.140, (-INF, INF, z0, 0.0485), "silver")
    conform(pb, -0.1394, x1, (-INF, INF, z0, 0.0475), "silver")
    # drive caddies: dark latches on the front face
    for k in range(4):
        y = -0.051 + k * 0.034
        pb.box((0.004, 0.026, 0.008), loc=(x0 - 0.0018, y, 0.014), mat="dark")
        pb.box((0.0008, 0.006, 0.003), loc=(x0 - 0.0040, y + 0.008, 0.026), mat="green")
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Storage & Network Switch", "Avionics", (0.0, 0, 0.82),
                                      "Hot-swap SSD array (2 x 4 TB, mirrored) and gigabit Ethernet switch "
                                      "linking LiDAR, camera, payload computer and DPU.", 52))


WING_ROOT = [(-INF, -0.066, -INF, 0.087), (0.066, INF, -INF, 0.087)]


def build_fan(root):
    pb = PB("Ventilation_Fan")
    x0, x1 = -0.143, -0.071
    conform(pb, x0, x1, (-INF, INF, 0.0485, INF), "grey", keep=WING_ROOT, centre=(0.0, 0.080))
    pb.cyl(0.022, 0.004, loc=(x0 - 0.002, 0.0, 0.092), rot=(0, 90, 0), mat="dark", seg=24)
    for k in range(5):
        pb.box((0.002, 0.018, 0.004), loc=(x0 - 0.0045, 0.0, 0.092), rot=(k * 72, 0, 0), mat="alu")
    pb.cyl(0.006, 0.006, loc=(x0 - 0.005, 0.0, 0.092), rot=(0, 90, 0), mat="alu", seg=12)
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Ventilation Fan Unit", "Avionics", (0.0, 0, 0.70),
                                      "Ducted 12 V blower pulling air from the NACA inlet over the payload "
                                      "computer and DPU heat sinks.", 54))


def build_bms(root):
    pb = PB("Battery_Management")
    for x0, x1 in ((B3[0], 0.090), (0.110, B3[1])):
        conform(pb, x0, x1, (-0.026, 0.026, 0.042, 0.0785), "pcb", bus=False)
        for k in range(int((x1 - x0) / 0.022)):
            pb.box((0.010, 0.040, 0.0015), loc=(x0 + 0.012 + k * 0.022, 0, 0.0790), mat="chip")
        pb.box((0.004, 0.030, 0.012), loc=(x1 + 0.0018, 0, 0.058), mat="beige")
    # cell-monitor boards beside the centre module, between the spar carry-throughs
    for x0, x1 in ((-0.022, 0.090), (0.110, B3[1])):
        for rect in sides(0.0275, 0.042, 0.0785, 0.066):
            conform(pb, x0, x1, rect, "dark", bus=False)
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Battery Management System", "Power", (0.0, 0, 0.62),
                                      "Cell balancing, temperature and state-of-charge monitoring for the main "
                                      "and belly packs, slotted between the wing spar carry-throughs.", 55))


def build_parachute(root):
    pb = PB("Parachute_System")
    x0, x1 = B3
    rect = (-INF, INF, 0.080, INF)
    conform(pb, x0, x1, rect, "orange", keep=WING_ROOT, centre=(0.0, 0.098))
    for x in (x0 + 0.04, 0.5 * (x0 + x1), x1 - 0.04):
        strap(pb, x, 0.008, rect, "dark", keep=WING_ROOT, centre=(0.0, 0.098))
    # pyro ejector on the aft face
    pb.cyl(0.010, 0.004, loc=(x1 - 0.003, 0.0, 0.098), rot=(0, 90, 0), mat="red", seg=16)
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Parachute Recovery System", "Airframe", (0.0, 0, 0.95),
                                      "Ballistic-deployed recovery parachute packed in a canister on the centre "
                                      "of gravity, ejected upward between the wing roots.", 7))


def build_payload_battery(root):
    pb = PB("Payload_Battery")
    rect = (-INF, INF, -0.0065, INF)
    conform(pb, B4[0], 0.420, rect, "cell")
    rect2 = (-INF, INF, 0.012, INF)
    conform(pb, 0.424, B4[1], rect2, "cell")
    for x in (0.27, 0.33, 0.39):
        strap(pb, x, 0.010, rect, "dark")
    strap(pb, 0.46, 0.010, rect2, "dark")
    label(pb, B4[0], 0.0, 0.040, 0.070, 0.030, dx=-1)
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Payload Battery", "Power", (0.10, 0, 0.62),
                                      "Separate 6S 10 Ah pack for the sensors and computers so a payload fault "
                                      "can never brown out the flight systems.", 56))


def build_belly_battery(root):
    pb = PB("Belly_Battery")
    rect = (-INF, INF, -INF, -0.0765)
    for x0, x1 in ((-0.457, B1[1]), B2, B3, (B4[0], 0.287)):
        conform(pb, x0, x1, rect, "cell", bus=False)
        L = x1 - x0
        nk = max(1, int(L / 0.07))
        for k in range(1, nk):
            strap(pb, x0 + L * k / nk, 0.006, rect, "dark", bus=False)
        pb.box((0.0015, 0.024, 0.008), loc=(x0 - 0.0008, 0.03, -0.092), mat="yellow")
    conform(pb, 0.291, B4[1], (-INF, INF, -INF, -0.0525), "cell", bus=False)
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Belly Battery Modules", "Power", (0.0, 0, -0.30),
                                      "Range-extender Li-ion modules under the avionics floor (4 x 6S 8 Ah), "
                                      "adding ~50 % endurance while keeping the CG low.", 57))


def build_remote_id(root):
    pb = PB("Remote_ID_ADSB")
    for rect, mat in zip(sides(0.030, -0.039, -0.0075), ("dark", "grey")):
        sy = 1 if rect[0] > 0 else -1
        conform(pb, B4[0], B4[1], rect, mat, bus=False)
        pb.cyl(0.004, 0.006, loc=(B4[0] - 0.0005, sy * 0.06, -0.027), rot=(0, 90, 0), mat="gold", seg=8)
        label(pb, B4[0], sy * 0.060, -0.020, 0.030, 0.008, dx=-1)
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Remote ID & ADS-B", "Avionics", (0.06, 0, 0.40),
                                      "Broadcast Remote ID module (right) and ADS-B In traffic receiver (left) "
                                      "for detect-and-avoid.", 58))


def build_tail(root):
    pb = PB("Tail_Equipment")
    xm = 0.640
    # ELT above the pushrods / motor phases
    conform(pb, TAIL[0], 0.597, (-INF, INF, 0.0355, INF), "yellow", bus=False)
    conform(pb, 0.5976, xm, (-INF, INF, 0.0355, INF), "yellow", bus=False, keep=[(-0.036, 0.016, 0.054, INF)],
            centre=(0.025, 0.050))
    # trim ballast under the motor phases
    conform(pb, TAIL[0], 0.600, (-INF, INF, -INF, -0.034), "silver", bus=False)
    # air ducts beside the phases, continuing under and over the stabiliser roots
    for rect in sides(0.026, -0.022, 0.0345):
        conform(pb, TAIL[0], xm, rect, "grey", bus=False)
    for rect in sides(0.016, -INF, 0.019):
        conform(pb, xm + 0.003, TAIL[1], rect, "grey", bus=False)
    for rect in sides(0.016, 0.047, INF):
        conform(pb, xm + 0.003, 0.80, rect, "grey", bus=False)
    conform(pb, xm + 0.003, 0.700, (-0.016, 0.016, -INF, -0.006), "silver", bus=False)
    # tail-cone end, either side of the phases and the fin roots
    for rect in sides(0.012, -INF, 0.0205):
        conform(pb, 0.803, 0.826, rect, "grey", bus=False, step=0.012)
    for rect in sides(0.012, 0.0455, INF):
        conform(pb, 0.803, 0.826, rect, "grey", bus=False, step=0.012)
    for rect in sides(0.012, -INF, INF):
        conform(pb, 0.829, 0.872, rect, "grey", bus=False, step=0.012)
    pb.box((0.004, 0.020, 0.010), loc=(TAIL[0] + 0.002, 0.0, 0.052), mat="red")
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Tail Equipment", "Avionics", (0.20, 0, 0.30),
                                      "Emergency locator beacon (yellow), trim ballast and cooling-air ducts in "
                                      "the tail cone.", 59))


def build(root):
    return [build_nose_bumper(root), build_lidar_cu(root), build_payload_computer(root), build_satcom(root),
            build_storage(root), build_fan(root), build_bms(root), build_parachute(root),
            build_payload_battery(root), build_belly_battery(root), build_remote_id(root), build_tail(root)]
