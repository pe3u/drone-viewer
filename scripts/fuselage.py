"""Fuselage shell, removable panels and internal structure."""
import math
import bmesh
from mathutils import Vector
import dl
from dl import PB, sx, fus_point, wh, zc, half_width_at, part_props

NSEG = 64
SIDE_S = (0.06, 0.72)
SIDE_R = (-62, 12)          # degrees, right side (+Y)
SIDE_L = (168, 242)         # left side (-Y)
HATCH_S = (0.16, 0.40)
HATCH_T = (55, 125)
LIDAR_S = (0.075, 0.125)
CAM_S = (0.175, 0.245)
BELLY_T = (245, 295)
BLUE_BANDS = [(0.0, 0.03), (0.745, 0.78)]

BULKHEADS = [0.14, 0.34, 0.47, 0.63, 0.78]


def ring_list():
    nn, nb = dl.segs(20, 10), dl.segs(64, 28)
    S = [0.2 * (1 - math.cos(math.pi / 2 * k / nn)) for k in range(1, nn + 1)]
    S += [0.2 + 0.8 * k / nb for k in range(1, nb + 1)]
    bounds = [b for b in (*SIDE_S, *HATCH_S, *LIDAR_S, *CAM_S, *[a for ab in BLUE_BANDS for a in ab]) if b > 0]
    S = [s for s in S if all(abs(s - b) > 0.004 for b in bounds)] + bounds
    return sorted(set(round(s, 6) for s in S))


def in_ang(th_deg, rng):
    a = th_deg % 360
    lo, hi = rng[0] % 360, rng[1] % 360
    if lo <= hi:
        return lo <= a <= hi
    return a >= lo or a <= hi


def region(s, th):
    d = math.degrees(th)
    if SIDE_S[0] <= s <= SIDE_S[1]:
        if in_ang(d, SIDE_R):
            return "panel_r"
        if in_ang(d, SIDE_L):
            return "panel_l"
    if HATCH_S[0] <= s <= HATCH_S[1] and in_ang(d, HATCH_T):
        return "hatch"
    if (LIDAR_S[0] <= s <= LIDAR_S[1] or CAM_S[0] <= s <= CAM_S[1]) and in_ang(d, BELLY_T):
        return "hole"
    return "shell"


def build_shell(root):
    S = ring_list()
    NSEG = 4 * dl.segs(16, 8)
    ths = [2 * math.pi * j / NSEG for j in range(NSEG)]
    builders = {k: PB(n) for k, n in (("shell", "Fuselage_Shell"), ("panel_r", "Access_Panel_R"),
                                         ("panel_l", "Access_Panel_L"), ("hatch", "Top_Hatch"))}
    cache = {}

    def V(pb, key, co):
        d = cache.setdefault(id(pb), {})
        if key not in d:
            d[key] = pb.bm.verts.new(co)
        return d[key]

    for i in range(len(S) - 1):
        s0, s1 = S[i], S[i + 1]
        sm = 0.5 * (s0 + s1)
        for j in range(NSEG):
            j1 = (j + 1) % NSEG
            thm = (ths[j] + (ths[j1] if j1 else 2 * math.pi)) / 2
            reg = region(sm, thm)
            if reg == "hole":
                continue
            pb = builders[reg]
            vs = [V(pb, (i, j), fus_point(s0, ths[j])), V(pb, (i, j1), fus_point(s0, ths[j1])),
                  V(pb, (i + 1, j1), fus_point(s1, ths[j1])), V(pb, (i + 1, j), fus_point(s1, ths[j]))]
            f = pb.bm.faces.new(vs)
            mat = "skin"
            if reg == "shell" and any(a <= sm <= b for a, b in BLUE_BANDS):
                mat = "blue"
            f.material_index = pb.mi(mat)
            f.smooth = True
    # nose pole fan
    sh = builders["shell"]
    pole = sh.bm.verts.new(fus_point(0.0, 0.0) * 0 + Vector((sx(0.0), 0, zc(0.0))))
    for j in range(NSEG):
        j1 = (j + 1) % NSEG
        a = V(sh, (0, j), fus_point(S[0], ths[j]))
        b = V(sh, (0, j1), fus_point(S[0], ths[j1]))
        f = sh.bm.faces.new([pole, b, a])
        f.material_index = sh.mi("blue")
        f.smooth = True

    # make normals point outward deterministically
    for pb in builders.values():
        for f in pb.bm.faces:
            c = f.calc_center_median()
            s = dl.s_of_x(c.x)
            radial = c - Vector((c.x, 0, zc(min(max(s, 0), 1))))
            if f.normal.dot(radial) < 0:
                f.normal_flip()

    props = {
        "shell": part_props("Fuselage Shell", "Airframe", (0, 0, 0),
                            "Monocoque composite shell, 4 mm skin, removable side and top panels.", 1),
        "panel_r": part_props("Access Panel (R)", "Airframe", (0, 0.40, 0.02),
                              "Right-hand avionics access panel.", 2),
        "panel_l": part_props("Access Panel (L)", "Airframe", (0, -0.40, 0.02),
                              "Left-hand avionics access panel.", 3),
        "hatch": part_props("Top Hatch", "Airframe", (0, 0, 0.42),
                            "Quick-release top hatch carrying the GNSS and datalink antennas.", 4),
    }
    objs = {}
    for k, pb in builders.items():
        objs[k] = pb.finish(parent=root, smooth_angle=40, solidify=dl.SHELL_T, props=props[k], recalc=False)
    # quarter-turn fasteners around each removable panel
    for k, (srng, trng) in (("panel_r", (SIDE_S, SIDE_R)), ("panel_l", (SIDE_S, SIDE_L)), ("hatch", (HATCH_S, HATCH_T))):
        sc = PB("screws")
        for s, th in perimeter(srng, trng, inset_s=0.012, inset_t=4.5, step=0.07):
            p = fus_point(s, th)
            nrm = surface_normal(s, th)
            sc.disc(0.0042, 0.0012, p - nrm * 0.0004, nrm, mat="alu", seg=8)
            sc.disc(0.0012, 0.0004, p + nrm * 0.0008, nrm, mat="dark", seg=4)
        dl.merge_pb(objs[k], sc, smooth_angle=40)
    # belly skids (rubber strips for belly landings) on the shell
    sk = PB("skids")
    for side in (1, -1):
        pts = [fus_point(s, math.radians(270 + side * 22)) for s in [0.30 + 0.02 * i for i in range(17)]]
        sk.tube(pts, 0.005, mat="rubber", seg=6, steps=2)
    dl.merge_pb(objs["shell"], sk, smooth_angle=40)
    return objs


def surface_normal(s, th, e=1e-3):
    p = fus_point(s, th)
    ds = fus_point(min(s + e, 1.0), th) - fus_point(max(s - e, 0.0), th)
    dt = fus_point(s, th + e) - fus_point(s, th - e)
    n = ds.cross(dt).normalized()
    radial = p - Vector((p.x, 0, zc(s)))
    return n if n.dot(radial) > 0 else -n


def perimeter(srng, trng, inset_s, inset_t, step):
    """Points (s, theta) running around a panel outline, inset from its edges."""
    s0, s1 = srng[0] + inset_s, srng[1] - inset_s
    t0, t1 = math.radians(trng[0] + inset_t), math.radians(trng[1] - inset_t)
    if t1 < t0:
        t1 += 2 * math.pi
    out = []
    n_s = max(2, int((s1 - s0) * dl.L / step) + 1)
    for i in range(n_s):
        s = s0 + (s1 - s0) * i / (n_s - 1)
        out += [(s, t0), (s, t1)]
    w = wh(0.4)[0]
    n_t = max(0, int((t1 - t0) * w / step))
    for j in range(1, n_t):
        th = t0 + (t1 - t0) * j / n_t
        out += [(s0, th), (s1, th)]
    return out


def build_structure(root):
    """Bulkheads, avionics floor, longerons, wing joiner tubes."""
    pb = PB("Airframe_Structure")
    t = 0.004
    for s in BULKHEADS:
        x = sx(s)
        outer = dl.fus_outline(s, inset=dl.SHELL_T + 0.0005, n=dl.segs(40, 20))
        inner = dl.fus_outline(s, inset=dl.SHELL_T + 0.0005, n=dl.segs(40, 20), scale=0.80)
        pb.ring_solid(outer, inner, x - t / 2, x + t / 2, mat="carbon")
    # avionics floor (carbon tray) following the hull
    zf = -0.072
    ss = [0.24 + k * (0.66 - 0.24) / 30 for k in range(31)]
    left = []
    right = []
    for s in ss:
        hw = half_width_at(s, zf, inset=dl.SHELL_T + 0.002)
        right.append((sx(s), hw))
        left.append((sx(s), -hw))
    outline = right + list(reversed(left))
    pb.plate(outline, zf - 0.003, zf, mat="carbon")
    # aft equipment tray (PDB, ESC, tail servos)
    zt = -0.048
    ss = [0.645 + k * (0.80 - 0.645) / 16 for k in range(17)]
    right = [(sx(s), half_width_at(s, zt - 0.003, inset=dl.SHELL_T + 0.002)) for s in ss]
    right = [p for p in right if p[1] > 0.006]
    outline = right + [(x, -y) for x, y in reversed(right)]
    pb.plate(outline, zt - 0.003, zt, mat="carbon")
    # longerons (square carbon strips along the upper corners)
    for side in (1, -1):
        pts = []
        for k in range(30):
            s = 0.10 + k * (0.86 - 0.10) / 29
            p = fus_point(s, math.radians(40 if side > 0 else 140), inset=dl.SHELL_T + 0.004)
            pts.append(p)
        pb.tube(pts, 0.004, mat="carbon", seg=4, steps=2)
    # wing joiner tubes
    for x in (-0.06, 0.10):
        pb.cyl(0.008, 0.36, loc=(x, 0, 0.068), rot=(90, 0, 0), mat="carbon", seg=12)
    # wing root sockets
    for x in (-0.06, 0.10):
        for side in (1, -1):
            pb.cyl(0.012, 0.03, loc=(x, side * 0.085, 0.068), rot=(90, 0, 0), mat="alu", seg=12)
    return pb.finish(parent=root, smooth_angle=35,
                     props=part_props("Airframe Structure", "Airframe", (0, 0, 0),
                                      "Carbon bulkheads, avionics floor, longerons and wing joiner tubes.", 5))


def build(root):
    objs = build_shell(root)
    objs["structure"] = build_structure(root)
    return objs
