"""Root fillets: fairings that blend the wing, tailplane, fin and ventral fin into the fuselage.

Each fairing is the root airfoil stretched forward (and aft where no control surface moves) by a
quarter-ellipse in plan, so the outline runs tangent to the fuselage at the hull and tangent to the
leading / trailing edge where it dies into the surface. Every section is pressed onto the hull:
its span coordinate is measured from the fuselage skin, so the fairing follows the tapering tail.
"""
import math
from mathutils import Vector
import dl
from dl import PB, part_props, airfoil_loop
import wings
import tail

INNER = -0.003      # inner end sits inside the shell thickness
# thickness relative to the surface: the fairing stays slightly proud of the wing / fin all the way out and
# ends in a seam, like a real bonded fairing (two skins crossing at a shallow angle render as a saw-tooth)
THICK_ROOT, THICK_TIP = 1.04, 1.02


def thick(t, W):
    return dl.lerp(THICK_ROOT, THICK_TIP, min(max(t / W, 0.0), 1.0))


def ext(t, E, W):
    """Concave quarter-ellipse: E at the hull (t = 0), 0 with zero slope at t = W."""
    if E <= 0 or W <= 0 or t >= W:
        return 0.0
    if t <= 0:
        return E
    u = 1 - t / W
    return E * (1 - math.sqrt(max(0.0, 1 - u * u)))


def span_stations(W):
    n = dl.segs(12, 6)
    return [INNER] + [W * (1 - math.cos(math.pi / 2 * i / n)) for i in range(n + 1)]


def hull_side(x, z):
    return dl.half_width_at(dl.s_of_x(x), z)


def hull_top(x, y, sign):
    s = dl.s_of_x(x)
    w, h = dl.wh(s)
    n = dl.N_TOP if sign > 0 else dl.N_BOT
    r = min(abs(y) / w, 0.999)
    return dl.zc(s) + sign * h * (1 - r ** n) ** (1 / n)


def stretched(c, ef, er, c_end, m, p, t, n, th=1.0):
    """Airfoil of chord c lengthened by ef in front and er behind, same absolute thickness / camber.
    c_end < 1 stops the profile at that fraction of the original chord (control-surface hinge)."""
    c2 = c + ef + er
    k = c / c2
    end = (ef + c * c_end + er) / c2
    loop = airfoil_loop(0.0, end, n, m=m * k, p=(ef + p * c) / c2, t=t * k * th)
    return loop, c2


def wing_fairing(pb, side):
    EF, WF, ER, WR = 0.11, 0.065, 0.13, 0.075
    d_hull = (hull_side(wings.ROOT_LE.x, wings.ROOT_LE.z) - wings.ROOT_LE.y) / math.cos(math.radians(wings.PHI0))
    secs = []
    for t in span_stations(max(WF, WR)):
        d = max(0.0, d_hull + t)
        c = wings.chord(d)
        f = wings.foil(d)
        ef, er = ext(t, EF, WF), ext(t, ER, WR)
        loop, c2 = stretched(c, ef, er, 1.0, f["m"], f["p"], f["t"], wings.NPTS, thick(t, max(WF, WR)))
        y, z, phi = wings.frame(d)
        pr = math.radians(phi)
        pts = dl.place_section(loop, (wings.x_le(d) - ef, y, z), c2, wings.twist(d),
                               nrm=(0, -math.sin(pr), math.cos(pr)))
        secs.append([Vector((q.x, side * (hull_side(q.x, q.z) + t), q.z)) for q in pts])
    pb.loft(secs, "skin", cap0=True, cap1=True)      # normals are recalculated in finish()


def stab_fairing(pb, side):
    geo_le = Vector((0.66, 0.02, dl.zc(0.86) + 0.005))
    span, rc, tc, sweep = 0.42, 0.20, 0.12, 0.08
    EF, WF, ER, WR = 0.075, 0.045, 0.035, 0.012
    d_hull = hull_side(geo_le.x + 0.04, geo_le.z) - geo_le.y
    secs = []
    for t in span_stations(WF):
        d = max(0.0, d_hull + t)
        k = d / span
        c = dl.lerp(rc, tc, k)
        x_le = geo_le.x + sweep * k
        # aft of the hinge the elevator starts 3 cm out: only the innermost stations keep the full chord
        full = t <= 0.012
        ef, er = ext(t, EF, WF), (ext(t, ER, WR) if full else 0.0)
        loop, c2 = stretched(c, ef, er, 1.0 if full else tail.HINGE - 0.004, 0.0, 0.4, 0.10, 14, thick(t, WF))
        pts = dl.place_section(loop, (x_le - ef, 0.0, geo_le.z), c2, 0.0, nrm=(0, 0, 1))
        secs.append([Vector((q.x, side * (hull_side(q.x, q.z) + t), q.z)) for q in pts])
    pb.loft(secs, "skin", cap0=True, cap1=True)      # normals are recalculated in finish()


def keel_fairing(pb, sign):
    """Fin (sign +1, on top of the tail boom) or ventral fin (sign -1, underneath)."""
    if sign > 0:
        x0, span, rc, tc, sweep = 0.60, 0.27, 0.24, 0.11, 0.20
        EF, WF, ER, WR, c_end = 0.17, 0.06, 0.0, 0.0, tail.HINGE - 0.004     # rudder stays free
    else:
        x0, span, rc, tc, sweep = 0.70, 0.12, 0.18, 0.10, 0.09
        EF, WF, ER, WR, c_end = 0.11, 0.035, 0.012, 0.012, 1.0
    d_hull = 0.02
    secs = []
    for t in span_stations(max(WF, WR)):
        d = max(0.0, d_hull + t)
        k = d / span
        c = dl.lerp(rc, tc, k)
        x_le = x0 + sweep * k
        ef, er = ext(t, EF, WF), ext(t, ER, WR)
        loop, c2 = stretched(c, ef, er, c_end, 0.0, 0.4, 0.10, 14, thick(t, max(WF, WR)))
        pts = dl.place_section(loop, (x_le - ef, 0.0, 0.0), c2, 0.0, nrm=(0, 1, 0))
        secs.append([Vector((q.x, q.y, hull_top(q.x, q.y, sign) + sign * t)) for q in pts])
    pb.loft(secs, "skin", cap0=True, cap1=True)


def build(root):
    pb = PB("Wing_Fairings")
    for side in (1, -1):
        wing_fairing(pb, side)
    w = pb.finish(parent=root, smooth_angle=40,
                  props=part_props("Wing Root Fairings", "Airframe", (0, 0, 0),
                                   "Composite fillets blending the wing roots into the fuselage to cut "
                                   "interference drag.", 10))
    pb = PB("Tail_Fairings")
    for side in (1, -1):
        stab_fairing(pb, side)
    keel_fairing(pb, 1)
    keel_fairing(pb, -1)
    t = pb.finish(parent=root, smooth_angle=40,
                  props=part_props("Tail Root Fairings", "Tail Assembly", (0, 0, 0),
                                   "Fillets at the tailplane, fin and ventral-fin roots.", 35))
    return [w, t]
