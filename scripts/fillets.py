"""Aft root fillets on the wing and tailplane.

The leading-edge blends are part of the surfaces themselves (dl.blend_ext). These fairings cover the
rear of the root only: the airfoil stretched aft by a quarter-ellipse in plan, so the outline runs
tangent to the fuselage at the hull and to the trailing edge where it dies into the surface. Every
section is pressed onto the hull: its span coordinate is measured from the fuselage skin.
"""
import math
from mathutils import Vector
import dl
from dl import PB, part_props, airfoil_loop
import wings
import tail

INNER = -0.003      # inner end sits inside the shell thickness
C_START = 0.42      # fairings start aft of the crest, where the surface below keeps its plain section
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


def stretched(c, er, c_end, m, p, t, n, th=1.0):
    """Rear of an airfoil of chord c lengthened by er behind, same absolute thickness / camber.
    c_end < 1 stops the profile at that fraction of the original chord (control-surface hinge)."""
    c2 = c + er
    k = c / c2
    loop = airfoil_loop(C_START * k, (c * c_end + er) / c2, n, m=m * k, p=p * k, t=t * k * th)
    return loop, c2


def wing_fairing(pb, side):
    ER, WR = 0.13, 0.075
    secs = []
    for t in span_stations(WR):
        d = max(0.0, wings.D_HULL + t)
        c = wings.chord(d)
        f = wings.foil(d)
        loop, c2 = stretched(c, ext(t, ER, WR), 1.0, f["m"], f["p"], f["t"], wings.NPTS, thick(t, WR))
        y, z, phi = wings.frame(d)
        pr = math.radians(phi)
        pts = dl.place_section(loop, (wings.x_le(d), y, z), c2, wings.twist(d),
                               nrm=(0, -math.sin(pr), math.cos(pr)))
        secs.append([Vector((q.x, side * (dl.hull_side(q.x, q.z) + t), q.z)) for q in pts])
    pb.loft(secs, "skin", cap0=True, cap1=True)      # normals are recalculated in finish()


def stab_fairing(pb, side):
    geo_le = Vector((0.66, 0.02, dl.zc(0.86) + 0.005))
    span, rc, tc, sweep = 0.42, 0.20, 0.12, 0.08
    WF, ER, WR = 0.045, 0.035, 0.012
    d_hull = dl.hull_side(geo_le.x + 0.04, geo_le.z) - geo_le.y
    secs = []
    for t in span_stations(WF):
        d = max(0.0, d_hull + t)
        k = d / span
        c = dl.lerp(rc, tc, k)
        x_le = geo_le.x + sweep * k
        # aft of the hinge the elevator starts 3 cm out: only the innermost stations keep the full chord
        full = t <= 0.012
        er = ext(t, ER, WR) if full else 0.0
        loop, c2 = stretched(c, er, 1.0 if full else tail.HINGE - 0.004, 0.0, 0.4, 0.10, 14, thick(t, WF))
        pts = dl.place_section(loop, (x_le, 0.0, geo_le.z), c2, 0.0, nrm=(0, 0, 1))
        secs.append([Vector((q.x, side * (dl.hull_side(q.x, q.z) + t), q.z)) for q in pts])
    pb.loft(secs, "skin", cap0=True, cap1=True)      # normals are recalculated in finish()


def build(root):
    pb = PB("Wing_Fairings")
    for side in (1, -1):
        wing_fairing(pb, side)
    w = pb.finish(parent=root, smooth_angle=40,
                  props=part_props("Wing Root Fairings", "Airframe", (0, 0, 0),
                                   "Composite fillets blending the wing trailing edges into the fuselage to cut "
                                   "interference drag.", 10))
    pb = PB("Tail_Fairings")
    for side in (1, -1):
        stab_fairing(pb, side)
    t = pb.finish(parent=root, smooth_angle=40,
                  props=part_props("Tail Root Fairings", "Tail Assembly", (0, 0, 0),
                                   "Fillets at the rear of the tailplane roots.", 35))
    return [w, t]
