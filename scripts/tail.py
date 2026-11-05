"""Tail assembly: horizontal stabiliser halves + elevators, vertical fin + rudder, ventral fin."""
import math
from mathutils import Vector
import dl
from dl import PB, part_props, airfoil_loop


def surf_section(root_le, span_dir, nrm, root_c, tip_c, sweep, span, d, c0, c1, t=0.10, n=14, blend=None):
    k = d / span
    le = Vector(root_le) + Vector(span_dir) * d + Vector((sweep * k, 0, 0))
    c = dl.lerp(root_c, tip_c, k)
    loop = airfoil_loop(c0, c1, n, m=0.0, p=0.4, t=t)
    if blend:
        loop = dl.stretch_nose(loop, dl.blend_ext(d - blend["d0"], blend["E"], blend["W"]) / c)
    pts = dl.place_section(loop, le, c, 0.0, nrm=nrm)
    if blend and d - blend["d0"] < blend["press_w"]:
        fade = 1 - dl.smoothstep(max(d - blend["d0"], 0.0) / blend["press_w"])
        pts = [blend["press"](q, fade) for q in pts]
    return pts


def surface(pb, geo, c0, c1, d0, d1, mat_fn, step=0.04, n=14, cap0=True, cap1=True, blend=None):
    step /= dl.DETAIL
    span = geo["span"]
    ds = [d0 + (d1 - d0) * i / max(1, int(math.ceil((d1 - d0) / step))) for i in range(int(math.ceil((d1 - d0) / step)) + 1)]
    if blend:
        ds = sorted(set(ds) | {blend["d0"] + t for t in dl.blend_stations(d0 - blend["d0"], blend["W"])})
    # split into colour bands
    cuts = [c for c in geo.get("cuts", []) if d0 < c < d1]
    edges = [d0] + cuts + [d1]
    for bi, (a, b) in enumerate(zip(edges[:-1], edges[1:])):
        seg = [a] + [d for d in ds if a < d < b] + [b]
        secs = [surf_section(geo["le"], geo["dir"], geo["nrm"], geo["rc"], geo["tc"], geo["sweep"], span, d, c0, c1,
                             geo.get("t", 0.10), n, blend) for d in seg]
        pb.loft(secs, mat_fn(0.5 * (a + b) / span), cap0=cap0 and bi == 0, cap1=cap1 and bi == len(edges) - 2)


HINGE = 0.70
INNER = -0.003      # blended roots sit inside the shell thickness


def side_blend(y0, x_ref, z, side, E, W, press_w):
    """Blended leading edge for a surface leaving the side of the hull; y0 is the root's plain y."""
    ref = dl.hull_side(x_ref, z)
    return dict(E=E, W=W, d0=ref - y0, press_w=press_w,
                press=lambda q, f: Vector((q.x, q.y + side * (dl.hull_side(q.x, q.z) - ref) * f, q.z)))


def keel_blend(x_ref, sign, E, W, press_w, d0=0.02):
    """Same for a fin leaving the top (sign +1) or bottom (-1) of the hull."""
    ref = dl.hull_top(x_ref, 0.0, sign)
    return dict(E=E, W=W, d0=d0, press_w=press_w,
                press=lambda q, f: Vector((q.x, q.y, q.z + (dl.hull_top(q.x, q.y, sign) - ref) * f)))


def build(root):
    s_tail = 0.86
    z_axis = dl.zc(s_tail)
    out = []
    # ---- horizontal stabiliser halves
    for side in (1, -1):
        S = "R" if side > 0 else "L"
        geo = dict(le=(0.66, side * 0.02, z_axis + 0.005), dir=(0, side, 0), nrm=(0, 0, 1),
                   rc=0.20, tc=0.12, sweep=0.08, span=0.42, t=0.10, cuts=[0.42 * 0.82])
        pb = PB(f"H_Stabilizer_{S}")
        bl = side_blend(0.02, 0.70, geo["le"][2], side, 0.27, 0.34, 0.12)
        surface(pb, geo, 0.0, HINGE, bl["d0"] + INNER, geo["span"], lambda k: "blue" if k > 0.82 else "skin", blend=bl)
        o = pb.finish(parent=root, smooth_angle=30,
                      props=part_props(f"Horizontal Stabiliser ({S})", "Tail Assembly", (0.10, side * 0.28, 0.0),
                                       "Symmetric-section tailplane half, plug-in to the tail boom.", 30))
        el = PB(f"Elevator_{S}")
        surface(el, geo, HINGE + 0.01, 1.0, 0.03, geo["span"] - 0.005, lambda k: "blue" if k > 0.82 else "skin", n=8)
        el.finish(parent=o, smooth_angle=30,
                  props=part_props(f"Elevator ({S})", "Tail Assembly", (0.10, 0, 0), "Pitch control surface.", 31))
        out.append(o)
    # ---- vertical fin
    zt = dl.zc(0.80) + dl.wh(0.80)[1] - 0.02
    geo = dict(le=(0.60, 0, zt), dir=(0, 0, 1), nrm=(0, 1, 0), rc=0.24, tc=0.11, sweep=0.20, span=0.27,
               t=0.10, cuts=[0.27 * 0.62])
    pb = PB("Vertical_Fin")
    bl = keel_blend(0.60, 1, 0.24, 0.24, 0.08)
    surface(pb, geo, 0.0, HINGE, bl["d0"] + INNER, geo["span"], lambda k: "blue" if k > 0.62 else "skin", blend=bl)
    # fin fairing / antenna light
    pb.cyl(0.006, 0.02, loc=(0.60 + 0.20 + 0.04, 0, zt + 0.27 + 0.004), mat="red", seg=8)
    fin = pb.finish(parent=root, smooth_angle=30,
                    props=part_props("Vertical Fin", "Tail Assembly", (0.12, 0, 0.30),
                                     "Swept vertical stabiliser with anti-collision light.", 32))
    rd = PB("Rudder")
    surface(rd, geo, HINGE + 0.01, 1.0, 0.03, geo["span"] - 0.005, lambda k: "blue" if k > 0.62 else "skin", n=8)
    rd.finish(parent=fin, smooth_angle=30,
              props=part_props("Rudder", "Tail Assembly", (0.10, 0, 0), "Yaw control surface.", 33))
    # ---- ventral fin
    zb = dl.zc(0.84) - dl.wh(0.84)[1] + 0.02
    geo = dict(le=(0.70, 0, zb), dir=(0, 0, -1), nrm=(0, 1, 0), rc=0.18, tc=0.10, sweep=0.09, span=0.12,
               t=0.10, cuts=[0.12 * 0.7])
    pb = PB("Ventral_Fin")
    bl = keel_blend(0.70, -1, 0.15, 0.105, 0.05)
    surface(pb, geo, 0.0, 1.0, bl["d0"] + INNER, geo["span"], lambda k: "blue" if k > 0.7 else "skin", blend=bl)
    # tail skid
    pb.box((0.08, 0.008, 0.006), loc=(0.70 + 0.09 + 0.05, 0, zb - 0.12 - 0.002), mat="rubber", bevel=0.002)
    pb.finish(parent=root, smooth_angle=30,
              props=part_props("Ventral Fin", "Tail Assembly", (0.12, 0, -0.24),
                               "Lower fin protecting the pusher propeller on belly landings, with rubber skid.", 34))
    return out
