"""Main wings: skin, control surfaces, ribs, spars, servos, winglets."""
import math
from mathutils import Vector
import dl
from dl import PB, part_props, airfoil_loop

ROOT_LE = Vector((-0.144, 0.07, 0.055))
D_STRAIGHT = 1.23
ARC_R = 0.07
PHI0, PHI1 = 3.0, 78.0
WINGLET_LEN = 0.12
SPLIT = 0.74          # chordwise hinge line
NPTS = 22
BLEND_E, BLEND_W = 0.62, 1.08     # leading-edge blend: 0.62 m ahead of the root at the hull, gone 1.08 m out
PRESS_W = 0.12                    # the skin follows the hull at the root and is plain beyond this
INNER = -0.003                    # the root sits inside the shell thickness


def arc_len():
    return ARC_R * math.radians(PHI1 - PHI0)


D_END = D_STRAIGHT + arc_len() + WINGLET_LEN


def frame(d):
    """Section frame at span distance d: base point (y,z), dihedral angle phi (deg)."""
    p0 = math.radians(PHI0)
    if d <= D_STRAIGHT:
        y = ROOT_LE.y + d * math.cos(p0)
        z = ROOT_LE.z + d * math.sin(p0)
        return y, z, PHI0
    yb = ROOT_LE.y + D_STRAIGHT * math.cos(p0)
    zb = ROOT_LE.z + D_STRAIGHT * math.sin(p0)
    # arc centre is "above-inboard" of the bend point
    cy = yb - ARC_R * math.sin(p0)
    cz = zb + ARC_R * math.cos(p0)
    a_len = arc_len()
    if d <= D_STRAIGHT + a_len:
        phi = PHI0 + math.degrees((d - D_STRAIGHT) / ARC_R)
        pr = math.radians(phi)
        return cy + ARC_R * math.sin(pr), cz - ARC_R * math.cos(pr), phi
    pr = math.radians(PHI1)
    ye = cy + ARC_R * math.sin(pr)
    ze = cz - ARC_R * math.cos(pr)
    r = d - D_STRAIGHT - a_len
    return ye + r * math.cos(pr), ze + r * math.sin(pr), PHI1


def x_le(d):
    if d <= D_STRAIGHT:
        return ROOT_LE.x + 0.105 * d
    return ROOT_LE.x + 0.105 * D_STRAIGHT + 0.55 * (d - D_STRAIGHT)


def chord(d):
    if d <= D_STRAIGHT:
        return dl.lerp(0.34, 0.17, d / D_STRAIGHT)
    return dl.lerp(0.17, 0.085, (d - D_STRAIGHT) / (D_END - D_STRAIGHT))


def foil(d):
    k = max(0.0, (d - D_STRAIGHT) / (D_END - D_STRAIGHT))
    return dict(m=0.03 * (1 - k), p=0.4, t=dl.lerp(0.12, 0.08, k))


def twist(d):
    return -2.0 * min(d / D_STRAIGHT, 1.0)


HULL_Y = dl.hull_side(ROOT_LE.x, ROOT_LE.z)
D_HULL = (HULL_Y - ROOT_LE.y) / math.cos(math.radians(PHI0))


def section(d, c0, c1, n=NPTS, tscale=1.0, shrink=0.0, blend=False):
    y, z, phi = frame(d)
    f = foil(d)
    f["t"] *= tscale
    loop = airfoil_loop(c0, c1, n, **f)
    if shrink:
        cx = sum(p[0] for p in loop) / len(loop)
        cz = sum(p[1] for p in loop) / len(loop)
        loop = [(cx + (p[0] - cx) * shrink, cz + (p[1] - cz) * shrink) for p in loop]
    t = d - D_HULL
    if blend:
        loop = dl.stretch_nose(loop, dl.blend_ext(t, BLEND_E, BLEND_W) / chord(d))
    pr = math.radians(phi)
    nrm = (0, -math.sin(pr), math.cos(pr))
    pts = dl.place_section(loop, (x_le(d), y, z), chord(d), twist(d), nrm=nrm)
    if blend and t < PRESS_W:
        fade = 1 - dl.smoothstep(max(t, 0.0) / PRESS_W)
        pts = [Vector((q.x, q.y + (dl.hull_side(q.x, q.z) - HULL_Y) * fade, q.z)) for q in pts]
    return pts


def stations(d0, d1, step=0.06):
    n = max(1, int(math.ceil((d1 - d0) / step * dl.DETAIL)))
    return [d0 + (d1 - d0) * k / n for k in range(n + 1)]


def all_stations(d0, d1):
    """Stations with a fine step around the winglet bend."""
    out = []
    a, b = D_STRAIGHT, D_STRAIGHT + arc_len()
    for lo, hi, st in ((d0, min(d1, a), 0.07), (max(d0, a), min(d1, b), 0.018),
                       (max(d0, b), d1, 0.04)):
        if hi > lo + 1e-6:
            out += stations(lo, hi, st)
    res = []
    for d in out:
        if not res or d - res[-1] > 1e-5:
            res.append(d)
    return res


BLUE_ROOT = 0.30
BLUE_TIP = 1.05


def skin_lofts(pb, c0, c1, d0, d1, cap0=False, cap1=False, colour_rule=True):
    """Loft chord range c0..c1 over span d0..d1, painting blue bands."""
    ds = sorted(set(all_stations(d0, d1)) | {D_HULL + t for t in dl.blend_stations(INNER, BLEND_W)})
    cuts = sorted({d for d in (BLUE_ROOT, BLUE_TIP) if d0 < d < d1})
    segs = []
    edges = [d0] + cuts + [d1]
    for a, b in zip(edges[:-1], edges[1:]):
        seg = [a] + [d for d in ds if a < d < b] + [b]
        segs.append(seg)
    for k, seg in enumerate(segs):
        mid = 0.5 * (seg[0] + seg[-1])
        mat = "skin"
        if colour_rule and (mid < BLUE_ROOT or mid > BLUE_TIP):
            mat = "blue"
        pb.loft([section(d, c0, c1, blend=True) for d in seg], mat,
                cap0=cap0 and k == 0, cap1=cap1 and k == len(segs) - 1)


def mirror(pb, side):
    if side < 0:
        dl.mirror_y_faces(pb, list(pb.bm.faces))


def build_side(root, side):
    S = "R" if side > 0 else "L"
    sname = "Right" if side > 0 else "Left"
    # ---- skin with fixed trailing pieces
    pb = PB(f"Wing_{S}")
    skin_lofts(pb, 0.0, SPLIT, D_HULL + INNER, D_END, cap1=True)
    mirror(pb, side)
    wing = pb.finish(parent=root, smooth_angle=30, solidify=0.0025, recalc=True,
                     props=part_props(f"{sname} Wing", "Wings", (0, side * 0.62, 0.05),
                                      "Composite wing skin, 3.1 m span, NACA 4412-type section, leading edge blended into the fuselage, upturned winglet.",
                                      20 + (side > 0)))
    _ensure_outward(wing)
    # fixed trailing-edge strips (solid, too thin to shell)
    te = PB("te")
    te.loft([section(d, SPLIT, 1.0, n=10) for d in all_stations(0.035, 0.20)], "blue", cap0=True, cap1=True)
    te.loft([section(d, SPLIT, 1.0, n=10) for d in all_stations(1.245, D_END)], "blue", cap0=True, cap1=True)
    # navigation light at the winglet root leading edge (built on +Y, mirrored below): red left, green right
    yl, zl, _ = frame(D_STRAIGHT - 0.01)
    te.disc(0.0065, 0.004, (x_le(D_STRAIGHT - 0.01) + 0.004, yl, zl + 0.004), (-1, 0, 0),
            mat="green" if side > 0 else "red", seg=12, r2=0.004)
    mirror(te, side)
    bmesh_recalc(te)
    dl.merge_pb(wing, te, smooth_angle=30)

    # ---- control surfaces
    for nm, lab, d0, d1, ex in (("Flap", "Flap", 0.205, 0.62, (0.10, 0, -0.06)),
                                 ("Aileron", "Aileron", 0.625, 1.24, (0.12, 0, -0.05))):
        cs = PB(f"{nm}_{S}")
        cs.loft([section(d, SPLIT + 0.006, 1.0, n=10) for d in stations(d0, d1, 0.08)], "skin",
                cap0=True, cap1=True)
        # hinge pin
        mirror(cs, side)
        o = cs.finish(parent=wing, smooth_angle=30,
                      props=part_props(f"{lab} ({S})", "Wings", (ex[0], side * ex[1], ex[2]),
                                       f"{lab} control surface on the {sname.lower()} wing.", 22))

    # ---- internal structure
    st = PB(f"Wing_Structure_{S}")
    rib_ds = [0.02] + [0.10 + k * 0.11 for k in range(11)] + [D_STRAIGHT - 0.005]
    for d in rib_ds:
        if d > D_STRAIGHT:
            continue
        b = d > 0.05       # ribs outside the hull reach forward into the blend
        outer0 = section(d - 0.0015, 0.02, SPLIT - 0.012, n=14, tscale=0.82, blend=b)
        outer1 = section(d + 0.0015, 0.02, SPLIT - 0.012, n=14, tscale=0.82, blend=b)
        inner1 = section(d + 0.0015, 0.02, SPLIT - 0.012, n=14, tscale=0.82, shrink=0.62, blend=b)
        inner0 = section(d - 0.0015, 0.02, SPLIT - 0.012, n=14, tscale=0.82, shrink=0.62, blend=b)
        st.loft([outer0, outer1, inner1, inner0], "beige", wrap=True, smooth=False)
    # spars: rectangular tubes following the section
    for xc, w in ((0.25, 0.012), (0.70, 0.008)):
        secs = []
        for d in stations(0.0, D_STRAIGHT, 0.1):
            y, z, phi = frame(d)
            f = foil(d)
            up, lo = dl.naca(xc, **f)
            c = chord(d)
            h_up = up[1] * 0.80
            h_lo = lo[1] * 0.80
            loop = [(xc - w / c / 2, h_up), (xc + w / c / 2, h_up), (xc + w / c / 2, h_lo), (xc - w / c / 2, h_lo)]
            pr = math.radians(phi)
            secs.append(dl.place_section(loop, (x_le(d), y, z), c, twist(d), nrm=(0, -math.sin(pr), math.cos(pr))))
        st.loft(secs, "carbon", cap0=True, cap1=True, smooth=False)
    # servos + pushrods for flap and aileron
    for d, cs_d in ((0.42, 0.42), (0.93, 0.93)):
        y, z, phi = frame(d)
        c = chord(d)
        xs = x_le(d) + 0.56 * c
        st.box((0.022, 0.012, 0.024), loc=(xs, y, z + 0.002), mat="dark", bevel=0.0015, seg=1)
        st.box((0.030, 0.012, 0.002), loc=(xs, y, z + 0.012), mat="dark")
        st.cyl(0.0035, 0.004, loc=(xs + 0.006, y, z - 0.012), mat="alu", seg=8)
        # pushrod to the control horn below the surface
        p0 = Vector((xs + 0.006, y + 0.006, z - 0.012))
        p1 = Vector((x_le(d) + 0.80 * c, y + 0.006, z - 0.020))
        st.tube([p0, p1], 0.0012, mat="alu", seg=5, smooth_path=False)
        st.box((0.006, 0.0015, 0.014), loc=(x_le(d) + 0.81 * c, y + 0.006, z - 0.016), mat="dark")
        # servo cable along the spar to the root
        pts = [Vector((xs - 0.012, y, z + 0.002))]
        for dd in (d * 0.7, d * 0.4, 0.05, 0.0):
            yy, zz, _ = frame(dd)
            pts.append(Vector((x_le(dd) + 0.25 * chord(dd) + 0.012, yy, zz + 0.006)))
        pts.append(Vector((pts[-1].x, 0.03, pts[-1].z - 0.01)))
        st.tube(pts, 0.0016, mat="dark", seg=5, steps=3)
    # wing root fitting plate
    y, z, _ = frame(0.0)
    st.box((0.24, 0.004, 0.026), loc=(ROOT_LE.x + 0.17, y + 0.004, z + 0.004), mat="alu")
    mirror(st, side)
    st.finish(parent=wing, smooth_angle=35,
              props=part_props(f"Wing Structure ({S})", "Wings", (0, 0, -0.16),
                               "Balsa/ply ribs, carbon main and rear spars, flap and aileron servos with pushrods.",
                               23))

    # ---- pitot probe under the left wing, nav light on both tips
    if side < 0:
        pp = PB("Pitot_Probe")
        y, z, _ = frame(0.85)
        xl = x_le(0.85)
        pp.box((0.05, 0.008, 0.03), loc=(xl + 0.06, y, z - 0.025), mat="skin", bevel=0.003)
        pp.tube([(xl + 0.07, y, z - 0.035), (xl - 0.05, y, z - 0.035)], 0.003, mat="alu", seg=8, smooth_path=False)
        pp.cyl(0.003, 0.012, loc=(xl - 0.055, y, z - 0.035), rot=(0, 90, 0), r2=0.0015, mat="alu", seg=8)
        mirror(pp, side)
        pp.finish(parent=wing, smooth_angle=35,
                  props=part_props("Pitot Probe", "Avionics", (0, 0, -0.08),
                                   "Air-speed pitot-static probe feeding the flight controller.", 40))
    return wing


def bmesh_recalc(pb):
    import bmesh
    bmesh.ops.remove_doubles(pb.bm, verts=pb.bm.verts, dist=1e-6)
    bmesh.ops.recalc_face_normals(pb.bm, faces=pb.bm.faces)


def _ensure_outward(obj):
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    if bm.calc_volume(signed=True) < 0:
        bmesh.ops.reverse_faces(bm, faces=bm.faces)
        bm.to_mesh(obj.data)
    bm.free()


def build(root):
    return [build_side(root, 1), build_side(root, -1)]
