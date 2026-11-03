"""Survey drone build library: materials, part builder, lofts, profiles.

Blender axes: nose toward -X, +Y = right wing, +Z = up. Metres.
"""
import bpy, bmesh, math
from mathutils import Vector, Matrix, Euler

COLL = "SurveyDrone"
DETAIL = 1.0          # global tessellation factor (1.0 = HD build, ~0.5 = lite LOD)


def segs(n, lo=4):
    return max(lo, int(round(n * DETAIL)))

# flat colour palette (base colour / metallic / roughness only: no textures)
PALETTE = {
    "white":   ((0.68, 0.70, 0.73, 1), 0.0, 0.42),
    "blue":    ((0.042, 0.044, 0.048, 1), 0.0, 0.92),    # accent bands: same dark matte as the skin; key kept for the builders
    "skin":    ((0.042, 0.044, 0.048, 1), 0.0, 0.92),     # exterior: dark matte composite
    "grey":    ((0.45, 0.47, 0.50, 1), 0.2, 0.5),
    "alu":     ((0.72, 0.74, 0.77, 1), 0.85, 0.32),
    "dark":    ((0.035, 0.037, 0.04, 1), 0.1, 0.55),
    "carbon":  ((0.09, 0.09, 0.10, 1), 0.3, 0.45),
    "pcb":     ((0.05, 0.38, 0.13, 1), 0.0, 0.45),
    "chip":    ((0.06, 0.06, 0.07, 1), 0.0, 0.4),
    "gold":    ((0.85, 0.62, 0.22, 1), 0.9, 0.3),
    "copper":  ((0.80, 0.42, 0.18, 1), 0.9, 0.35),
    "red":     ((0.80, 0.05, 0.04, 1), 0.0, 0.5),
    "wblue":   ((0.05, 0.25, 0.85, 1), 0.0, 0.5),
    "yellow":  ((0.98, 0.70, 0.05, 1), 0.0, 0.45),
    "cell":    ((0.08, 0.38, 0.95, 1), 0.0, 0.3),
    "glass":   ((0.04, 0.30, 0.32, 1), 0.3, 0.08),
    "lens":    ((0.22, 0.10, 0.45, 1), 0.4, 0.05),
    "beige":   ((0.86, 0.80, 0.66, 1), 0.0, 0.6),
    "orange":  ((0.95, 0.36, 0.05, 1), 0.0, 0.5),
    "rubber":  ((0.02, 0.02, 0.02, 1), 0.0, 0.8),
    "green":   ((0.02, 0.75, 0.15, 1), 0.0, 0.3),
    "silver":  ((0.55, 0.57, 0.60, 1), 0.7, 0.35),
    "foam":    ((0.30, 0.32, 0.36, 1), 0.0, 0.9),
}


def get_mat(name):
    m = bpy.data.materials.get("D_" + name)
    if m is None:
        m = bpy.data.materials.new("D_" + name)
        m.use_nodes = True
    col, metal, rough = PALETTE[name]
    bsdf = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = col
    bsdf.inputs["Metallic"].default_value = metal
    bsdf.inputs["Roughness"].default_value = rough
    m.diffuse_color = col
    m.metallic = metal
    m.roughness = rough
    return m


def collection():
    c = bpy.data.collections.get(COLL)
    if c is None:
        c = bpy.data.collections.new(COLL)
        bpy.context.scene.collection.children.link(c)
    return c


def clear_all():
    c = bpy.data.collections.get(COLL)
    if c:
        for o in list(c.all_objects):
            bpy.data.objects.remove(o, do_unlink=True)
    for me in list(bpy.data.meshes):
        if me.users == 0:
            bpy.data.meshes.remove(me)


def remove_tree(name):
    o = bpy.data.objects.get(name)
    if not o:
        return
    for ch in list(o.children_recursive):
        bpy.data.objects.remove(ch, do_unlink=True)
    bpy.data.objects.remove(o, do_unlink=True)


def M(loc=(0, 0, 0), rot=(0, 0, 0), scale=(1, 1, 1)):
    return Matrix.LocRotScale(Vector(loc), Euler([math.radians(a) for a in rot]), Vector(scale))


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def lerp(a, b, t):
    return a + (b - a) * t


def catmull(points, steps=6):
    """Smooth polyline through control points (Catmull-Rom)."""
    P = [Vector(p) for p in points]
    if len(P) < 3:
        out = []
        for i in range(len(P) - 1):
            for k in range(steps):
                out.append(P[i].lerp(P[i + 1], k / steps))
        out.append(P[-1])
        return out
    P = [P[0] * 2 - P[1]] + P + [P[-1] * 2 - P[-2]]
    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        for k in range(steps):
            t = k / steps
            t2, t3 = t * t, t * t * t
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
    out.append(P[-2])
    return out


class PB:
    """Accumulates geometry for one part into a single bmesh with material slots."""

    def __init__(self, name):
        self.name = name
        self.bm = bmesh.new()
        self.mats = []

    def mi(self, mat):
        if mat not in self.mats:
            self.mats.append(mat)
        return self.mats.index(mat)

    def _tag(self, verts, mat, smooth=True):
        idx = self.mi(mat)
        faces = {f for v in verts for f in v.link_faces}
        for f in faces:
            f.material_index = idx
            f.smooth = smooth
        return faces

    # ---- primitives -------------------------------------------------
    def box(self, size, loc=(0, 0, 0), rot=(0, 0, 0), mat="white", bevel=0.0, seg=1):
        r = bmesh.ops.create_cube(self.bm, size=1.0, matrix=M(loc, rot, size))
        verts = r["verts"]
        if bevel > 0:
            edges = list({e for v in verts for e in v.link_edges})
            bmesh.ops.bevel(self.bm, geom=edges + list(verts), offset=bevel, segments=seg,
                            profile=0.5, affect="EDGES", clamp_overlap=True)
            verts = self._grow(verts)
        return self._tag(verts, mat)

    def _grow(self, verts):
        # after bevel, collect the connected island
        seen = set()
        stack = [v for v in verts if v.is_valid]
        while stack:
            v = stack.pop()
            if v in seen:
                continue
            seen.add(v)
            for e in v.link_edges:
                o = e.other_vert(v)
                if o not in seen:
                    stack.append(o)
        return list(seen)

    def cyl(self, r, h, loc=(0, 0, 0), rot=(0, 0, 0), mat="alu", seg=16, r2=None, caps=True):
        res = bmesh.ops.create_cone(self.bm, cap_ends=caps, cap_tris=False, segments=segs(seg, 6),
                                    radius1=r, radius2=r if r2 is None else r2, depth=h,
                                    matrix=M(loc, rot))
        return self._tag(res["verts"], mat)

    def disc(self, r, h, loc, normal, mat="dark", seg=8, r2=None):
        """Short cylinder whose axis follows `normal`, base centred at loc (screw heads, lights)."""
        n = Vector(normal).normalized()
        m = Matrix.Translation(Vector(loc) + n * (h / 2)) @ n.to_track_quat("Z", "Y").to_matrix().to_4x4()
        res = bmesh.ops.create_cone(self.bm, cap_ends=True, cap_tris=False, segments=segs(seg, 4), radius1=r,
                                    radius2=r if r2 is None else r2, depth=h, matrix=m)
        return self._tag(res["verts"], mat)

    def sphere(self, r, loc=(0, 0, 0), rot=(0, 0, 0), mat="white", u=16, v=8, scale=(1, 1, 1)):
        res = bmesh.ops.create_uvsphere(self.bm, u_segments=segs(u, 6), v_segments=segs(v, 4), radius=r,
                                        matrix=M(loc, rot, scale))
        return self._tag(res["verts"], mat)

    def loft(self, sections, mat="white", closed=True, cap0=False, cap1=False, wrap=False, smooth=True):
        """sections: list of point lists (same length). closed: ring is a loop.
        wrap: connect last section back to first (torus-like ring solids)."""
        bm = self.bm
        rows = [[bm.verts.new(Vector(p)) for p in sec] for sec in sections]
        n = len(rows[0])
        faces = []
        nsec = len(rows)
        span = nsec if wrap else nsec - 1
        for j in range(span):
            a, b = rows[j], rows[(j + 1) % nsec]
            rng = n if closed else n - 1
            for i in range(rng):
                i2 = (i + 1) % n
                vs = [a[i], a[i2], b[i2], b[i]]
                if len({id(v) for v in vs}) < 4:
                    continue
                try:
                    faces.append(bm.faces.new(vs))
                except ValueError:
                    pass
        if cap0:
            faces.append(bm.faces.new(list(reversed(rows[0]))))
        if cap1:
            faces.append(bm.faces.new(rows[-1]))
        idx = self.mi(mat)
        for f in faces:
            f.material_index = idx
            f.smooth = smooth
        return faces, rows

    def tube(self, points, r, mat="red", seg=6, steps=5, smooth_path=True, caps=True):
        """Sweep a circle along a (smoothed) path. Used for wires, rods, pipes."""
        seg = segs(seg, 4)
        pts = catmull(points, segs(steps, 2)) if smooth_path else [Vector(p) for p in points]
        # drop duplicates
        clean = [pts[0]]
        for p in pts[1:]:
            if (p - clean[-1]).length > 1e-5:
                clean.append(p)
        pts = clean
        if len(pts) < 2:
            return
        tangents = []
        for i in range(len(pts)):
            a = pts[max(i - 1, 0)]
            b = pts[min(i + 1, len(pts) - 1)]
            tangents.append((b - a).normalized())
        # parallel transport frame
        t0 = tangents[0]
        up = Vector((0, 0, 1)) if abs(t0.z) < 0.9 else Vector((1, 0, 0))
        nrm = t0.cross(up).normalized()
        secs = []
        for i, p in enumerate(pts):
            t = tangents[i]
            nrm = (nrm - t * nrm.dot(t))
            if nrm.length < 1e-6:
                nrm = t.orthogonal()
            nrm.normalize()
            bin_ = t.cross(nrm)
            ring = []
            for k in range(seg):
                a = 2 * math.pi * k / seg
                ring.append(p + (nrm * math.cos(a) + bin_ * math.sin(a)) * r)
            secs.append(ring)
        return self.loft(secs, mat, closed=True, cap0=caps, cap1=caps)

    def lathe(self, prof, mats="alu", n=32, axis="Z", center=(0, 0, 0), wrap=False, smooth=True):
        """Solid of revolution. prof: [(h, r), ...]; ends with r == 0 become poles (closed solid).
        wrap=True treats the profile as a closed loop (rings, tyres). mats: name, list per segment,
        or callable(h0, r0, h1, r1) -> name."""
        bm = self.bm
        cx, cy, cz = center
        n = segs(n, 6)

        def P(h, r, k):
            a = 2 * math.pi * k / n
            c, s = math.cos(a) * r, math.sin(a) * r
            if axis == "Z":
                return Vector((cx + c, cy + s, h))
            if axis == "X":
                return Vector((h, cy + c, cz + s))
            return Vector((cx + c, h, cz + s))

        rings = []
        for h, r in prof:
            if r <= 1e-7:
                rings.append([bm.verts.new(P(h, 0, 0))] * n)
            else:
                rings.append([bm.verts.new(P(h, r, k)) for k in range(n)])
        nseg = len(prof) if wrap else len(prof) - 1
        faces = []
        for j in range(nseg):
            a, b = rings[j], rings[(j + 1) % len(rings)]
            (h0, r0), (h1, r1) = prof[j], prof[(j + 1) % len(prof)]
            if callable(mats):
                m = mats(h0, r0, h1, r1)
            elif isinstance(mats, (list, tuple)):
                m = mats[j]
            else:
                m = mats
            idx = self.mi(m)
            for k in range(n):
                k2 = (k + 1) % n
                vs = [a[k], a[k2], b[k2], b[k]]
                uniq = []
                for v in vs:
                    if v not in uniq:
                        uniq.append(v)
                if len(uniq) < 3:
                    continue
                try:
                    f = bm.faces.new(uniq)
                except ValueError:
                    continue
                f.material_index = idx
                f.smooth = smooth
                faces.append(f)
        return faces

    def ring_solid(self, outer, inner, x0, x1, mat="carbon"):
        """Flat frame between two closed outlines (lists of (y,z)) extruded along X."""
        o0 = [(x0, y, z) for y, z in outer]
        o1 = [(x1, y, z) for y, z in outer]
        i1 = [(x1, y, z) for y, z in inner]
        i0 = [(x0, y, z) for y, z in inner]
        return self.loft([o0, o1, i1, i0], mat, closed=True, wrap=True, smooth=False)

    def plate(self, outline, z0, z1, mat="carbon", axis="Z"):
        """Prism from a closed outline (list of (a,b)) between two heights along axis."""
        def P(a, b, h):
            if axis == "Z":
                return (a, b, h)
            if axis == "X":
                return (h, a, b)
            return (a, h, b)
        s0 = [P(a, b, z0) for a, b in outline]
        s1 = [P(a, b, z1) for a, b in outline]
        return self.loft([s0, s1], mat, closed=True, cap0=True, cap1=True, smooth=False)

    def transform_faces(self, faces, mat4):
        vs = {v for f in faces for v in f.verts}
        bmesh.ops.transform(self.bm, matrix=mat4, verts=list(vs))

    # ---- finish -----------------------------------------------------
    def finish(self, parent=None, smooth_angle=32, origin="center", solidify=0.0,
               props=None, recalc=True, location=None):
        bm = self.bm
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
        if recalc:
            bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        me = bpy.data.meshes.new(self.name)
        bm.to_mesh(me)
        bm.free()
        for m in self.mats:
            me.materials.append(get_mat(m))
        obj = bpy.data.objects.new(self.name, me)
        collection().objects.link(obj)
        if solidify:
            mod = obj.modifiers.new("solid", "SOLIDIFY")
            mod.thickness = solidify
            mod.offset = -1
            mod.use_even_offset = True
            mod.use_quality_normals = True
            dg = bpy.context.evaluated_depsgraph_get()
            new = bpy.data.meshes.new_from_object(obj.evaluated_get(dg))
            obj.modifiers.clear()
            obj.data = new
            bpy.data.meshes.remove(me)
            me = new
            me.name = self.name
        me.shade_smooth()
        if smooth_angle is not None:
            me.set_sharp_from_angle(angle=math.radians(smooth_angle))
        # origin
        if origin == "center" or location is not None:
            if location is not None:
                c = Vector(location)
            else:
                xs = [v.co for v in me.vertices]
                lo = Vector((min(p.x for p in xs), min(p.y for p in xs), min(p.z for p in xs)))
                hi = Vector((max(p.x for p in xs), max(p.y for p in xs), max(p.z for p in xs)))
                c = (lo + hi) / 2
            me.transform(Matrix.Translation(-c))
            obj.location = c
        if parent is not None:
            set_parent(obj, parent)
        if props:
            for k, v in props.items():
                obj[k] = v
        return obj


def merge_pb(obj, pb, smooth_angle=32):
    """Append the geometry of a PB (world coords) into an existing object's mesh."""
    me = obj.data
    names = [m.name[2:] for m in me.materials]
    remap = {}
    for i, m in enumerate(pb.mats):
        if m not in names:
            me.materials.append(get_mat(m))
            names.append(m)
        remap[i] = names.index(m)
    for f in pb.bm.faces:
        f.material_index = remap[f.material_index]
        f.smooth = True
    inv = world(obj).inverted()
    bmesh.ops.transform(pb.bm, matrix=inv, verts=pb.bm.verts)
    tmp = bpy.data.meshes.new("tmp")
    pb.bm.to_mesh(tmp)
    pb.bm.free()
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.from_mesh(tmp)
    bm.to_mesh(me)
    bm.free()
    bpy.data.meshes.remove(tmp)
    me.shade_smooth()
    me.set_sharp_from_angle(angle=math.radians(smooth_angle))


def world(o):
    """World matrix computed from basis (matrix_world is stale until a depsgraph update)."""
    if o.parent is None:
        return o.matrix_basis.copy()
    return world(o.parent) @ o.matrix_parent_inverse @ o.matrix_basis


def set_parent(child, parent):
    mw = world(child)
    child.parent = parent
    child.matrix_parent_inverse = Matrix.Identity(4)
    child.matrix_basis = world(parent).inverted() @ mw


def empty(name, loc=(0, 0, 0), parent=None, props=None, size=0.05):
    o = bpy.data.objects.new(name, None)
    o.empty_display_size = size
    o.empty_display_type = "PLAIN_AXES"
    o.location = loc
    collection().objects.link(o)
    if parent is not None:
        set_parent(o, parent)
    if props:
        for k, v in props.items():
            o[k] = v
    return o


def part_props(label, group, explode, desc="", order=0):
    return {"label": label, "group": group, "explode": list(explode), "desc": desc, "order": order}


# ---------------------------------------------------------------------
# Fuselage profile
# ---------------------------------------------------------------------
L = 1.80
X0 = -0.90
W0 = 0.105
H0 = 0.125
N_TOP = 2.3
N_BOT = 3.0
SHELL_T = 0.004


def sx(s):
    return X0 + L * s


def s_of_x(x):
    return (x - X0) / L


def f_nose(s):
    t = min(max(s / 0.2, 0.0), 1.0)
    return (1 - (1 - t) ** 2.2) ** (1 / 2.2)


def f_tail(s):
    if s <= 0.55:
        return 1.0
    t = (s - 0.55) / 0.45
    return 1 - 0.68 * smoothstep(t)


def zc(s):
    t = max(0.0, (s - 0.5) / 0.5)
    return 0.035 * smoothstep(t) - 0.012 * (1 - f_nose(s))


def wh(s):
    k = f_nose(s) * f_tail(s)
    return W0 * k, H0 * k


def sp(c, e):
    return math.copysign(abs(c) ** e, c)


def fus_point(s, th, inset=0.0):
    """th in radians, 0 = +Y side, pi/2 = top."""
    w, h = wh(s)
    w = max(w - inset, 1e-4)
    h = max(h - inset, 1e-4)
    c, si = math.cos(th), math.sin(th)
    n = N_TOP if si >= 0 else N_BOT
    y = w * sp(c, 2 / n)
    z = zc(s) + h * sp(si, 2 / n)
    return Vector((sx(s), y, z))


def fus_outline(s, inset=0.0, n=48, scale=1.0):
    """(y,z) outline of the cross-section at s, optionally shrunk."""
    out = []
    z0 = zc(s)
    for k in range(n):
        th = 2 * math.pi * k / n
        p = fus_point(s, th, inset)
        out.append((p.y * scale, z0 + (p.z - z0) * scale))
    return out


def half_width_at(s, z, inset=0.0):
    w, h = wh(s)
    w -= inset
    h -= inset
    dz = z - zc(s)
    n = N_TOP if dz >= 0 else N_BOT
    r = abs(dz) / h
    if r >= 1:
        return 0.0
    return w * (1 - r ** n) ** (1 / n)


# ---------------------------------------------------------------------
# Airfoils
# ---------------------------------------------------------------------
def naca(xc, m=0.03, p=0.4, t=0.12):
    yt = 5 * t * (0.2969 * math.sqrt(max(xc, 0)) - 0.1260 * xc - 0.3516 * xc ** 2 + 0.2843 * xc ** 3 - 0.1015 * xc ** 4)
    if m == 0:
        return (xc, yt), (xc, -yt)
    if xc < p:
        yc = m / p ** 2 * (2 * p * xc - xc ** 2)
        dy = 2 * m / p ** 2 * (p - xc)
    else:
        yc = m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * xc - xc ** 2)
        dy = 2 * m / (1 - p) ** 2 * (p - xc)
    th = math.atan(dy)
    up = (xc - yt * math.sin(th), yc + yt * math.cos(th))
    lo = (xc + yt * math.sin(th), yc - yt * math.cos(th))
    return up, lo


def airfoil_loop(c0=0.0, c1=1.0, n=18, m=0.03, p=0.4, t=0.12):
    """Closed 2D loop (chordwise x, thickness z) of the airfoil between chord stations c0..c1.
    Starts at the upper trailing point and runs forward over the top, back under the bottom."""
    n = segs(n, 6)
    if c0 <= 1e-6:
        xs = [c1 * (1 - math.cos(math.pi / 2 * i / (n - 1))) for i in range(n)]
        xs = [c1 * (0.5 * (1 - math.cos(math.pi * i / (n - 1)))) for i in range(n)]
    else:
        xs = [c0 + (c1 - c0) * i / (n - 1) for i in range(n)]
    ups = [naca(x, m, p, t)[0] for x in xs]
    los = [naca(x, m, p, t)[1] for x in xs]
    if c0 <= 1e-6:
        return list(reversed(ups)) + los[1:]
    return list(reversed(ups)) + los


def place_section(loop2d, le, chord, twist_deg=0.0, nrm=(0, 0, 1), chord_dir=(1, 0, 0), pivot=0.25):
    """Map 2D airfoil loop to 3D. le: leading edge point."""
    le = Vector(le)
    cd = Vector(chord_dir).normalized()
    nv = Vector(nrm).normalized()
    a = math.radians(twist_deg)
    out = []
    for x, z in loop2d:
        x0 = x - pivot
        xr = x0 * math.cos(a) + z * math.sin(a)
        zr = -x0 * math.sin(a) + z * math.cos(a)
        xr += pivot
        out.append(le + cd * (xr * chord) + nv * (zr * chord))
    return out


def mirror_y_faces(pb, faces):
    vs = {v for f in faces for v in f.verts}
    for v in vs:
        v.co.y = -v.co.y
    bmesh.ops.reverse_faces(pb.bm, faces=list(faces))
