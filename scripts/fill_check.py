"""Measure how much of the fuselage interior is occupied by parts (voxel sampling). Run inside Blender.

Interior = inside the inner skin of the fuselage (inset by the shell thickness), whole length.
A sample counts as filled when it lies inside any closed part other than the shell / panels / hatch.
Prints the overall fraction plus a per-station profile, and stores the empty voxels in EMPTY.
"""
import sys
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

SCRIPTS = r"D:\My files\G\proiecte\261006_Blender and CCode\survey_drone\scripts"
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)
import dl

STEP = globals().get("STEP", 0.008)
SKIN = {"Fuselage_Shell", "Access_Panel_R", "Access_Panel_L", "Top_Hatch"}


def inside_hull(p):
    s = dl.s_of_x(p.x)
    if not 0.0 <= s <= 1.0:
        return False
    return abs(p.y) < dl.half_width_at(s, p.z, inset=dl.SHELL_T)


def islands(me):
    """Faces grouped by connected component (each closed sub-shell is tested on its own)."""
    parent = list(range(len(me.vertices)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for e in me.edges:
        a, b = find(e.vertices[0]), find(e.vertices[1])
        if a != b:
            parent[a] = b
    groups = {}
    for p in me.polygons:
        groups.setdefault(find(p.vertices[0]), []).append(tuple(p.vertices))
    return list(groups.values())


def trees():
    dg = bpy.context.evaluated_depsgraph_get()
    out = []
    for o in dl.collection().all_objects:
        if o.type != "MESH" or o.name in SKIN:
            continue
        ev = o.evaluated_get(dg)
        me = ev.to_mesh()
        mw = o.matrix_world
        vs = [mw @ v.co for v in me.vertices]
        for polys in islands(me):
            idx = {i for p in polys for i in p}
            if len(idx) < 4:
                continue
            t = BVHTree.FromPolygons(vs, polys)
            lo = Vector((min(vs[i][k] for i in idx) for k in range(3)))
            hi = Vector((max(vs[i][k] for i in idx) for k in range(3)))
            out.append((o.name, t, lo, hi))
        ev.to_mesh_clear()
    return out


def filled(p, T):
    for name, t, lo, hi in T:
        if lo.x <= p.x <= hi.x and lo.y <= p.y <= hi.y and lo.z <= p.z <= hi.z:
            loc, nrm, _, _ = t.find_nearest(p)
            if loc is not None and (p - loc).dot(nrm) < 0:
                return name
    return None


def containing(p, T):
    out = []
    for name, t, lo, hi in T:
        if lo.x <= p.x <= hi.x and lo.y <= p.y <= hi.y and lo.z <= p.z <= hi.z:
            loc, nrm, _, _ = t.find_nearest(p)
            if loc is not None and (p - loc).dot(nrm) < 0 and (p - loc).length > 0.0015:
                out.append(name)
    return out


def overlaps(step=0.004, x0=-0.9, x1=0.9):
    """Part pairs that intersect inside the hull (voxels more than 1.5 mm deep in both)."""
    T = trees()
    pairs = {}
    x = x0 + step / 2
    while x < x1:
        y = -dl.W0 + step / 2
        while y < dl.W0:
            z = -0.16 + step / 2
            while z < 0.18:
                p = Vector((x, y, z))
                if inside_hull(p):
                    c = containing(p, T)
                    for i in range(len(c)):
                        for j in range(i + 1, len(c)):
                            k = tuple(sorted((c[i], c[j])))
                            a = pairs.setdefault(k, [0, p])
                            a[0] += 1
                z += step
            y += step
        x += step
    for k, (n, p) in sorted(pairs.items(), key=lambda kv: -kv[1][0]):
        print(f"  overlap {k[0]} / {k[1]}: {n * step ** 3 * 1e6:.1f} cm3 near ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})")
    return pairs


def run(step=STEP, bins=0.05):
    T = trees()
    n_in = n_fill = 0
    prof = {}
    empty = []
    x = dl.X0 + step / 2
    while x < dl.X0 + dl.L:
        y = -dl.W0 + step / 2
        while y < dl.W0:
            z = -0.16 + step / 2
            while z < 0.18:
                p = Vector((x, y, z))
                if inside_hull(p):
                    n_in += 1
                    b = int((x - dl.X0) / bins)
                    a = prof.setdefault(b, [0, 0])
                    a[0] += 1
                    if filled(p, T):
                        n_fill += 1
                        a[1] += 1
                    else:
                        empty.append(p)
                z += step
            y += step
        x += step
    v = step ** 3
    print(f"interior {n_in * v * 1e3:.2f} L, filled {n_fill * v * 1e3:.2f} L -> {100 * n_fill / max(n_in, 1):.1f}%")
    for b in sorted(prof):
        a = prof[b]
        x0 = dl.X0 + b * bins
        print(f"  x {x0:+.2f}..{x0 + bins:+.2f}  vol {a[0] * v * 1e3:5.2f} L  fill {100 * a[1] / a[0]:5.1f}%")
    return n_fill / max(n_in, 1), empty


if __name__ != "fill_check_lib":
    FRACTION, EMPTY = run()
    if globals().get("OVERLAPS"):
        overlaps()
