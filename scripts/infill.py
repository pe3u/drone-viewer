"""Automatic foam infill around wires, pushrods and small units (runs after wiring).

Each lane is cut into thin slabs along X. In every slab the y-z bounding boxes of the obstacles
(clipped to the slab) are subtracted from the lane rectangle, and the free rectangles become foam
blocks; identical rectangles in neighbouring slabs are merged into one longer block.
"""
import bpy
from mathutils import Vector
import dl
import interior
from dl import PB, part_props

INF = interior.INF
CLEAR = 0.0015      # clearance kept around obstacles
SEP = 0.0003        # blocks never share faces (each stays a separate closed shell)
MIN = 0.004         # smallest block side worth keeping

# x0, x1, lane rect (y0, y1, z0, z1), obstacle parts, slab length
LANES = [
    # tail cone: phases, pushrods and the tail loom
    (0.507, 0.640, (-INF, INF, -0.0335, 0.035), ["Wiring_Harness", "Tail_Servos", "Tail_Equipment"], 0.010),
    (0.643, 0.880, (-INF, INF, -INF, INF), ["Wiring_Harness", "Tail_Servos", "Tail_Equipment", "Vertical_Fin",
                                            "Ventral_Fin", "H_Stabilizer_L", "H_Stabilizer_R", "Rudder"], 0.010),
    # aft bay: around the tail servos and the PDB connector
    (0.396, 0.501, (-0.0295, 0.0295, -0.0475, -0.012), ["Wiring_Harness", "Tail_Servos", "ESC"], 0.008),
    (0.237, 0.2495, (-0.029, 0.029, -0.0475, -0.0075), ["Wiring_Harness", "Power_Distribution"], 0.006),
    # nose bay: around the scanner feeds below the shelf
    (-0.7015, -0.651, (-INF, INF, -0.037, -0.0065), ["Wiring_Harness", "LiDAR_Sensor", "Foam_Cradles",
                                                    "Aux_Electronics"], 0.008),
    # avionics bay: risers, the bridge centre lane, under-bridge, cable slots
    (-0.3415, -0.333, (-INF, INF, -0.0045, 0.049), ["Wiring_Harness"], 0.006),
    (-0.396, -0.2915, (-0.0195, 0.0195, 0.0855, INF), ["Wiring_Harness", "Aux_Electronics", "GNSS_Antenna"], 0.010),
    (-0.2845, -0.228, (-0.0195, 0.0195, 0.0855, INF), ["Wiring_Harness", "Aux_Electronics", "Comm_Antenna"], 0.010),
    (-0.3845, -0.2915, (-0.027, 0.027, 0.0495, 0.0785), ["Wiring_Harness", "GNSS_Receiver", "Aux_Electronics"],
     0.010),
    (-0.2845, -0.180, (-0.027, 0.027, 0.0495, 0.0785), ["Wiring_Harness", "Datalink_Radio", "Aux_Electronics"],
     0.010),
    (-0.178, -0.1435, (-0.0255, 0.0255, 0.0495, INF), ["Wiring_Harness", "Aux_Electronics"], 0.008),
    (-0.0675, -0.0580, (-INF, INF, -0.0715, 0.0595), ["Wiring_Harness", "Airframe_Structure"], 0.005),
    (-0.487, -0.456, (-0.0195, 0.0195, -0.0715, -0.0165), ["Wiring_Harness", "IMU", "Flight_Controller"], 0.008),
    (-0.341, -0.291, (-0.042, 0.042, -0.0715, -0.0445), ["Wiring_Harness", "Flight_Controller"], 0.010),
    (-0.285, -0.247, (-0.053, 0.053, -0.0715, -0.0405), ["Wiring_Harness", "Data_Processing_Unit", "Cable_Ducts"],
     0.008),
    # camera mount: between the damper plates and under the IMU shelf
    (-0.572, -0.472, (-0.037, 0.037, -0.1030, -0.0820), ["Mapping_Camera", "Wiring_Harness"], 0.010),
    (-0.572, -0.497, (-0.035, 0.035, -0.0785, -0.0535), ["IMU", "Mapping_Camera", "Wiring_Harness"], 0.010),
    # flight controller: between the base plate and the main board
    (-0.452, -0.348, (-0.037, 0.037, -0.0675, -0.0492), ["Flight_Controller", "Wiring_Harness"], 0.010),
]


def edge_cache(names):
    """World-space edges per island for each obstacle part."""
    out = []
    for n in names:
        o = bpy.data.objects.get(n)
        if not o or o.type != "MESH":
            continue
        mw = dl.world(o)
        me = o.data
        vs = [mw @ v.co for v in me.vertices]
        parent = list(range(len(vs)))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a
        for e in me.edges:
            a, b = find(e.vertices[0]), find(e.vertices[1])
            if a != b:
                parent[a] = b
        isl = {}
        for e in me.edges:
            i, j = e.vertices
            isl.setdefault(find(i), []).append((vs[i], vs[j]))
        for edges in isl.values():
            xs = [p.x for e in edges for p in e]
            out.append((min(xs), max(xs), edges))
    return out


def holes_in(cache, a, b):
    """y-z boxes of everything that crosses the slab [a, b]."""
    lo, hi = a - CLEAR, b + CLEAR
    out = []
    for xmin, xmax, edges in cache:
        if xmax < lo or xmin > hi:
            continue
        ys, zs = [], []
        for p, q in edges:
            if max(p.x, q.x) < lo or min(p.x, q.x) > hi:
                continue
            for t in clip(p.x, q.x, lo, hi):
                r = p.lerp(q, t)
                ys.append(r.y)
                zs.append(r.z)
        if ys:
            out.append((min(ys) - CLEAR, max(ys) + CLEAR, min(zs) - CLEAR, max(zs) + CLEAR))
    return out


def clip(x0, x1, lo, hi):
    """Parameters t of the segment x0->x1 limited to [lo, hi] (endpoints of the inside part)."""
    if abs(x1 - x0) < 1e-12:
        return (0.0, 1.0)
    t0, t1 = (lo - x0) / (x1 - x0), (hi - x0) / (x1 - x0)
    t0, t1 = max(0.0, min(t0, t1)), min(1.0, max(t0, t1))
    return (t0, t1)


def rect_minus(rect, holes):
    y0, y1, z0, z1 = rect
    hs = [h for h in holes if h[1] > y0 and h[0] < y1 and h[3] > z0 and h[2] < z1]
    zs = sorted({z0, z1} | {min(max(h[2], z0), z1) for h in hs} | {min(max(h[3], z0), z1) for h in hs})
    out = []
    for za, zb in zip(zs, zs[1:]):
        if zb - za < 1e-6:
            continue
        zm = 0.5 * (za + zb)
        cuts = sorted((h[0], h[1]) for h in hs if h[2] < zm < h[3])
        ya = y0
        segs = []
        for c0, c1 in cuts:
            if c0 > ya:
                segs.append((ya, min(c0, y1)))
            ya = max(ya, c1)
        if ya < y1:
            segs.append((ya, y1))
        for s in segs:
            if s[1] - s[0] <= 1e-6:
                continue
            prev = next((r for r in out if abs(r[0] - s[0]) < 1e-6 and abs(r[1] - s[1]) < 1e-6
                         and abs(r[3] - za) < 1e-6), None)
            if prev:
                out[out.index(prev)] = (prev[0], prev[1], prev[2], zb)
            else:
                out.append((s[0], s[1], za, zb))
    return out


def fits(x0, x1, r):
    """Box fully inside the hull interior (with margin) and clear of the longerons?"""
    for x in (x0, x1):
        s = dl.s_of_x(x)
        free = interior.free_fn(s, x, (), interior.MARGIN, False, 0.013)
        for y in (r[0], r[1]):
            for z in (r[2], r[3]):
                if not free(y, z):
                    return False
        if not free(0.5 * (r[0] + r[1]), r[3]) or not free(0.5 * (r[0] + r[1]), r[2]):
            return False
    return True


def fill_lane(pb, x0, x1, rect, names, step, mat="foam"):
    cache = edge_cache(names)
    n = max(1, round((x1 - x0) / (step / max(dl.DETAIL, 0.25))))
    runs = []          # open blocks: [xa, xb, rect]
    done = []
    for k in range(n):
        a, b = x0 + (x1 - x0) * k / n, x0 + (x1 - x0) * (k + 1) / n
        s = dl.s_of_x(0.5 * (a + b))
        lane = interior.clamp_rect(s, rect, interior.MARGIN)
        free = rect_minus(lane, holes_in(cache, a, b))
        nxt = []
        for r in free:
            if r[1] - r[0] < MIN or r[3] - r[2] < MIN:
                continue
            m = next((o for o in runs if all(abs(o[2][i] - r[i]) < 0.001 for i in range(4))), None)
            if m:
                m[1] = b
                m[2] = tuple(max(m[2][i], r[i]) if i % 2 == 0 else min(m[2][i], r[i]) for i in range(4))
                nxt.append(m)
                runs.remove(m)
            else:
                nxt.append([a, b, r])
        done += runs
        runs = nxt
    done += runs
    count = 0
    for xa, xb, r in done:
        xa, xb = xa + SEP, xb - SEP
        r = (r[0] + SEP, r[1] - SEP, r[2] + SEP, r[3] - SEP)
        if fits(xa, xb, r):
            pb.box((xb - xa, r[1] - r[0], r[3] - r[2]), loc=(0.5 * (xa + xb), 0.5 * (r[0] + r[1]), 0.5 * (r[2] + r[3])),
                   mat=mat)
        else:
            interior.conform(pb, xa, xb, r, mat, bus=False, n=16, step=0.03)
        count += 1
    return count


def build(root):
    pb = PB("Foam_Infill")
    total = 0
    for x0, x1, rect, names, step in LANES:
        total += fill_lane(pb, x0, x1, rect, names, step)
    print("infill blocks", total)
    return pb.finish(parent=root, smooth_angle=40,
                     props=part_props("Foam Infill", "Airframe", (0.0, 0, -0.12),
                                      "Cut-to-fit EPP foam blocks packed around cable runs, pushrods and the "
                                      "small units, so nothing can shift in flight.", 9))
