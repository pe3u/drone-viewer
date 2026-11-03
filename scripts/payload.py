"""Survey payload: LiDAR, gimballed mapping camera, GNSS antenna, datalink antenna."""
import math
import bpy
from mathutils import Vector, Matrix
import dl
from dl import PB, part_props, sx, zc, wh

LIDAR = Vector((sx(0.10), 0.0, -0.105))      # flange position
CAM_X = sx(0.21)
GNSS = Vector((sx(0.30), 0.0, 0.0))
COMM = Vector((sx(0.365), 0.0, 0.0))


def top_z(s):
    return zc(s) + wh(s)[1]


def ring(x_or_z, r, n, axis="Z", c=(0, 0, 0)):
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        if axis == "Z":
            out.append((c[0] + math.cos(a) * r, c[1] + math.sin(a) * r, x_or_z))
        else:
            out.append((x_or_z, c[1] + math.cos(a) * r, c[2] + math.sin(a) * r))
    return out


def lathe(pb, prof, mat, n=32, axis="Z", c=(0, 0, 0), cap0=True, cap1=True):
    pb.loft([ring(h, r, n, axis, c) for h, r in prof], mat, cap0=cap0, cap1=cap1)


def build_lidar(root):
    pb = PB("LiDAR_Sensor")
    x, y = LIDAR.x, LIDAR.y
    z0 = LIDAR.z
    C = (x, y, 0)
    # one closed profile, top to bottom: cooling fins (inside the hull), flange, scan window, bottom cap
    prof = [(0.050, 0.0), (0.050, 0.030), (0.048, 0.044)]
    for k in range(5):
        zz = 0.042 - k * 0.008
        prof += [(zz, 0.044), (zz, 0.048), (zz - 0.004, 0.048), (zz - 0.004, 0.044)]
    prof += [(0.003, 0.044), (0.003, 0.056), (-0.003, 0.056), (-0.003, 0.049), (-0.012, 0.049), (-0.014, 0.047),
             (-0.030, 0.047), (-0.038, 0.047), (-0.052, 0.047), (-0.052, 0.049), (-0.060, 0.049), (-0.064, 0.044),
             (-0.066, 0.020), (-0.066, 0.0)]
    prof = [(z0 + h, r) for h, r in prof]

    def band(h0, r0, h1, r1):
        m = 0.5 * (h0 + h1) - z0
        if r0 == r1 == 0.047 and -0.052 <= m <= -0.014:
            return "lens" if -0.038 < m < -0.030 else "glass"
        return "alu"
    pb.lathe(prof, band, n=40, center=C)
    for k in range(6):
        a = 2 * math.pi * k / 6
        pb.cyl(0.0035, 0.004, loc=(x + math.cos(a) * 0.050, y + math.sin(a) * 0.050, z0 + 0.005), mat="dark", seg=6)
    # connector on top
    pb.cyl(0.008, 0.012, loc=(x + 0.02, y, z0 + 0.056), mat="dark", seg=12)
    sc = Matrix.Translation((x, y, z0)) @ Matrix.Scale(1.2, 4) @ Matrix.Translation((-x, -y, -z0))
    for v in pb.bm.verts:
        v.co = sc @ v.co
    o = pb.finish(parent=root, smooth_angle=35,
                  props=part_props("LiDAR Sensor", "Payload", (-0.10, 0, -0.32),
                                   "360 deg multi-beam laser scanner, ~240 k pts/s, 100 m range. Chin-mounted, rubber isolated.",
                                   60))
    return o


def build_camera(root):
    x = CAM_X
    zb = zc(0.21) - wh(0.21)[1]          # belly
    pb = PB("Mapping_Camera")
    # damper plate inside hull + 4 rubber dampers + upper plate
    pb.box((0.10, 0.075, 0.003), loc=(x, 0, zb + 0.045), mat="carbon")
    pb.box((0.10, 0.075, 0.003), loc=(x, 0, zb + 0.020), mat="carbon")
    for sx_ in (-1, 1):
        for sy_ in (-1, 1):
            pb.sphere(0.008, loc=(x + sx_ * 0.040, sy_ * 0.028, zb + 0.0325), mat="orange", u=10, v=6)
    # yaw motor through the belly
    pb.cyl(0.022, 0.020, loc=(x, 0, zb + 0.008), mat="dark", seg=24)
    pb.cyl(0.024, 0.006, loc=(x, 0, zb - 0.004), mat="alu", seg=24)
    # yoke: horizontal arm then vertical arm on +Y side
    yo = 0.050
    pb.box((0.022, yo + 0.010, 0.006), loc=(x, yo / 2, zb - 0.010), mat="dark", bevel=0.002)
    pb.box((0.022, 0.006, 0.070), loc=(x, yo + 0.002, zb - 0.045), mat="dark", bevel=0.002)
    # roll/pitch motor
    zcam = zb - 0.075
    pb.cyl(0.018, 0.014, loc=(x, yo - 0.006, zcam), rot=(90, 0, 0), mat="dark", seg=20)
    pb.cyl(0.019, 0.004, loc=(x, yo - 0.014, zcam), rot=(90, 0, 0), mat="alu", seg=20)
    # camera body pitched down-forward
    tilt = -35.0
    R = Matrix.Rotation(math.radians(tilt), 4, "Y")
    cam = PB("tmp")
    cam.box((0.062, 0.070, 0.054), loc=(0, 0, 0), mat="dark", bevel=0.006, seg=2)
    cam.box((0.020, 0.030, 0.010), loc=(0.010, 0, 0.031), mat="dark", bevel=0.002)   # hot-shoe hump
    cam.cyl(0.003, 0.004, loc=(0.018, 0.022, 0.029), mat="red", seg=8)               # status LED
    # lens barrel forward (-X)
    lens = [(-0.028, 0.0), (-0.028, 0.026), (-0.040, 0.026), (-0.040, 0.024), (-0.044, 0.024), (-0.044, 0.0255),
            (-0.048, 0.0255), (-0.048, 0.024), (-0.052, 0.024), (-0.052, 0.027), (-0.062, 0.027), (-0.064, 0.024),
            (-0.060, 0.019), (-0.058, 0.0)]
    lm = ["dark"] * (len(lens) - 1)
    lm[5] = "alu"
    lm[11] = lm[12] = "lens"
    cam.lathe(lens, lm, n=28, axis="X")
    for v in cam.bm.verts:
        v.co = (Matrix.Translation((x, 0.0, zcam)) @ R) @ v.co
    # transfer temp geometry into the part
    tmp = bpy.data.meshes.new("tmp")
    for f in cam.bm.faces:
        f.material_index = pb.mi(cam.mats[f.material_index])
    cam.bm.to_mesh(tmp)
    cam.bm.free()
    pb.bm.from_mesh(tmp)
    bpy.data.meshes.remove(tmp)
    # opposite side bearing arm
    pb.box((0.022, 0.006, 0.050), loc=(x, -yo + 0.002, zb - 0.055), mat="dark", bevel=0.002)
    pb.box((0.022, yo, 0.006), loc=(x, -yo / 2 + 0.002, zb - 0.030), mat="dark", bevel=0.002)
    pb.cyl(0.010, 0.012, loc=(x, -yo + 0.010, zcam), rot=(90, 0, 0), mat="alu", seg=16)
    # enlarge the external gimbal/camera (everything below the belly)
    sc = Matrix.Translation((x, 0, zb)) @ Matrix.Scale(1.22, 4) @ Matrix.Translation((-x, 0, -zb))
    for v in pb.bm.verts:
        if v.co.z < zb + 0.001:
            v.co = sc @ v.co
    o = pb.finish(parent=root, smooth_angle=35,
                  props=part_props("Mapping Camera", "Payload", (0.0, 0, -0.42),
                                   "42 MP full-frame mapping camera with 35 mm lens on a 2-axis stabilised gimbal, "
                                   "isolated by rubber dampers.", 61))
    return o


def build_gnss(parent):
    s = dl.s_of_x(GNSS.x)
    zt = top_z(s)
    x = GNSS.x
    pb = PB("GNSS_Antenna")
    zd = zt + 0.032
    # mount + stem
    pb.lathe([(zt - 0.004, 0.0), (zt - 0.004, 0.014), (zt + 0.004, 0.014), (zt + 0.004, 0.006), (zd + 0.001, 0.006),
              (zd + 0.001, 0.0)], ["alu", "alu", "alu", "dark", "dark"], n=20, center=(x, 0, 0))
    # puck: dark base, blue band, white radome
    prof = [(zd, 0.0), (zd, 0.030), (zd + 0.004, 0.040), (zd + 0.008, 0.042), (zd + 0.016, 0.042)]
    for k in range(1, 9):
        a = k / 8 * math.pi / 2
        prof.append((zd + 0.016 + math.sin(a) * 0.022, 0.042 * math.cos(a) if k < 8 else 0.0))
    pm = ["dark", "dark", "dark", "blue"] + ["white"] * 8
    pb.lathe(prof, pm, n=36, center=(x, 0, 0))
    return pb.finish(parent=parent, smooth_angle=35,
                     props=part_props("GNSS Antenna", "Avionics", (0, 0, 0.16),
                                      "Multi-band survey-grade GNSS antenna (L1/L2/L5) for RTK/PPK positioning.", 70))


def build_comm(parent):
    s = dl.s_of_x(COMM.x)
    zt = top_z(s)
    x = COMM.x
    pb = PB("Comm_Antenna")
    prof = [(zt - 0.004, 0.0), (zt - 0.004, 0.013), (zt + 0.006, 0.013), (zt + 0.008, 0.010), (zt + 0.008, 0.008),
            (zt + 0.020, 0.008), (zt + 0.024, 0.0065), (zt + 0.070, 0.0065), (zt + 0.074, 0.0072), (zt + 0.080, 0.0072),
            (zt + 0.084, 0.0062), (zt + 0.12, 0.0060), (zt + 0.16, 0.0050), (zt + 0.163, 0.0)]
    pb.lathe(prof, ["alu"] * 4 + ["dark"] * 9, n=14, center=(x, 0, 0))
    return pb.finish(parent=parent, smooth_angle=35,
                     props=part_props("Communication Antenna", "Avionics", (0, 0, 0.24),
                                      "Dipole whip for the 900 MHz / 2.4 GHz command & telemetry datalink.", 71))


def build(root):
    out = [build_lidar(root), build_camera(root)]
    hatch = bpy.data.objects.get("Top_Hatch")
    out.append(build_gnss(hatch or root))
    out.append(build_comm(hatch or root))
    return out
