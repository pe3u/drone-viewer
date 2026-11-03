"""Wiring harness: power, signal, data and RF cables routed along the floor buses."""
import math
from mathutils import Vector
import dl
from dl import PB, part_props, sx
import avionics
import payload

ZB = avionics.ZF + 0.007        # floor bus height
YB = 0.074                      # bus offset from centreline


def V(*a):
    return Vector(a)


def bundle(pb, pts, colours, r, gap=None, mat_conn=None):
    """Several parallel wires following the same path. Ends stay together."""
    gap = gap or r * 2.1
    n = len(colours)
    P = [Vector(p) for p in pts]
    for i, col in enumerate(colours):
        off = (i - (n - 1) / 2) * gap
        path = []
        for k, p in enumerate(P):
            a = P[max(k - 1, 0)]
            b = P[min(k + 1, len(P) - 1)]
            t = (b - a).normalized()
            side = t.cross(Vector((0, 0, 1)))
            if side.length < 0.2:
                side = t.cross(Vector((0, 1, 0)))
            side.normalize()
            path.append(p + side * off)
        pb.tube(path, r, mat=col, seg=5, steps=4)
    if mat_conn:
        for end in (P[0], P[-1]):
            pb.box((0.007, 0.004 + n * gap, 0.005), loc=end, mat=mat_conn)


def tie(pb, p, r=0.006):
    pb.cyl(r, 0.003, loc=p, rot=(0, 90, 0), mat="dark", seg=8)


def build(root):
    P = avionics.PORTS
    pb = PB("Wiring_Harness")
    G1 = -0.010                 # cable layer between the lower and mid decks
    G2 = 0.0525                 # cable layer under the bridge units
    RA, RB, RC = -0.3375, -0.162, -0.062  # vertical riser gaps
    XR = -0.470                 # riser between the camera and flight-controller cradles
    # --- main power
    for sy, col in ((1, "red"), (-1, "dark")):
        k = "bat_r" if sy > 0 else "bat_l"
        bundle(pb, [P[k], V(0.2338, sy * 0.016, -0.034), P["pdb_in"]], [col], 0.0032)
    bundle(pb, [P["pdb_out"], V(0.305, 0, -0.030), P["esc_in"]], ["red", "dark"], 0.0026)
    # --- motor phases through the tail boom
    zax = dl.zc(1.0)
    bundle(pb, [P["esc_out"], V(0.46, 0, -0.030), V(0.62, 0, 0.000), V(0.78, 0, 0.022), V(0.88, 0, zax),
                V(0.905, 0, zax)], ["red", "dark", "wblue"], 0.0022)
    # --- power feeders forward along the right floor channel
    fwd = [P["pdb_fwd"], V(0.243, 0.024, -0.047), V(0.239, 0.046, -0.056), V(0.228, 0.066, -0.063), V(0.18, YB, ZB), V(-0.05, YB, ZB),
           V(-0.20, YB, ZB)]
    bundle(pb, fwd + [V(-0.30, YB, ZB), V(-0.335, 0.050, -0.062), V(-0.340, 0.030, -0.052), P["fc_r"]],
           ["red", "dark"], 0.0014)
    bundle(pb, [p + V(0, 0, 0.005) for p in fwd[:-1]] + [V(-0.245, 0.056, -0.060), V(-0.252, 0.036, -0.055),
                                                       P["dpu_pwr"]], ["red", "dark"], 0.0018)
    lid = payload.LIDAR
    lid_top = V(lid.x + 0.024, 0.0, lid.z + 0.072)

    def to_lidar(sy):
        return [V(-0.40, sy * YB, ZB), V(-0.462, sy * 0.069, -0.063), V(XR, sy * 0.068, -0.058),
                V(XR, sy * 0.067, -0.045), V(XR, sy * 0.066, -0.030), V(XR, sy * 0.062, -0.016), V(-0.478, sy * 0.058, G1),
                V(-0.60, sy * 0.048, G1), V(-0.648, sy * 0.036, G1), V(-0.675, sy * 0.020, -0.022),
                lid_top + V(0, sy * 0.01, 0)]
    bundle(pb, [p + V(0, 0, -0.004) for p in fwd] + [V(-0.30, YB, ZB - 0.004)] + to_lidar(1)[:1] +
           [p + V(0, -0.004, 0) for p in to_lidar(1)[1:-1]] + to_lidar(1)[-1:], ["red", "dark"], 0.0016)
    # --- data: DPU -> LiDAR (Ethernet) along the left floor channel
    bundle(pb, [P["dpu_f"], V(-0.252, -0.040, -0.060), V(-0.275, -0.060, -0.064), V(-0.30, -YB, ZB)] + to_lidar(-1),
           ["wblue"], 0.0024)
    # --- data: DPU -> camera (USB) under the cradles, into the slot between IMU and FC
    cam_top = V(payload.CAM_X + 0.03, 0.0, -0.075)
    bundle(pb, [P["dpu_f2"], V(-0.262, -0.022, -0.062), V(-0.32, -0.050, -0.064), V(-0.42, -0.050, -0.064),
                V(-0.462, -0.040, -0.064), V(-0.472, -0.018, -0.068), cam_top], ["wblue", "red"], 0.0018)
    # --- flight controller signal looms
    bundle(pb, [P["fc_f"], V(-0.465, 0, -0.040), V(-0.485, 0, -0.034), P["imu"]], ["dark", "red", "yellow"], 0.0011)
    bundle(pb, [P["fc_l"], V(-0.335, -0.030, -0.055), V(-0.29, -0.035, -0.060), V(-0.26, -0.034, -0.056), P["dpu_f"]
                + V(0, 0.006, 0.006)], ["dark", "yellow"], 0.0011)
    up = [P["fc_top"], V(-0.352, 0, -0.014), V(-0.343, 0, -0.009), V(RA, 0, -0.002), V(RA, 0, 0.012), V(RA, 0, 0.030)]
    bundle(pb, up + [V(RA + 0.002, 0, 0.056), P["gnss_io"]], ["dark", "red", "yellow", "wblue"], 0.0010)
    bundle(pb, [p + V(0, -0.010, 0) for p in up] + [V(RA + 0.004, -0.012, G2), V(-0.26, -0.012, G2),
                                                    V(-0.212, -0.008, G2 + 0.002), P["radio_io"]],
           ["dark", "red", "yellow"], 0.0010)
    # wing servo / pitot leads: floor channels aft, up the riser in front of the centre bulkhead
    for sy in (1, -1):
        bundle(pb, [P["fc_r"] if sy > 0 else P["fc_l"], V(-0.335, sy * 0.050, -0.062), V(-0.30, sy * (YB - 0.006), ZB),
                    V(-0.09, sy * (YB - 0.006), ZB), V(RC, sy * 0.064, -0.040), V(RC, sy * 0.056, 0.020),
                    V(-0.050, sy * 0.044, 0.046), V(-0.035, sy * 0.032, 0.052)],
               ["dark", "red", "yellow"], 0.0011)
    # tail servo loom: left floor channel, then over the aft tray under the Remote ID module
    bundle(pb, [P["fc_l"], V(-0.335, -0.056, -0.064), V(-0.30, -YB, ZB + 0.004), V(0.20, -YB, ZB + 0.004),
                V(0.236, -0.066, -0.056), V(0.27, -0.050, -0.0455), V(0.40, -0.026, -0.0455), P["servo_t"]],
           ["dark", "red", "yellow"], 0.0011)
    # --- RF coax (above the bridge)
    comm_base = V(payload.COMM.x, 0.0, payload.top_z(dl.s_of_x(payload.COMM.x)) - 0.006)
    gnss_base = V(payload.GNSS.x, 0.0, payload.top_z(dl.s_of_x(payload.GNSS.x)) - 0.006)
    bundle(pb, [P["radio_rf"], V(-0.282, 0.016, 0.074), V(-0.280, 0.0175, 0.090), V(-0.268, 0.016, 0.104),
                V(-0.252, 0.008, 0.112), comm_base], ["dark"], 0.0018)
    bundle(pb, [P["gnss_rf"], V(-0.3835, 0.012, 0.074), V(-0.381, 0.012, 0.090), V(-0.372, 0.008, 0.106),
                gnss_base], ["dark"], 0.0018)
    # --- nose shelf: power, SSD data, airspeed tubes (G1 layer, through the forward bulkhead)
    nose = [V(-0.47, 0.0, -0.030), V(-0.48, 0.0, G1), V(-0.60, 0.0, G1), V(-0.648, 0.0, G1 + 0.001)]
    bundle(pb, [P["fc_f"] + V(0, 0.01, 0), V(-0.465, 0.010, -0.044)] + [p + V(0, 0.012, 0) for p in nose] +
           [V(-0.668, 0.012, -0.004), P["shelf_pwr"]], ["red", "dark"], 0.0013)
    bundle(pb, [P["dpu_f"] + V(0, -0.006, 0.004), V(-0.27, -0.052, -0.062), V(-0.30, -YB + 0.006, ZB + 0.004),
                V(-0.40, -YB + 0.006, ZB + 0.004)] + [p + V(0, 0.008, 0) for p in to_lidar(-1)[1:8]] +
           [V(-0.630, -0.034, G1 + 0.001), V(-0.648, -0.030, G1 + 0.002), V(-0.672, -0.030, -0.004), P["shelf_ssd"]], ["wblue"], 0.0018)
    bundle(pb, [P["shelf_as"], V(-0.664, 0.040, -0.004), V(-0.640, 0.066, G1), V(-0.36, 0.066, G1),
                V(-0.09, 0.066, G1), V(RC, 0.030, 0.000), V(RC, -0.030, 0.020), V(-0.050, -0.050, 0.050),
                V(-0.04, -0.050, 0.055)], ["white", "white"], 0.0016)
    # --- RC receiver and safety switch: aft over the bridge, down the riser, forward in G2, down to the FC
    bundle(pb, [P["rc"], V(-0.205, 0.006, 0.090), V(RB - 0.004, 0.012, 0.080), V(RB, 0.016, G2 + 0.002),
                V(-0.25, 0.018, G2), V(RA + 0.004, 0.016, G2 - 0.002), V(RA, 0.012, 0.020), V(-0.350, 0.012, -0.012),
                P["fc_top"] + V(0, 0.012, 0)], ["dark", "red", "yellow"], 0.0009)
    bundle(pb, [P["safety"], V(RB, -0.012, 0.085), V(RB, -0.022, G2 + 0.002), V(-0.25, -0.024, G2),
                V(RA + 0.004, -0.022, G2 - 0.002), V(RA, -0.020, 0.020), V(-0.350, -0.018, -0.012),
                P["fc_top"] + V(0, -0.020, 0)], ["dark", "red"], 0.0010)
    # --- tail loom: up riser A, aft in G2, then along the left longeron to the fin anti-collision light
    tail = [P["fc_top"] + V(0, 0.020, 0), V(-0.352, 0.022, -0.012), V(RA, 0.024, 0.020), V(RA + 0.004, 0.0, G2),
            V(-0.296, -0.017, G2), V(-0.2885, -0.024, 0.0545), V(-0.2885, -0.046, 0.0615)]
    for s in (0.34, 0.37, 0.42, 0.50, 0.60, 0.70, 0.80, 0.84):
        tail.append(dl.fus_point(s, math.radians(140), inset=dl.SHELL_T + 0.012))
    tail += [V(0.625, -0.012, 0.078), V(0.66, -0.004, 0.110), V(0.70, 0.0, dl.zc(0.86) + 0.12)]
    bundle(pb, tail, ["dark", "red", "yellow"], 0.0010)
    # --- cable ties along the buses
    for x in (-0.42, -0.30, -0.18, -0.06, 0.06, 0.18):
        tie(pb, V(x, YB, ZB), 0.0062)
        tie(pb, V(x, -YB, ZB), 0.0055)
    # spiral sleeve on the motor phases
    for k in range(8):
        x = 0.47 + k * 0.05
        t = (x - 0.46) / (0.88 - 0.46)
        z = dl.lerp(-0.030, zax, dl.smoothstep(t))
        tie(pb, V(x, 0, z), 0.0058)
    return pb.finish(parent=root, smooth_angle=50,
                     props=part_props("Wiring Harness", "Power", (0, 0, 0),
                                      "Silicone power leads (red/black), motor phases, Ethernet/USB data lines (blue), "
                                      "servo/sensor looms and RF coax, secured with cable ties.", 49))
