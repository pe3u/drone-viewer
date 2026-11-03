"""Propulsion: firewall, brushless outrunner motor, ESC, spinner and pusher propeller."""
import math
from mathutils import Vector, Matrix
import dl
from dl import PB, part_props

AX_Z = dl.zc(1.0)
X_FW = dl.sx(1.0)          # firewall / end of tail cone
R_MOTOR = 0.033


def blade(pb, angle_deg, radius=0.21, r0=0.028, pitch=0.16, xh=0.0, zc=0.0):
    rot = Matrix.Rotation(math.radians(angle_deg), 4, "X")
    secs = []
    nsec = dl.segs(14, 8)
    for k in range(nsec + 1):
        u = k / nsec
        r = r0 + (radius - r0) * (u ** 0.9)
        c = 0.024 + 0.022 * math.sin(math.pi * min(1, u * 1.15)) * (1 - 0.6 * u) - 0.012 * u ** 3
        c = max(c, 0.012)
        beta = math.atan(pitch / (2 * math.pi * r))
        if u > 0.97:
            c *= 0.75
        cd = Vector((math.sin(beta), math.cos(beta), 0))
        nv = Vector((math.cos(beta), -math.sin(beta), 0))
        loop = dl.airfoil_loop(0.0, 1.0, 9, m=0.04, p=0.4, t=0.10 if u > 0.15 else 0.10 + (0.15 - u) * 1.4)
        centre = Vector((xh, 0, r))
        le = centre - cd * 0.35 * c
        secs.append([rot @ p + Vector((0, 0, zc)) for p in dl.place_section(loop, le, c, 0, nrm=nv, chord_dir=cd)])
    # paint: black blade, yellow tip
    tip_k = int(round(nsec * 0.86))
    pb.loft(secs[:tip_k + 1], "dark", cap0=True)
    pb.loft(secs[tip_k:], "yellow", cap1=True)


def build(root):
    zc = AX_Z
    # ---- firewall + motor mount (stays with the airframe)
    mm = PB("Motor_Mount")
    mm.cyl(0.037, 0.005, loc=(X_FW - 0.002, 0, zc), rot=(0, 90, 0), mat="alu", seg=24)
    for a in range(4):
        ang = math.radians(45 + 90 * a)
        mm.box((0.006, 0.010, 0.030), loc=(X_FW + 0.004, math.cos(ang) * 0.012, zc + math.sin(ang) * 0.012),
               rot=(math.degrees(ang), 0, 0), mat="alu")
    mount = mm.finish(parent=root, smooth_angle=35,
                      props=part_props("Motor Mount", "Propulsion", (0.12, 0, 0),
                                       "Aluminium firewall and X-mount closing the tail cone.", 50))
    # ---- motor
    xm0 = X_FW + 0.008
    lm = 0.042
    mo = PB("Electric_Motor")
    mo.cyl(0.020, 0.006, loc=(xm0 + 0.003, 0, zc), rot=(0, 90, 0), mat="dark", seg=20)   # stator base
    # stator teeth with copper windings
    nt = 12
    for i in range(nt):
        a = 2 * math.pi * i / nt
        mo.box((lm - 0.010, 0.007, 0.010), loc=(xm0 + 0.006 + (lm - 0.010) / 2, math.cos(a) * 0.022,
                                              zc + math.sin(a) * 0.022), rot=(math.degrees(a) + 90, 0, 0),
               mat="copper", bevel=0.0015)
    mo.cyl(0.012, lm - 0.008, loc=(xm0 + 0.006 + (lm - 0.008) / 2, 0, zc), rot=(0, 90, 0), mat="grey", seg=16)
    # rotor bell: rear ring, front cap, bars between (cooling windows)
    xb0 = xm0 + 0.008
    mo.lathe([(xb0, R_MOTOR), (xb0 + 0.007, R_MOTOR), (xb0 + 0.007, R_MOTOR - 0.003), (xb0, R_MOTOR - 0.003)],
             "alu", n=32, axis="X", center=(0, 0, zc), wrap=True)
    bar_n = 8
    for i in range(bar_n):
        a = 2 * math.pi * (i + 0.5) / bar_n
        mo.box((lm - 0.014, 0.010, 0.003), loc=(xb0 + 0.007 + (lm - 0.014) / 2, math.cos(a) * (R_MOTOR - 0.0015),
                                              zc + math.sin(a) * (R_MOTOR - 0.0015)),
               rot=(math.degrees(a) + 90, 0, 0), mat="alu")
    xc = xb0 + lm - 0.007
    mo.cyl(R_MOTOR, 0.006, loc=(xc + 0.003, 0, zc), rot=(0, 90, 0), r2=R_MOTOR - 0.004, mat="alu", seg=32)
    mo.cyl(0.010, 0.006, loc=(xc + 0.009, 0, zc), rot=(0, 90, 0), mat="dark", seg=16)
    mo.cyl(0.0035, 0.030, loc=(xc + 0.015, 0, zc), rot=(0, 90, 0), mat="alu", seg=10)  # shaft
    # rotor magnet ring hint (dark inner band)
    mo.lathe([(xb0 + 0.0072, R_MOTOR - 0.0032), (xc, R_MOTOR - 0.0032), (xc, R_MOTOR - 0.006),
              (xb0 + 0.0072, R_MOTOR - 0.006)], "dark", n=32, axis="X", center=(0, 0, zc), wrap=True)
    motor = mo.finish(parent=root, smooth_angle=35,
                      props=part_props("Electric Motor", "Propulsion", (0.28, 0, 0),
                                       "Brushless outrunner, 12-slot stator, ~800 W, driving a pusher propeller.", 51))
    # ---- spinner + propeller (one rotating part)
    xh = xc + 0.022
    pr = PB("Propeller")
    pr.cyl(0.014, 0.014, loc=(xh, 0, zc), rot=(0, 90, 0), mat="alu", seg=20)  # hub
    # spinner (rotational loft)
    prof = [(xh - 0.012, 0.030), (xh + 0.005, 0.030), (xh + 0.025, 0.026), (xh + 0.045, 0.016), (xh + 0.058, 0.004)]
    secs = [[(x, math.cos(2 * math.pi * k / 24) * r, zc + math.sin(2 * math.pi * k / 24) * r) for k in range(24)]
            for x, r in prof]
    pr.loft(secs, "alu", cap0=True, cap1=True)
    for b in (0, 180):
        blade(pr, b, xh=xh, zc=zc)
    prop = pr.finish(parent=root, smooth_angle=35, location=(xh, 0, zc),
                     props=part_props("Propeller", "Propulsion", (0.50, 0, 0),
                                      "Two-blade 17 x 6.5 in folding-style carbon propeller with spinner. Yellow tips for visibility.",
                                      52))
    return [mount, motor, prop]
