#!/usr/bin/env python3
import argparse, csv, time, pathlib
import pybullet as p
import pybullet_data

def ensure_dir(path: pathlib.Path):
    path.parent.mkdir(parents=True, exist_ok=True)

def run(args):
    gui = p.GUI if not args.headless else p.DIRECT
    p.connect(gui)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -args.g)
    p.loadURDF("plane.urdf")

    # Body: light sphere (point-mass-ish)
    sphere = p.loadURDF("sphere2.urdf", [0, 0, args.h0], useFixedBase=False, globalScaling=0.3)

    p.setTimeStep(args.dt)
    p.setRealTimeSimulation(0)

    # Params & state
    m, g, k, b, l0 = args.m, args.g, args.k, args.b, args.l0
    h_target = args.h_ref
    flight = True              # start in flight
    prev_zdot = 0.0
    apex_pred = args.h0

    # Logging
    out = pathlib.Path("data/logs/hopper_1d_log.csv")
    ensure_dir(out)
    f = out.open("w", newline="")
    wr = csv.writer(f)
    wr.writerow(["t","z","zdot","state","u_h","apex_pred","apex_event"])

    t = 0.0
    for step in range(args.steps):
        pos, _ = p.getBasePositionAndOrientation(sphere)
        lin, _ = p.getBaseVelocity(sphere)
        z, zdot = pos[2], lin[2]

        # Events
        if flight and (z <= l0 and zdot < 0):  # touchdown
            flight = False
        if not flight and (z >= l0 and zdot > 0):  # liftoff
            flight = True

        # Apex detection: + to − crossing in flight
        apex_event = 0
        if flight and step > 0 and prev_zdot > 0 and zdot <= 0:
            apex_event = 1

        # Height control (stance only)
        u_h = 0.0
        if not flight:
            apex_pred = z + (zdot*zdot)/(2*g)   # ballistic prediction
            u_h = args.k_h * (h_target - apex_pred)
            comp = max(0.0, l0 - z)
            F_extra = args.alpha * u_h * (comp / max(l0, 1e-6))
        else:
            apex_pred = z
            F_extra = 0.0

        # Vertical forces
        F_spring = k*(l0 - z) if not flight else 0.0
        F_damp   = -b*zdot     if not flight else 0.0
        Fg       = -m*g
        Fz       = F_spring + F_damp + Fg + F_extra

        p.applyExternalForce(sphere, -1, [0,0,Fz], [0,0,0], p.WORLD_FRAME)

        p.stepSimulation()
        wr.writerow([t, z, zdot, ("FLIGHT" if flight else "STANCE"), u_h, apex_pred, apex_event])

        if args.sleep > 0 and not args.headless:
            time.sleep(args.sleep)
        prev_zdot = zdot
        t += args.dt

    f.close()
    p.disconnect()
    print(f"Saved {out.resolve()}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Minimal 1-D hopper (Raibert-style height loop)")
    ap.add_argument("--dt", type=float, default=1/240)
    ap.add_argument("--steps", type=int, default=4000)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--sleep", type=float, default=0.0)

    # physical params
    ap.add_argument("--m", type=float, default=1.0)
    ap.add_argument("--g", type=float, default=9.81)
    ap.add_argument("--k", type=float, default=1200.0)
    ap.add_argument("--b", type=float, default=15.0)
    ap.add_argument("--l0", type=float, default=0.45)
    ap.add_argument("--h0", type=float, default=0.6)

    # height control
    ap.add_argument("--h-ref", dest="h_ref", type=float, default=0.65)
    ap.add_argument("--k-h", dest="k_h", type=float, default=0.25)
    ap.add_argument("--alpha", type=float, default=50.0)

    run(ap.parse_args())
