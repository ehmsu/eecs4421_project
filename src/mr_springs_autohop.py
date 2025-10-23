#!/usr/bin/env python3
import time, pathlib, csv, argparse
import pybullet as p, pybullet_data

def key_pressed(ch):
    ev = p.getKeyboardEvents(); code = ord(ch)
    return ev.get(code) in (p.KEY_IS_DOWN, p.KEY_WAS_TRIGGERED)

def clamp(x, lo, hi): return max(lo, min(hi, x))

def draw_world_axes(length=1.0, zoff=0.001):
    o = [0,0,zoff]
    p.addUserDebugLine(o, [length,0,zoff], [1,0,0], 4, 0)  # X red
    p.addUserDebugLine(o, [0,length,zoff], [0,1,0], 4, 0)  # Y green
    p.addUserDebugLine(o, [0,0,length],  [0,0,1], 4, 0)    # Z blue

def triad(origin, R=0.25, life=0.1):
    o = origin
    lid = []
    lid += [p.addUserDebugLine(o, [o[0]+R,o[1],o[2]], [1,0,0], 2, life)]
    lid += [p.addUserDebugLine(o, [o[0],o[1]+R,o[2]], [0,1,0], 2, life)]
    lid += [p.addUserDebugLine(o, [o[0],o[1],o[2]+R], [0,0,1], 2, life)]
    return lid

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true", help="Use GUI instead of DIRECT")
    ap.add_argument("--dt", type=float, default=1/240.0)
    ap.add_argument("--steps", type=int, default=240*30, help="Total sim steps")

    ap.add_argument("--g", type=float, default=9.81)
    ap.add_argument("--friction", type=float, default=1.5)

    # energy knobs (slightly higher defaults)
    ap.add_argument("--l0", type=float, default=0.05)
    ap.add_argument("--base-release", type=float, default=0.16, dest="base_release")
    ap.add_argument("--release-min", type=float, default=0.06)
    ap.add_argument("--release-max", type=float, default=0.24)

    ap.add_argument("--Fmax", type=float, default=3200.0)
    ap.add_argument("--pos-gain", type=float, default=2.0e-3)
    ap.add_argument("--vel-gain", type=float, default=4.0e-1)

    # apex loop
    ap.add_argument("--h-ref", type=float, default=0.65, dest="h_ref")
    ap.add_argument("--k-h", type=float, default=1.0, dest="k_h")
    ap.add_argument("--alpha", type=float, default=0.08)

    # timings
    ap.add_argument("--t-compress", type=float, default=0.35)
    ap.add_argument("--t-release", type=float, default=0.14)
    ap.add_argument("--t-stance", type=float, default=0.22)

    ap.add_argument("--urdf", type=str, default="urdf/mr_springs.urdf")
    ap.add_argument("--log", type=str, default="data/logs/mr_springs_autohop.csv")
    args = ap.parse_args()

    LOG = pathlib.Path(args.log); LOG.parent.mkdir(parents=True, exist_ok=True)

    cid = p.connect(p.GUI if args.gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setTimeStep(args.dt)
    p.setGravity(0,0,-args.g)

    plane = p.loadURDF("plane.urdf")
    robot = p.loadURDF(args.urdf, [0,0,0.02])
    jid = 0

    p.changeDynamics(plane, -1, lateralFriction=args.friction, restitution=0.0)
    for link in (-1,0,1,2):
        try: p.changeDynamics(robot, link, lateralFriction=args.friction, restitution=0.0)
        except: pass

    if args.gui:
        # Camera & world axes
        p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=60, cameraPitch=-35, cameraTargetPosition=[0,0,0.3])
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        draw_world_axes(1.0)
        p.addUserDebugText("Press 'q' to quit", [0,0,1.2], [1,1,1], 1.4, 0)

    # FSM
    state, timer = "compress", 0.0
    prev_zdot = 0.0
    last_apex_z = None

    l0 = args.l0
    release = l0 + args.base_release
    rmin, rmax = args.release_min, args.release_max

    # visuals: target apex dot + vertical guide
    apex_dot = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1,0,0,1])
    apex_id  = p.createMultiBody(baseMass=0, baseVisualShapeIndex=apex_dot, basePosition=[0,0,args.h_ref])
    apex_line = p.addUserDebugLine([0,0,0],[0,0,args.h_ref],[1,0,0],1,0)

    # COM breadcrumb (short trail)
    trail = []; max_trail = 120

    # initial motor command
    p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL,
        targetPosition=0.0, positionGain=args.pos_gain, velocityGain=args.vel_gain, force=args.Fmax)

    t = 0.0
    with LOG.open("w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t","state","z","zdot","release_cmd","apex_event","last_apex","h_ref"])
        for i in range(args.steps):
            # --- FSM ---
            if state == "compress":
                if timer == 0.0:
                    p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
                        targetPosition=0.0, positionGain=args.pos_gain, velocityGain=args.vel_gain, force=args.Fmax)
                if timer > args.t_compress:
                    p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
                        targetPosition=release, positionGain=args.pos_gain, velocityGain=args.vel_gain, force=args.Fmax)
                    state, timer = "release", 0.0

            elif state == "release":
                if timer > args.t_release:
                    p.setJointMotorControl2(robot,jid,p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)
                    state, timer = "flight", 0.0

            elif state == "flight":
                zdot = p.getBaseVelocity(robot)[0][2]
                apex_event = 0
                if prev_zdot > 0 and zdot <= 0:
                    last_apex_z = p.getBasePositionAndOrientation(robot)[0][2]
                    apex_event = 1
                    # retarget release for the NEXT hop
                    err = args.h_ref - last_apex_z
                    release = clamp(l0 + args.base_release + args.alpha*args.k_h*err, rmin, rmax)
                if p.getContactPoints(bodyA=robot):
                    p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
                        targetPosition=l0, positionGain=args.pos_gain, velocityGain=args.vel_gain, force=args.Fmax)
                    state, timer = "stance", 0.0
                prev_zdot = zdot

            elif state == "stance":
                if timer > args.t_stance:
                    p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
                        targetPosition=0.0, positionGain=args.pos_gain, velocityGain=args.vel_gain, force=args.Fmax)
                    state, timer = "compress", 0.0

            # --- visuals (COM triad, trail, live apex guide) ---
            pos = p.getBasePositionAndOrientation(robot)[0]
            if args.gui:
                triad(pos, 0.15, life=args.dt*3)
                trail.append(p.addUserDebugLine(pos, [pos[0],pos[1],pos[2]+0.001],[0.2,0.2,1], 1, args.dt*max(3,int(1/args.dt))))
                if len(trail) > max_trail:
                    trail.pop(0)
                # update apex target if h_ref changed (CLI keeps it fixed; line is static)

            # --- log ---
            z  = pos[2]
            zd = p.getBaseVelocity(robot)[0][2]
            wr.writerow([t, state, z, zd, release,
                         1 if (state=="flight" and prev_zdot>0 and zd<=0) else 0,
                         last_apex_z if last_apex_z is not None else "", args.h_ref])

            p.stepSimulation()
            if args.gui: time.sleep(args.dt)
            t += args.dt; timer += args.dt
            if args.gui and key_pressed('q'): break

    p.disconnect()
    print(f"Saved {LOG.resolve()}")

if __name__ == "__main__":
    main()
