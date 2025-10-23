#!/usr/bin/env python3
import time, pathlib, csv
import pybullet as p, pybullet_data

# --- helpers ---
def key_pressed(ch):
    ev = p.getKeyboardEvents(); code = ord(ch)
    return ev.get(code) in (p.KEY_IS_DOWN, p.KEY_WAS_TRIGGERED)

def clamp(x, lo, hi): return max(lo, min(hi, x))

LOG = pathlib.Path("data/logs/mr_springs_apex.csv")
LOG.parent.mkdir(parents=True, exist_ok=True)

# --- sim setup ---
cid = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setTimeStep(1/240.0)
p.setGravity(0,0,-9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("urdf/mr_springs.urdf",[0,0,0.02])
jid = 0  # prismatic joint

# Make ground contact non-bouncy and grippy
p.changeDynamics(plane, -1, lateralFriction=1.2, restitution=0.0)
for link in (-1, 0, 1, 2):  # base and links (foot, body, cap if present)
    try:
        p.changeDynamics(robot, link, lateralFriction=1.0, restitution=0.0)
    except Exception:
        pass

# --- controller params ---
dt = 1/240.0
l0 = 0.05                  # nominal rest extension (m)
base_release = 0.11        # nominal extra extension for a jump (m)
compress = 0.00            # pull-in to pre-load spring
release = l0 + base_release
release_min, release_max = 0.06, 0.20

posGain = 2.0e-3           # motor position control gains (soft)
velGain = 4.0e-1
Fmax    = 2000.0

# --- UI sliders ---
h_ref_slider = p.addUserDebugParameter("h_ref (m)", 0.20, 1.20, 0.55)
kh_slider    = p.addUserDebugParameter("k_h", 0.0, 2.0, 0.8)
alpha_slider = p.addUserDebugParameter("alpha (m/unit)", 0.0, 0.20, 0.06)
text_id = p.addUserDebugText("Press 'q' to quit", [0,0,1.3], [1,1,1], 1.5, 0)

# --- state machine ---
state, timer = "compress", 0.0
prev_zdot = 0.0
last_apex_z = None

# initial motor target
p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
    targetPosition=compress, positionGain=posGain, velocityGain=velGain, force=Fmax)

t = 0.0
with LOG.open("w", newline="") as f:
    wr = csv.writer(f); wr.writerow(["t","state","q","z","h_ref","release_cmd","apex_event","last_apex"])
    while p.isConnected():
        # read UI
        h_ref   = p.readUserDebugParameter(h_ref_slider)
        k_h     = p.readUserDebugParameter(kh_slider)
        alpha   = p.readUserDebugParameter(alpha_slider)

        # states
        if state == "compress" and timer > 0.35:
            # Jump command: extend to current 'release'
            p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
                targetPosition=release, positionGain=posGain, velocityGain=velGain, force=Fmax)
            state, timer = "release", 0.0

        elif state == "release" and timer > 0.10:
            # Let it fly freely
            p.setJointMotorControl2(robot,jid,p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)
            state, timer = "flight", 0.0

        elif state == "flight":
            # detect apex (zdot changes from + to -)
            zdot = p.getBaseVelocity(robot)[0][2]
            apex_event = 0
            if prev_zdot > 0.0 and zdot <= 0.0:
                last_apex_z = p.getBasePositionAndOrientation(robot)[0][2]
                apex_event = 1
                # Raibert-style height correction: adjust next release
                err = (h_ref - last_apex_z)
                delta = alpha * k_h * err
                release = clamp(l0 + base_release + delta, release_min, release_max)
            else:
                apex_event = 0

            # when we touch down again → go to stance
            if p.getContactPoints(bodyA=robot):
                p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
                    targetPosition=l0, positionGain=posGain, velocityGain=velGain, force=Fmax)
                state, timer = "stance", 0.0
            prev_zdot = zdot

        elif state == "stance" and timer > 0.20:
            # re-compress for next jump
            p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
                targetPosition=compress, positionGain=posGain, velocityGain=velGain, force=Fmax)
            state, timer = "compress", 0.0

        # log
        q = p.getJointState(robot, jid)[0]
        z = p.getBasePositionAndOrientation(robot)[0][2]
        wr.writerow([t,state,q,z,h_ref,release, 1 if (state=="flight" and prev_zdot>0 and p.getBaseVelocity(robot)[0][2]<=0) else 0, last_apex_z or ""])

        # step
        p.stepSimulation(); time.sleep(dt)
        t += dt; timer += dt

        if key_pressed('q'): break

p.disconnect()
print(f"Saved {LOG.resolve()}")
