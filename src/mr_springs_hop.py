#!/usr/bin/env python3
import time, pathlib, csv
import pybullet as p, pybullet_data

def key_pressed(ch):
    ev = p.getKeyboardEvents(); code = ord(ch)
    return ev.get(code) in (p.KEY_IS_DOWN, p.KEY_WAS_TRIGGERED)

LOG = pathlib.Path("data/logs/mr_springs_live.csv")
LOG.parent.mkdir(parents=True, exist_ok=True)

cid = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setTimeStep(1/240.0)
p.setGravity(0,0,-9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.loadURDF("plane.urdf")
robot = p.loadURDF("urdf/mr_springs.urdf",[0,0,0.02])
jid = 0

# spring-like control via motor
l0 = 0.05; compress = 0.00; release = 0.15
posGain = 3.0e-3; velGain = 4.0e-1; Fmax = 1500.0
dt = 1/240.0

state, timer = "compress", 0.0
p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
    targetPosition=compress, positionGain=posGain, velocityGain=velGain, force=Fmax)
p.addUserDebugText("Press 'q' to quit", [0,0,1.2], [1,1,1], 1.5, 0)

t = 0.0
with LOG.open("w", newline="") as f:
    wr = csv.writer(f); wr.writerow(["t","state","q","z"])
    while p.isConnected():
        if state == "compress" and timer > 0.35:
            p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
                targetPosition=release, positionGain=posGain, velocityGain=velGain, force=Fmax)
            state, timer = "release", 0.0
        elif state == "release" and timer > 0.10:
            p.setJointMotorControl2(robot,jid,p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)
            state, timer = "coast", 0.0
        elif state == "coast":
            if p.getContactPoints(bodyA=robot):
                p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
                    targetPosition=l0, positionGain=posGain, velocityGain=velGain, force=Fmax)
                state, timer = "stance", 0.0
        elif state == "stance" and timer > 0.25:
            p.setJointMotorControl2(robot,jid,p.POSITION_CONTROL,
                targetPosition=compress, positionGain=posGain, velocityGain=velGain, force=Fmax)
            state, timer = "compress", 0.0

        q = p.getJointState(robot, jid)[0]
        z = p.getBasePositionAndOrientation(robot)[0][2]
        wr.writerow([t,state,q,z])

        p.stepSimulation(); time.sleep(dt)
        t += dt; timer += dt
        if key_pressed('q'): break

p.disconnect()
print(f"Saved {LOG.resolve()}")
