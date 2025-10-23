import time, pathlib, csv
import pybullet as p, pybullet_data

LOG = pathlib.Path("data/logs/mr_springs_bullet.csv")
LOG.parent.mkdir(parents=True, exist_ok=True)

p.connect(p.GUI)  # or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0,0,-9.81)
p.loadURDF("plane.urdf")
robot = p.loadURDF("urdf/mr_springs.urdf", [0,0,0.01])

jid = 0  # prismatic joint index
dt = 1/240

# STORE: compress to 0.00 m
p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, targetPosition=0.00, force=500)
for _ in range(240):
    p.stepSimulation(); time.sleep(dt)

# LATCH/RELEASE: free the joint (no motor force) so spring effect is zero in Bullet,
# but we simulate a release by quickly moving to a small extension then letting it coast.
p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, targetPosition=0.05, force=0)

with LOG.open("w", newline="") as f:
    wr = csv.writer(f); wr.writerow(["t","q","z"])
    t=0.0
    for _ in range(2400):
        p.stepSimulation()
        q = p.getJointState(robot, jid)[0]
        z = p.getBasePositionAndOrientation(robot)[0][2]
        wr.writerow([t,q,z])
        t += dt
        time.sleep(dt)

p.disconnect()
print(f"Saved {LOG.resolve()}")
