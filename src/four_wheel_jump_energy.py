#!/usr/bin/env python3
import pybullet as p, pybullet_data, time, csv, math

print("Running energy monitor...")

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0,0,-9.81)
p.setTimeStep(1/240.)
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("../urdf/four_wheeled_jumping_robot.urdf", [0,0,0.5])
dt = 1/240.
mass = 1.5
k = 800.0
l0 = 0.6

with open("../data/logs/energy_log.csv", "w") as f:
    w = csv.writer(f)
    w.writerow(["t","z","vz","E_total"])
    t = 0
    for i in range(2000):
        pos, orn = p.getBasePositionAndOrientation(robot)
        vel, _ = p.getBaseVelocity(robot)
        z = pos[2]
        vz = vel[2]
        E = 0.5*mass*vz**2 + 0.5*k*(l0 - z)**2 + mass*9.81*z
        w.writerow([t,z,vz,E])
        p.stepSimulation()
        time.sleep(dt)
        t += dt

p.disconnect()
print("Logged energy values at ../data/logs/energy_log.csv")
