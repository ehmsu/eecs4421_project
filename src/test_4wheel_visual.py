#!/usr/bin/env python3
import pybullet as p, pybullet_data, time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot = p.loadURDF("../urdf/four_wheeled_jumping_robot.urdf", [0,0,0.3])

print("Loaded robot with", p.getNumJoints(robot), "joints (should be 8: 4 wheels, 4 springs)")

for _ in range(2000):
    p.stepSimulation()
    time.sleep(1/240.0)
p.disconnect()
