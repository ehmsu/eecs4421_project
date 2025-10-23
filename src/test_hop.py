#!/usr/bin/env python3
import pybullet as p, pybullet_data, time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240.)
p.loadURDF("plane.urdf")
robot = p.loadURDF("../urdf/four_wheeled_jumping_robot.urdf", [0,0,0.2])

jump_joint = 0

compress = 0.01      # Compressed spring position (close to ground)
release = 0.13       # Max jump extension
k_spring = 700       # Stronger
charging = False

print("Press SPACE to hop! Press q to quit.")

while p.isConnected():
    keys = p.getKeyboardEvents()
    # Charge and jump sequence
    if ord(' ') in keys and not charging:
        charging = True
        t = 0
        # Step 1: Compress
        p.setJointMotorControl2(robot, jump_joint, p.POSITION_CONTROL,
                        targetPosition=compress, force=k_spring)
        print("Charging...")
    if charging:
        t += 1
        if t > 45:
            # Step 2: Release
            print("Release!")
            p.setJointMotorControl2(robot, jump_joint, p.POSITION_CONTROL,
                        targetPosition=release, force=k_spring)
            charging = False
    if ord('q') in keys:
        break
    p.stepSimulation()
    time.sleep(1/240.)
