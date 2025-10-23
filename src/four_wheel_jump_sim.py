#!/usr/bin/env python3
import pybullet as p, pybullet_data, time, math

print("=== Four-Wheeled Jumping Robot Simulation ===")

# Initialize simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0,0,-9.81)
p.setTimeStep(1/240.)

# Ground and robot loading
p.loadURDF("plane.urdf")
robot = p.loadURDF("../urdf/four_wheeled_jumping_robot.urdf", [0, 0, 0.5])

# Control setup (example placeholder joint IDs)
jump_joint = 0  # assigned first (only one prismatic joint for now)
wheel_joints = []  # placeholder if wheel joints added later

jump_force = 250.0
dt = 1/240.0

print("Controls: J=jump, Q=quit")

while p.isConnected():
    keys = p.getKeyboardEvents()

    if ord('q') in keys:
        break

    if ord('j') in keys:
        p.setJointMotorControl2(robot, jump_joint, controlMode=p.POSITION_CONTROL,
                                targetPosition=0.15, force=jump_force)

    p.stepSimulation()
    pos, orn = p.getBasePositionAndOrientation(robot)
    pitch, roll, yaw = p.getEulerFromQuaternion(orn)
    
    # Check stability
    if abs(pitch) > 0.2 or abs(roll) > 0.2:
        print(f"Warning: robot tilting - pitch={pitch:.2f}, roll={roll:.2f}")

    time.sleep(dt)
