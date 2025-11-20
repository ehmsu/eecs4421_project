#!/usr/bin/env python3
"""Check if wheels affect the center of mass of the robot."""

import pybullet as p
import pybullet_data
import numpy as np
from pathlib import Path
import os

def get_com_position(body_id):
    """Calculate the center of mass position of the robot."""
    # Get base link position and orientation
    base_pos, base_ori = p.getBasePositionAndOrientation(body_id)
    
    # Get total mass and weighted COM
    total_mass = 0.0
    weighted_com = np.array([0.0, 0.0, 0.0])
    
    # Base link
    dyn_info = p.getDynamicsInfo(body_id, -1)
    mass = dyn_info[0]
    com_local = dyn_info[3]
    if mass > 0:
        # Transform COM to world coordinates
        com_world = p.multiplyTransforms(base_pos, base_ori, com_local, [0, 0, 0, 1])[0]
        weighted_com += np.array(com_world) * mass
        total_mass += mass
    
    # All other links
    num_joints = p.getNumJoints(body_id)
    for i in range(num_joints):
        link_state = p.getLinkState(body_id, i, computeForwardKinematics=True)
        link_pos = link_state[0]
        link_ori = link_state[1]
        
        dyn_info = p.getDynamicsInfo(body_id, i)
        mass = dyn_info[0]
        com_local = dyn_info[3]
        if mass > 0:
            # Transform COM to world coordinates
            com_world = p.multiplyTransforms(link_pos, link_ori, com_local, [0, 0, 0, 1])[0]
            weighted_com += np.array(com_world) * mass
            total_mass += mass
    
    if total_mass > 0:
        com = weighted_com / total_mass
        return com, total_mass
    else:
        return np.array([0, 0, 0]), 0.0

def check_wheel_impact():
    """Check if wheels significantly affect the center of mass."""
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    
    # Find URDF
    urdf_path = None
    override = os.environ.get("MR_SPRINGS_URDF")
    if override and Path(override).exists():
        urdf_path = override
    else:
        assets = Path("assets")
        if assets.exists():
            urdfs = sorted(assets.glob("**/*.urdf"))
            if urdfs:
                urdf_path = str(urdfs[0])
    
    if not urdf_path:
        print("ERROR: No URDF found!")
        return
    
    print(f"Loading URDF: {urdf_path}")
    
    # Load robot with all parts
    urdf_dir = str(Path(urdf_path).parent.absolute())
    meshes_dir = str(Path(urdf_dir) / "meshes")
    p.setAdditionalSearchPath(urdf_dir)
    if Path(meshes_dir).exists():
        p.setAdditionalSearchPath(meshes_dir)
    
    body = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)
    
    # Get COM with all parts
    com_full, mass_full = get_com_position(body)
    print(f"\n=== Full Robot (with wheels) ===")
    print(f"Total mass: {mass_full:.4f} kg")
    print(f"Center of Mass: ({com_full[0]:.4f}, {com_full[1]:.4f}, {com_full[2]:.4f}) m")
    
    # Get wheel link indices
    num_joints = p.getNumJoints(body)
    wheel_links = []
    wheel_masses = []
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(body, i)
        link_name = joint_info[12].decode('utf-8') if isinstance(joint_info[12], bytes) else joint_info[12]
        if 'wheel' in link_name.lower():
            wheel_links.append(i)
            dyn_info = p.getDynamicsInfo(body, i)
            mass = dyn_info[0]
            wheel_masses.append(mass)
            print(f"  Found wheel link: {link_name} (index {i}, mass {mass:.4f} kg)")
    
    if not wheel_links:
        print("\nNo wheel links found in URDF!")
        p.disconnect()
        return
    
    # Calculate COM without wheels (by zeroing their mass)
    total_mass_no_wheels = mass_full - sum(wheel_masses)
    print(f"\n=== Robot without wheels (estimated) ===")
    print(f"Total mass: {total_mass_no_wheels:.4f} kg")
    print(f"Wheel mass removed: {sum(wheel_masses):.4f} kg ({100*sum(wheel_masses)/mass_full:.2f}% of total)")
    
    # Estimate COM shift (rough calculation)
    # This is approximate - we'd need to actually remove wheels for exact calculation
    wheel_com_contrib = np.array([0.0, 0.0, 0.0])
    for i in wheel_links:
        link_state = p.getLinkState(body, i, computeForwardKinematics=True)
        link_pos = link_state[0]
        link_ori = link_state[1]
        dyn_info = p.getDynamicsInfo(body, i)
        mass = dyn_info[0]
        com_local = dyn_info[3]
        if mass > 0:
            com_world = p.multiplyTransforms(link_pos, link_ori, com_local, [0, 0, 0, 1])[0]
            wheel_com_contrib += np.array(com_world) * mass
    
    if sum(wheel_masses) > 0:
        wheel_com = wheel_com_contrib / sum(wheel_masses)
        # Estimate COM without wheels (weighted average)
        com_no_wheels = (com_full * mass_full - wheel_com_contrib) / total_mass_no_wheels
        com_shift = com_no_wheels - com_full
        
        print(f"Wheel COM: ({wheel_com[0]:.4f}, {wheel_com[1]:.4f}, {wheel_com[2]:.4f}) m")
        print(f"Estimated COM without wheels: ({com_no_wheels[0]:.4f}, {com_no_wheels[1]:.4f}, {com_no_wheels[2]:.4f}) m")
        print(f"COM shift: ({com_shift[0]:.4f}, {com_shift[1]:.4f}, {com_shift[2]:.4f}) m")
        print(f"COM shift magnitude: {np.linalg.norm(com_shift):.4f} m")
        
        # Check if shift is significant (> 1cm)
        if np.linalg.norm(com_shift) > 0.01:
            print(f"\n⚠️  WARNING: Wheels cause significant COM shift ({np.linalg.norm(com_shift)*100:.2f} cm)")
            print("   Consider removing wheels if they're not used for locomotion.")
        else:
            print(f"\n✓ Wheels cause minimal COM shift ({np.linalg.norm(com_shift)*100:.2f} cm)")
            print("   Wheels can remain if desired (minimal impact on dynamics).")
    
    p.disconnect()

if __name__ == "__main__":
    check_wheel_impact()

