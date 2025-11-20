import pybullet as p, pybullet_data, numpy as np
import matplotlib
matplotlib.use("Agg")  # no GUI popups
import matplotlib.pyplot as plt
import csv, logging, os
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
FLIGHT, STANCE = 0, 1

class Hopper1D:
    def __init__(self, mode=p.DIRECT, sim_timestep=1/240.0):
        self.sim_mode = mode
        try:
            self.client = p.connect(p.GUI if mode==p.GUI else p.DIRECT)
        except Exception as e:
            logging.warning(f"GUI failed ({e}); using DIRECT.")
            self.client = p.connect(p.DIRECT)
            self.sim_mode = p.DIRECT

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0,0,-9.81)
        self.dt = sim_timestep
        p.setTimeStep(self.dt)
        # Optimize physics solver for efficiency
        p.setPhysicsEngineParameter(
            numSolverIterations=10,  # Reduced from default for efficiency
            enableConeFriction=0,    # Disable cone friction (not needed for 1D)
            deterministicOverlappingPairs=1  # Deterministic for reproducibility
        )
        self.plane = p.loadURDF("plane.urdf")
        # Ground plane properties - will be used when robot tips (collisions enabled)
        # For normal 1D hopping, collisions are disabled and we use spring-damper model
        p.changeDynamics(self.plane, -1, 
                        lateralFriction=0.5,  # Some friction for natural sliding/rolling when tipped
                        restitution=0.3,      # Some bounce for natural physics when tipped
                        contactStiffness=1e4,  # Contact stiffness
                        contactDamping=50)     # Contact damping

        self.body, self.motion_mode, self.motion_link_idx, self.has_joint = self._load_robot()
        self.mass = self._infer_mass(self.body, self.motion_link_idx)
        # Store the base orientation (-180° rotation around Z axis to turn robot around)
        self.base_orientation = p.getQuaternionFromEuler([0, 0, -np.pi])
        # Calculate the offset from base COM to the lowest point of the robot (spring tips)
        # This is needed to detect actual ground contact
        self._calculate_ground_offset()
        if self.has_joint:
            # Don't set any joint control - let the joint move freely under gravity
            # We'll apply forces via TORQUE_CONTROL in _apply_force when needed
            # Reduce joint damping to allow free motion
            p.changeDynamics(self.body, self.motion_link_idx, 
                           linearDamping=0.0, angularDamping=0.0,
                           jointDamping=0.0)
        # Optimize robot link dynamics for efficiency
        # Keep collisions enabled so model stays together visually
        num_joints = p.getNumJoints(self.body)
        for i in range(-1, num_joints):
            p.changeDynamics(self.body, i, 
                           lateralFriction=0.0,  # No friction needed (1D motion, clamp mode)
                           restitution=0.0,
                           contactStiffness=1e4,  # Reduced for efficiency
                           contactDamping=50,     # Reduced for efficiency
                           activationState=p.ACTIVATION_STATE_WAKE_UP)  # Wake up robot so it responds to gravity
        # Disable collision between ALL robot links and ground plane - we handle contact via spring-damper model
        # All internal collisions between robot parts remain enabled by default (fixed joints keep them together)
        p.setCollisionFilterPair(self.body, self.plane, -1, -1, enableCollision=0)
        # Also disable collisions for all individual links with ground
        num_joints = p.getNumJoints(self.body)
        for i in range(-1, num_joints):
            p.setCollisionFilterPair(self.body, self.plane, i, -1, enableCollision=0)
        # Lock ALL joints (fixed, prismatic, revolute) so the robot moves as one rigid body
        # This prevents wheels, springs, and other parts from sliding away
        for i in range(num_joints):
            ji = p.getJointInfo(self.body, i)
            joint_name = ji[1].decode('utf-8')
            joint_type = ji[2]
            
            if joint_type == p.JOINT_FIXED:
                # Fixed joints are already locked, but ensure they stay that way
                pass
            elif joint_type == p.JOINT_PRISMATIC:
                # Lock prismatic joints (springs, moving frame) - robot should move as one body
                # Use very high force to prevent parts from sliding when manually pushed
                p.setJointMotorControl2(self.body, i, p.POSITION_CONTROL, 
                                       targetPosition=0, force=1e20)
                p.changeDynamics(self.body, i, 
                               jointLowerLimit=0, jointUpperLimit=0)
            elif joint_type == p.JOINT_REVOLUTE:
                # Lock revolute joints (wheels, gears) - they shouldn't spin in 1D hopping
                # Use very high force to prevent parts from rotating when manually pushed
                p.setJointMotorControl2(self.body, i, p.POSITION_CONTROL, 
                                       targetPosition=0, force=1e20)
                p.changeDynamics(self.body, i, 
                               jointLowerLimit=0, jointUpperLimit=0)

        # Model/Control (tuned for stable, consistent hopping)
        # Spring stiffness: moderate for stable hopping (~12-14 Hz natural frequency)
        # Damping: increased to allow longer stance phase and smoother bouncing
        self.K_h=4000.0; self.B_h=45.0; self.L_rest=0.30
        self.h_target=0.80; self.k_raibert=0.30; self.u_h_max=0.08
        # Stance time estimate: tuned to match actual stance duration (~0.05-0.06s)
        self.T_stance_estimate=0.055; self.pulse_phase_start=0.25; self.pulse_width=0.50
        # Increase F_max_scalar significantly to allow sufficient force for hopping
        # At z=-0.1, L=0.3, force = K*(L-z) = 4000*0.4 = 1600N needed
        self.F_max_scalar=500.0; self.F_peak_guard=self.F_max_scalar*self.mass*9.81
        self.ENERGY_RECOVERY=False

        self.state=FLIGHT; self.last_vz=0.0; self.stance_t0=0.0
        self.current_u_h=0.0; self.F_peak_stance=0.0
        self.log_data=[]; self.apex_history=[]
        self.h_start=0.80; self._reset_height(self.h_start)
        # Track if we've had a real stance phase (landed) before counting apex
        # This prevents counting oscillations in the air as "hops"
        self.has_landed_once = False
        # Disturbance testing: track horizontal drift and recovery
        self.disturbance_applied = False
        self.disturbance_force = [0.0, 0.0, 0.0]  # [Fx, Fy, Fz] - horizontal pushes
        self.disturbance_time = None
        self.max_drift = 0.0  # Maximum horizontal drift from origin
        self.recovery_time = None  # Time to return to within threshold of origin
        # Track if robot has tipped over - once tipped, stop hopping permanently
        self.has_tipped = False

    # ---------- loading ----------
    def _find_first_urdf(self):
        override=os.environ.get("MR_SPRINGS_URDF")
        if override and Path(override).exists():
            logging.info(f"Using MR_SPRINGS_URDF={override}")
            return override
        assets=Path("assets")
        if assets.exists():
            urdfs=sorted(assets.glob("**/*.urdf"))
            if urdfs: return str(urdfs[0])
        return None

    def _load_robot(self):
        urdf_path=self._find_first_urdf()
        if urdf_path is None:
            Path("assets").mkdir(exist_ok=True)
            fb="assets/mr_springs.urdf"; Path(fb).write_text(self._fallback_urdf_text())
            urdf_path=fb
        else:
            # Add URDF directory and meshes directory to PyBullet search path
            urdf_dir = str(Path(urdf_path).parent.absolute())
            meshes_dir = str(Path(urdf_dir) / "meshes")
            # PyBullet searches relative to URDF file location, so add the directory containing the URDF
            p.setAdditionalSearchPath(urdf_dir)
            if Path(meshes_dir).exists():
                p.setAdditionalSearchPath(meshes_dir)
            # Also add parent directory in case meshes are referenced relatively (e.g., ../meshes/)
            parent_dir = str(Path(urdf_dir).parent.absolute())
            p.setAdditionalSearchPath(parent_dir)
            # Add the meshes directory from parent as well (for ../meshes/ paths)
            parent_meshes = str(Path(parent_dir) / "meshes")
            if Path(parent_meshes).exists():
                p.setAdditionalSearchPath(parent_meshes)
        try:
            # Check if URDF has a "world" link - old structure that doesn't work
            urdf_text = Path(urdf_path).read_text() if Path(urdf_path).exists() else ""
            has_world_link = '<link name="world"' in urdf_text or 'parent link="world"' in urdf_text
            
            # New structure without world link - base_link is root
            # Use useFixedBase=False so robot can move, then clamp to 1D in code
            use_fixed = False
            # -180° rotation around Z axis to turn robot around
            base_orientation = p.getQuaternionFromEuler([0, 0, -np.pi])
            body=p.loadURDF(urdf_path,useFixedBase=use_fixed,baseOrientation=base_orientation,flags=p.URDF_USE_INERTIA_FROM_FILE)
            logging.info(f"Loaded URDF: {urdf_path} (useFixedBase=False, no rotation, clamp mode)")
        except Exception as e:
            logging.error(f"URDF load failed: {urdf_path} -> {e}; using fallback.")
            fb="assets/mr_springs.urdf"; Path(fb).write_text(self._fallback_urdf_text())
            body=p.loadURDF(fb,useFixedBase=False,flags=p.URDF_USE_INERTIA_FROM_FILE)
            logging.info(f"Loaded fallback URDF: {fb}")

        # Check if URDF has world link - if so, force clamp mode (joint mode doesn't work with world link)
        urdf_text = Path(urdf_path).read_text() if Path(urdf_path).exists() else ""
        has_world_link = '<link name="world"' in urdf_text or 'parent link="world"' in urdf_text
        
        if has_world_link:
            # World link structure - use clamp mode (joint mode is broken with world link)
            anchor=self._pick_heaviest_link(body)
            logging.info(f"World link detected; using clamp mode on link {anchor} (joint mode not supported).")
            return body,"clamp",anchor,False
        
        # No world link - use clamp mode (robot moves as one body)
        # Even if there are internal prismatic joints (like moving_frame_to_base_link),
        # we want the whole robot to move together, so use clamp mode
        anchor=self._pick_heaviest_link(body)
        logging.info(f"Using clamp mode on link {anchor} (robot moves as one body).")
        return body,"clamp",anchor,False

    def _pick_heaviest_link(self, body):
        best=-1; mmax=-1.0
        bm=p.getDynamicsInfo(body,-1)[0]; 
        if bm>mmax: mmax=bm; best=-1
        for j in range(p.getNumJoints(body)):
            m=p.getDynamicsInfo(body,j)[0]
            if m>mmax: mmax=m; best=j
        return best

    def _infer_mass(self, body, link):
        try:
            m=p.getDynamicsInfo(body,link)[0]
            if m>0: return m
        except Exception: pass
        total=max(0.0,p.getDynamicsInfo(body,-1)[0])
        for j in range(p.getNumJoints(body)):
            total+=max(0.0,p.getDynamicsInfo(body,j)[0])
        return total if total>0 else 1.0

    def _fallback_urdf_text(self):
        return """<?xml version="1.0"?>
<robot name="mr_springs_fallback">
  <link name="world"/>
  <link name="chassis">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0.01" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <joint name="z_joint" type="prismatic">
    <parent link="world"/><child link="chassis"/>
    <axis xyz="0 0 1"/><limit lower="0" upper="5" effort="100" velocity="10"/>
  </joint>
</robot>
"""

    # ---------- state/helpers ----------
    def _reset_height(self,z0):
        if self.motion_mode=="joint" and self.has_joint:
            p.resetJointState(self.body,self.motion_link_idx,targetValue=z0,targetVelocity=0.0)
        else:
            # Use stored orientation (springs down, wheels up)
            p.resetBasePositionAndOrientation(self.body,[0,0,z0],self.base_orientation)
            p.resetBaseVelocity(self.body,[0,0,0],[0,0,0])
        # Wake up the robot after reset so it responds to gravity
        p.changeDynamics(self.body, -1, activationState=p.ACTIVATION_STATE_WAKE_UP)

    def _get_z_vz(self):
        if self.motion_mode=="joint" and self.has_joint:
            # For joint mode, get joint position directly (not affected by base orientation)
            state = p.getJointState(self.body, self.motion_link_idx)
            z = state[0]  # Joint position
            vz = state[1]  # Joint velocity
            # Allow z to be slightly negative for spring compression
            # But clamp it for display/logging purposes
            return z, vz
        if self.motion_link_idx==-1:
            pos,_=p.getBasePositionAndOrientation(self.body); lin,_=p.getBaseVelocity(self.body)
            return pos[2], lin[2]
        ls=p.getLinkState(self.body,self.motion_link_idx,computeLinkVelocity=1)
        return ls[0][2], ls[6][2]

    def _contacting(self): return len(p.getContactPoints(self.body,self.plane))>0
    
    def _calculate_ground_offset(self):
        """Calculate the offset from base COM to the lowest point of the robot (spring tips).
        This is used to detect actual ground contact, not just COM position."""
        # Get base position and orientation
        base_pos, base_ori = p.getBasePositionAndOrientation(self.body)
        # Find the lowest point of all links in world coordinates
        lowest_z_world = float('inf')
        for i in range(-1, p.getNumJoints(self.body)):
            # Get AABB in world coordinates
            aabb_min, aabb_max = p.getAABB(self.body, i)
            if aabb_min[2] < lowest_z_world:
                lowest_z_world = aabb_min[2]
        # Calculate offset: how far below the base COM is the lowest point?
        self.ground_offset = base_pos[2] - lowest_z_world
        logging.info(f"Ground offset (base COM to lowest point): {self.ground_offset:.3f}m")
        return self.ground_offset
    
    def _get_lowest_point_z(self):
        """Get the Z coordinate of the lowest point of the robot (spring tips) in world coordinates."""
        # Get base COM position
        base_pos, _ = p.getBasePositionAndOrientation(self.body)
        # Return base COM z minus the offset to get the lowest point
        return base_pos[2] - self.ground_offset

    def _apply_force(self,fz):
        if abs(fz)<=0: return
        if self.motion_mode=="joint" and self.has_joint:
            # For prismatic joint, apply force directly to the joint
            # This overcomes contact constraints better than external force
            # Force is in the joint's axis direction (Z-axis for vertical motion)
            p.setJointMotorControl2(self.body, self.motion_link_idx,
                                   p.TORQUE_CONTROL,
                                   force=fz)
        else:
            # Apply force to base link in world frame (upward +Z direction)
            # This works for clamp mode
            p.applyExternalForce(self.body, -1, [0,0,fz], [0,0,0], p.WORLD_FRAME)

    def apply_disturbance(self, force_x=0.0, force_y=0.0, duration=0.1):
        """Apply a horizontal disturbance force (push) to test robustness.
        
        Args:
            force_x: Force in X direction (N)
            force_y: Force in Y direction (N)
            duration: How long to apply the force (s)
        """
        self.disturbance_force = [force_x, force_y, 0.0]
        self.disturbance_duration = duration
        self.disturbance_applied = True
        self.disturbance_time = None  # Will be set when first applied
        self.max_drift = 0.0
        self.recovery_time = None
        logging.info(f"Disturbance scheduled: Fx={force_x:.2f}N, Fy={force_y:.2f}N, duration={duration:.3f}s")

    def _clamp_to_1d(self, allow_drift=False):
        """Clamp robot to 1D motion. If allow_drift=True, track drift but don't reset it (for disturbance testing)."""
        if self.motion_mode!="clamp": return
        # Always use base link for clamp mode
        pos,ori=p.getBasePositionAndOrientation(self.body)
        lin,ang=p.getBaseVelocity(self.body)
        # Store Z velocity before any resets
        vz_preserve = lin[2]
        
        # Track horizontal drift for disturbance testing
        horizontal_drift = np.sqrt(pos[0]**2 + pos[1]**2)
        if horizontal_drift > self.max_drift:
            self.max_drift = horizontal_drift
        
        # Check if recovered (drift < 0.01m threshold)
        # Note: recovery check happens in step() where we have access to time t
        
        # Only clamp if not allowing drift (normal operation)
        if not allow_drift:
            # Only clamp if position or orientation has drifted from desired state
            # Check if position is off X/Y axis or orientation has changed
            if abs(pos[0]) > 1e-6 or abs(pos[1]) > 1e-6:
                p.resetBasePositionAndOrientation(self.body,[0,0,pos[2]],self.base_orientation)
            # Check if velocity has X/Y components or angular velocity - preserve Z velocity
            if abs(lin[0]) > 1e-6 or abs(lin[1]) > 1e-6 or abs(ang[0]) > 1e-6 or abs(ang[1]) > 1e-6 or abs(ang[2]) > 1e-6:
                p.resetBaseVelocity(self.body,[0,0,vz_preserve],[0,0,0])
        else:
            # When allowing drift (disturbance testing), don't reset position/orientation
            # But still preserve Z velocity for vertical hopping
            # Only reset if Z velocity is being affected by horizontal motion
            # Actually, let it continue naturally - don't interfere with the displacement
            pass
        
        # Re-lock all joints every step to ensure parts stay together (wheels, springs don't slide)
        # This is critical when manually pushing the robot in GUI mode
        num_joints = p.getNumJoints(self.body)
        for i in range(num_joints):
            ji = p.getJointInfo(self.body, i)
            joint_type = ji[2]
            if joint_type == p.JOINT_PRISMATIC or joint_type == p.JOINT_REVOLUTE:
                # Keep joints locked at position 0 with very high force
                # This prevents parts from sliding away when manually pushed
                p.setJointMotorControl2(self.body, i, p.POSITION_CONTROL, 
                                       targetPosition=0, force=1e20)

    # ---------- control ----------
    def set_target_height(self,h):
        self.h_target=float(h); logging.info(f"h* = {self.h_target:.2f} m")

    def _calc_u_h(self):
        last=self.h_start if not self.apex_history else self.apex_history[-1][1]
        return float(np.clip(self.k_raibert*(self.h_target-last),0.0,self.u_h_max))

    def _spring_damper(self,z,vz,L):
        # Per proposal: STANCE: m*z_ddot = k(l0 - z) - b*z_dot - mg
        # The spring-damper force (excluding gravity, which PyBullet handles) is:
        # F = k(l0 - z) - b*z_dot
        # z is the COM height above ground
        # L is the rest length of the spring (l0 = L_rest = 0.3m)
        # When spring tips are at ground (z=0), COM is at z=ground_offset
        # Spring compression = how much the COM is below its position when tips are at ground
        # Effective compression = (ground_offset + L_rest) - z
        # But we use the simpler model: spring is compressed when z < (ground_offset + L_rest)
        spring_rest_com_height = self.ground_offset + L  # COM height when tips are at ground and spring is at rest
        if z >= spring_rest_com_height: return 0.0
        # Calculate spring force: F_spring = K*(compression) when compressed
        compression = spring_rest_com_height - z
        # Damping force: F_damp = -B*vz (opposes velocity)
        # When vz < 0 (moving down), -B*vz > 0 (pushes up) ✓
        # When vz > 0 (moving up), -B*vz < 0 (pulls down) ✓
        F = self.K_h * compression - self.B_h * vz
        # Add strong repulsive force when spring tips are very close to ground to prevent penetration
        lowest_z = self._get_lowest_point_z()
        if lowest_z < 0.01:  # Tips within 1cm of ground
            penetration_penalty = self.K_h * (0.01 - lowest_z) * 30  # Very strong repulsive force
            F += penetration_penalty
        # Force must be non-negative (can't pull down through ground)
        F = max(0.0, F)
        return min(F, self.F_peak_guard)

    def step(self,t):
        z,vz=self._get_z_vz()
        # Clamp z to prevent negative values (ground is at z=0)
        z = max(0.0, z)
        u_h_applied=0.0; F_contact=0.0
        
        # Apply disturbance force if scheduled
        if self.disturbance_applied and (self.disturbance_force[0] != 0.0 or self.disturbance_force[1] != 0.0):
            if self.disturbance_time is None:
                self.disturbance_time = t
            elapsed = t - self.disturbance_time
            if elapsed < self.disturbance_duration:
                # Apply horizontal force to base link
                p.applyExternalForce(self.body, -1, 
                                   [self.disturbance_force[0], self.disturbance_force[1], 0.0],
                                   [0, 0, 0], p.WORLD_FRAME)
            else:
                # Disturbance period ended, stop applying force
                self.disturbance_force = [0.0, 0.0, 0.0]

        # Check if robot is tipped (before state check, so we can enable collisions)
        pos_check, ori_check = p.getBasePositionAndOrientation(self.body)
        euler_check = p.getEulerFromQuaternion(ori_check)
        tilt_angle_check = np.sqrt(euler_check[0]**2 + euler_check[1]**2)
        max_tilt_check = np.pi / 6  # 30 degrees
        horizontal_drift_check = np.sqrt(pos_check[0]**2 + pos_check[1]**2)
        max_drift_check = 0.5  # 50cm
        is_tipped_check = tilt_angle_check >= max_tilt_check or horizontal_drift_check >= max_drift_check
        
        if is_tipped_check and not self.has_tipped:
            # Just tipped - enable collisions with ground for pure physics
            self.has_tipped = True
            # Re-enable collisions between robot and ground plane for natural physics
            p.setCollisionFilterPair(self.body, self.plane, -1, -1, enableCollision=1)
            num_joints = p.getNumJoints(self.body)
            for i in range(-1, num_joints):
                p.setCollisionFilterPair(self.body, self.plane, i, -1, enableCollision=1)
        
        if self.state==FLIGHT:
            # If robot has tipped, don't allow any more hopping - just let it fall
            if self.has_tipped:
                F_contact = 0.0
            else:
                # Detect apex (velocity crosses zero from positive to negative)
                # BUT only count it as a hop if we've actually landed at least once
                # This prevents counting oscillations in the air as "hops"
                if getattr(self,'last_vz',0.0)>0 and vz<=0 and self.has_landed_once:
                    self.apex_history.append((t,z,getattr(self,'F_peak_stance',0.0),self.h_target))
                    self.F_peak_stance=0.0
                # Per proposal: "touchdown when z = l0 with z_dot < 0"
                # But we need to detect when the actual spring tips touch the ground, not the COM
                # Get the lowest point of the robot (spring tips)
                lowest_z = self._get_lowest_point_z()
                # Touchdown when spring tips are very close to ground (within 1.5cm) and moving down
                # The spring compresses when tips touch ground, so COM will be at L_rest when tips are at ground
                touchdown_threshold = 0.015  # 1.5cm - spring tips are touching/near ground
                # Only allow touchdown if robot hasn't tipped
                if lowest_z <= touchdown_threshold and vz < 0 and not self.has_tipped:
                    self.state=STANCE; self.stance_t0=t; self.current_u_h=self._calc_u_h()
                    self.has_landed_once = True  # Mark that we've actually landed
        else:
            # STANCE phase: apply spring-damper force
            # But only if robot is not too tilted (for realistic animation)
            pos, ori = p.getBasePositionAndOrientation(self.body)
            # Check if robot is too tilted (angular displacement from base orientation)
            euler = p.getEulerFromQuaternion(ori)
            tilt_angle = np.sqrt(euler[0]**2 + euler[1]**2)  # Roll and pitch angles
            max_tilt = np.pi / 6  # 30 degrees - if tilted more than this, stop hopping
            
            # Also check horizontal drift - if too far displaced, stop hopping
            horizontal_drift = np.sqrt(pos[0]**2 + pos[1]**2)
            max_drift = 0.5  # 50cm - if drifted more than this, stop hopping
            
            # Only apply spring force if robot is reasonably upright and not too displaced
            if tilt_angle < max_tilt and horizontal_drift < max_drift:
                ts=t-self.stance_t0
                p0=self.pulse_phase_start*self.T_stance_estimate
                p1=p0+self.pulse_width*self.T_stance_estimate
                L=self.L_rest
                if p0<ts<p1:
                    L=self.L_rest+self.current_u_h
                    u_h_applied=self.current_u_h
                # Apply spring-damper force when compressed (z < L)
                # Per proposal: F = k(l0 - z) - b*z_dot during STANCE
                F_contact=self._spring_damper(z,vz,L)
                if F_contact>0:
                    self._apply_force(F_contact)
                    self.F_peak_stance=max(getattr(self,'F_peak_stance',0.0),F_contact)
            else:
                # Robot is too tilted or displaced - stop applying spring force, let it fall
                F_contact = 0.0
                # Mark as tipped and transition to FLIGHT so it falls naturally
                if not self.has_tipped:
                    # Just tipped - enable collisions with ground for pure physics
                    self.has_tipped = True
                    # Re-enable collisions between robot and ground plane for natural physics
                    p.setCollisionFilterPair(self.body, self.plane, -1, -1, enableCollision=1)
                    num_joints = p.getNumJoints(self.body)
                    for i in range(-1, num_joints):
                        p.setCollisionFilterPair(self.body, self.plane, i, -1, enableCollision=1)
                self.state = FLIGHT
            # Per proposal: "liftoff when spring re-extends to z = l0 with z_dot > 0"
            # OR when contact force becomes zero (spring fully extended)
            # Get the lowest point of the robot (spring tips)
            lowest_z = self._get_lowest_point_z()
            # Liftoff when spring tips have extended back up significantly and moving up
            # The spring should compress and then extend - wait for proper extension
            # Use a slightly lower threshold to allow natural spring extension
            liftoff_threshold = 0.04  # 4cm - spring tips have extended back up
            if lowest_z > liftoff_threshold and vz > 0:  # Tips are well above ground and moving up
                self.state=FLIGHT
            # Also liftoff if force becomes zero (spring fully extended, no contact) and tips are above ground
            if F_contact <= 0.0 and lowest_z > 0.02:
                self.state=FLIGHT

        p.stepSimulation()
        # After physics step, prevent ground penetration (CRITICAL FIX)
        # Always prevent penetration so robot bounces on ground, even when tipped
        pos, ori = p.getBasePositionAndOrientation(self.body)
        euler = p.getEulerFromQuaternion(ori)
        tilt_angle = np.sqrt(euler[0]**2 + euler[1]**2)
        max_tilt = np.pi / 6  # 30 degrees
        is_tipped = tilt_angle >= max_tilt
        
        # Check if spring tips went below ground, not just COM
        z_after, vz_after = self._get_z_vz()
        lowest_z_after = self._get_lowest_point_z()
        # When tipped, use pure physics - let PyBullet handle everything naturally
        if self.has_tipped or is_tipped:
            # Pure physics mode: no manual intervention at all
            # Let PyBullet's collision system handle ground contact naturally
            # Skip 1D clamping entirely - let it move freely in 3D space
            pass
        else:
            # Normal operation: prevent ground penetration and clamp to 1D
            if lowest_z_after < 0.0:
                # Spring tips went below ground - reset so tips are at ground level
                if self.has_joint:
                    # Joint mode: reset joint position
                    p.resetJointState(self.body, self.motion_link_idx, targetValue=0.0, targetVelocity=max(0.0, vz_after))
                else:
                    # Clamp mode: reset base position so lowest point is just above ground
                    current_base_z = pos[2]
                    offset_from_base_to_lowest = current_base_z - lowest_z_after
                    target_base_z = 0.001 + offset_from_base_to_lowest
                    
                    # Preserve horizontal position and orientation if testing disturbances
                    if self.disturbance_applied and (self.disturbance_time is None or t >= self.disturbance_time):
                        p.resetBasePositionAndOrientation(self.body, [pos[0], pos[1], target_base_z], ori)
                    else:
                        p.resetBasePositionAndOrientation(self.body, [0, 0, target_base_z], self.base_orientation)
                    # If moving down, reverse to upward velocity to bounce back smoothly
                    lin, ang = p.getBaseVelocity(self.body)
                    if lin[2] < 0:
                        vz_corrected = min(abs(lin[2]) * 0.5, 2.0)  # Bounce back, cap at 2 m/s
                    else:
                        vz_corrected = lin[2]  # Keep upward velocity
                    # Preserve horizontal velocity and angular velocity if testing disturbances
                    if self.disturbance_applied and (self.disturbance_time is None or t >= self.disturbance_time):
                        p.resetBaseVelocity(self.body, [lin[0], lin[1], vz_corrected], ang)
                    else:
                        p.resetBaseVelocity(self.body, [0, 0, vz_corrected], [0, 0, 0])
            # Clamp to 1D motion (normal operation)
            allow_drift = self.disturbance_applied and (self.disturbance_time is None or t < self.disturbance_time + self.disturbance_duration + 2.0)
            self._clamp_to_1d(allow_drift=allow_drift)
        # Get horizontal position for disturbance tracking
        pos, _ = p.getBasePositionAndOrientation(self.body)
        horizontal_drift = np.sqrt(pos[0]**2 + pos[1]**2)
        
        self.log_data.append({"t":t,"z":z,"vz":vz,"state":1 if self.state==STANCE else 0,
                              "u_h_applied":u_h_applied,"F_contact":F_contact,"h_target":self.h_target,
                              "drift":horizontal_drift})
        self.last_vz=vz

    # ---------- I/O ----------
    def close(self):
        try: p.disconnect(self.client)
        except p.error: pass

    def save_csv(self, base):
        base=Path(base); base.parent.mkdir(parents=True, exist_ok=True)
        if self.log_data:
            with open(base.with_suffix(".csv"),"w",newline="") as f:
                w=csv.DictWriter(f,fieldnames=self.log_data[0].keys()); w.writeheader(); w.writerows(self.log_data)
        if self.apex_history:
            with open(f"{base}_apex.csv","w",newline="") as f:
                w=csv.writer(f); w.writerow(["t_apex","apex_h","F_peak_stance","h_target"]); w.writerows(self.apex_history)

    def quick_plots(self, show=True, save=False, path="logs/plot.png"):
        if not self.log_data: return
        d={k:[r[k] for r in self.log_data] for k in self.log_data[0]}; t=d["t"]
        # Check if drift data exists (for disturbance tests)
        has_drift = "drift" in d
        num_plots = 6 if has_drift else 5
        fig,axs=plt.subplots(num_plots,1,figsize=(10,14+2 if has_drift else 14),sharex=True)
        axs[0].plot(t,d["z"]); axs[0].plot(t,d["h_target"],"r--"); axs[0].set_ylabel("z [m]"); axs[0].grid(True)
        if self.apex_history:
            ap_t=[a[0] for a in self.apex_history]; ap_h=[a[1] for a in self.apex_history]
            axs[0].plot(ap_t,ap_h,"go",ms=4)
        axs[1].plot(t,d["vz"]); axs[1].set_ylabel("vz [m/s]"); axs[1].grid(True)
        axs[2].plot(t,d["state"],"m"); axs[2].set_ylabel("state"); axs[2].set_ylim(-0.1,1.1); axs[2].grid(True)
        axs[3].plot(t,d["u_h_applied"],"c"); axs[3].set_ylabel("u_h [m]"); axs[3].grid(True)
        axs[4].plot(t,d["F_contact"],"orange"); axs[4].set_ylabel("F [N]"); axs[4].grid(True)
        if has_drift:
            axs[5].plot(t,d["drift"],"r"); axs[5].set_ylabel("Horizontal drift [m]"); axs[5].set_xlabel("t [s]"); axs[5].grid(True)
            if hasattr(self, 'recovery_time') and self.recovery_time is not None and hasattr(self, 'disturbance_time') and self.disturbance_time is not None:
                axs[5].axvline(x=self.disturbance_time + self.recovery_time, color='g', linestyle='--', label='Recovery')
            if hasattr(self, 'disturbance_time') and self.disturbance_time is not None:
                axs[5].axvline(x=self.disturbance_time, color='orange', linestyle='--', label='Disturbance')
                if hasattr(self, 'disturbance_duration'):
                    axs[5].axvline(x=self.disturbance_time + self.disturbance_duration, color='orange', linestyle='--')
            axs[5].legend()
        else:
            axs[4].set_xlabel("t [s]")
        fig.suptitle("Hopper 1-D Summary"); plt.tight_layout()
        if save: Path(path).parent.mkdir(parents=True, exist_ok=True); plt.savefig(path,dpi=160)
        # only show if GUI mode; backend is Agg so this is harmless if headless
        if show and self.sim_mode==p.GUI: plt.show()
        plt.close(fig)
