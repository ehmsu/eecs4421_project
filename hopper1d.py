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
        self.plane = p.loadURDF("plane.urdf")
        # Reduce contact stiffness to allow spring-damper forces to overcome contact constraints
        p.changeDynamics(self.plane, -1, lateralFriction=0.5, restitution=0.0, contactStiffness=1e6, contactDamping=100)

        self.body, self.motion_mode, self.motion_link_idx, self.has_joint = self._load_robot()
        self.mass = self._infer_mass(self.body, self.motion_link_idx)
        # Store the desired orientation (springs down, wheels up)
        # Wheels are on Y-axis (0, 0.15, 0) and (0, -0.15, 0)
        # Rotate -90° around X-axis to put wheels at top (Z+) and springs pointing down (Z-)
        self.base_orientation = p.getQuaternionFromEuler([-1.5708, 0, 0])
        if self.has_joint:
            # Don't set any joint control - let the joint move freely under gravity
            # We'll apply forces via TORQUE_CONTROL in _apply_force when needed
            # Reduce joint damping to allow free motion
            p.changeDynamics(self.body, self.motion_link_idx, 
                           linearDamping=0.0, angularDamping=0.0,
                           jointDamping=0.0)
        # Reduce contact stiffness on robot links to allow spring forces to work
        # Keep collisions enabled so model stays together visually
        num_joints = p.getNumJoints(self.body)
        for i in range(-1, num_joints):
            p.changeDynamics(self.body, i, 
                           lateralFriction=0.5, restitution=0.0,
                           contactStiffness=1e6, contactDamping=100)
        # Disable collision only between base link and ground plane - we handle contact via spring-damper model
        # All internal collisions between robot parts remain enabled by default (fixed joints keep them together)
        p.setCollisionFilterPair(self.body, self.plane, -1, -1, enableCollision=0)
        # Ensure all fixed joints are properly constrained to keep parts together (wheels, springs, etc.)
        # Fixed joints should automatically keep parts together, but we ensure they're stable
        for i in range(num_joints):
            ji = p.getJointInfo(self.body, i)
            if ji[2] == p.JOINT_FIXED:  # Fixed joint
                # Fixed joints don't need motor control, but ensure they're stable
                # The joint itself keeps parts connected
                pass

        # Model/Control (per feedback)
        # Increased spring stiffness and control gains for more visible hopping
        self.K_h=4000.0; self.B_h=30.0; self.L_rest=0.30
        self.h_target=0.80; self.k_raibert=0.30; self.u_h_max=0.08
        self.T_stance_estimate=0.15; self.pulse_phase_start=0.10; self.pulse_width=0.30
        # Increase F_max_scalar significantly to allow sufficient force for hopping
        # At z=-0.1, L=0.3, force = K*(L-z) = 4000*0.4 = 1600N needed
        self.F_max_scalar=500.0; self.F_peak_guard=self.F_max_scalar*self.mass*9.81
        self.ENERGY_RECOVERY=False

        self.state=FLIGHT; self.last_vz=0.0; self.stance_t0=0.0
        self.current_u_h=0.0; self.F_peak_stance=0.0
        self.log_data=[]; self.apex_history=[]
        self.h_start=0.80; self._reset_height(self.h_start)

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
            # Rotate -90° around X-axis to put wheels (on Y-axis) at top and springs pointing down
            base_orientation = p.getQuaternionFromEuler([-1.5708, 0, 0])
            body=p.loadURDF(urdf_path,useFixedBase=use_fixed,baseOrientation=base_orientation,flags=p.URDF_USE_INERTIA_FROM_FILE)
            logging.info(f"Loaded URDF: {urdf_path} (useFixedBase=False, rotated 90° around X-axis: wheels at top, springs down, clamp mode)")
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
        
        # No world link - check for Z prismatic joint
        z_joint=None
        for j in range(p.getNumJoints(body)):
            ji=p.getJointInfo(body,j)
            if ji[2]==p.JOINT_PRISMATIC and abs(ji[13][2])>0.9:
                z_joint=j; break
        if z_joint is not None:
            return body,"joint",z_joint,True
        # else clamp mode (choose heaviest link/base)
        anchor=self._pick_heaviest_link(body)
        logging.info(f"No Z-prismatic joint; using clamp on link {anchor}.")
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

    def _clamp_to_1d(self):
        if self.motion_mode!="clamp": return
        if self.motion_link_idx==-1:
            pos,ori=p.getBasePositionAndOrientation(self.body); lin,ang=p.getBaseVelocity(self.body)
            # Only clamp if position or orientation has drifted from desired state
            # Check if position is off X/Y axis or orientation has changed
            if abs(pos[0]) > 1e-6 or abs(pos[1]) > 1e-6:
                p.resetBasePositionAndOrientation(self.body,[0,0,pos[2]],self.base_orientation)
            # Check if velocity has X/Y components or angular velocity
            if abs(lin[0]) > 1e-6 or abs(lin[1]) > 1e-6 or abs(ang[0]) > 1e-6 or abs(ang[1]) > 1e-6 or abs(ang[2]) > 1e-6:
                p.resetBaseVelocity(self.body,[0,0,lin[2]],[0,0,0])
        else:
            pos,ori=p.getBasePositionAndOrientation(self.body)
            # Only clamp if position has drifted
            if abs(pos[0]) > 1e-6 or abs(pos[1]) > 1e-6:
                p.resetBasePositionAndOrientation(self.body,[0,0,pos[2]],self.base_orientation)
            lin,ang=p.getBaseVelocity(self.body)
            # Preserve Z velocity, clamp X/Y and angular
            if abs(lin[0]) > 1e-6 or abs(lin[1]) > 1e-6 or abs(ang[0]) > 1e-6 or abs(ang[1]) > 1e-6 or abs(ang[2]) > 1e-6:
                p.resetBaseVelocity(self.body,[0,0,lin[2]],[0,0,0])

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
        # z is the height above ground (should be >= 0, clamped in step())
        # L is the rest length of the spring (l0)
        # Spring is active when z < L (compressed)
        if z>=L: return 0.0
        # Calculate spring force: F_spring = K*(L-z) when compressed
        # Damping force: F_damp = -B*vz (opposes velocity)
        # When vz < 0 (moving down), -B*vz > 0 (pushes up) ✓
        # When vz > 0 (moving up), -B*vz < 0 (pulls down) ✓
        F = self.K_h*(L-z) - self.B_h*vz
        # Force must be non-negative (can't pull down through ground)
        F = max(0.0, F)
        return min(F, self.F_peak_guard)

    def step(self,t):
        z,vz=self._get_z_vz()
        # Clamp z to prevent negative values (ground is at z=0)
        z = max(0.0, z)
        u_h_applied=0.0; F_contact=0.0

        if self.state==FLIGHT:
            # Detect apex (velocity crosses zero from positive to negative)
            if getattr(self,'last_vz',0.0)>0 and vz<=0:
                self.apex_history.append((t,z,getattr(self,'F_peak_stance',0.0),self.h_target))
                self.F_peak_stance=0.0
            # Per proposal: "touchdown when z = l0 with z_dot < 0"
            if z <= self.L_rest and vz < 0:
                self.state=STANCE; self.stance_t0=t; self.current_u_h=self._calc_u_h()
        else:
            # STANCE phase: apply spring-damper force
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
            # Per proposal: "liftoff when spring re-extends to z = l0 with z_dot > 0"
            if z >= self.L_rest and vz > 0:
                self.state=FLIGHT

        p.stepSimulation()
        # After physics step, clamp z to prevent penetration
        z_after, vz_after = self._get_z_vz()
        if z_after < 0.0 and self.has_joint:
            # Reset joint to z=0 if it went negative
            p.resetJointState(self.body, self.motion_link_idx, targetValue=0.0, targetVelocity=max(0.0, vz_after))
        # Clamp to 1D motion (preserves orientation)
        self._clamp_to_1d()
        self.log_data.append({"t":t,"z":z,"vz":vz,"state":1 if self.state==STANCE else 0,
                              "u_h_applied":u_h_applied,"F_contact":F_contact,"h_target":self.h_target})
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
        fig,axs=plt.subplots(5,1,figsize=(10,14),sharex=True)
        axs[0].plot(t,d["z"]); axs[0].plot(t,d["h_target"],"r--"); axs[0].set_ylabel("z [m]"); axs[0].grid(True)
        if self.apex_history:
            ap_t=[a[0] for a in self.apex_history]; ap_h=[a[1] for a in self.apex_history]
            axs[0].plot(ap_t,ap_h,"go",ms=4)
        axs[1].plot(t,d["vz"]); axs[1].set_ylabel("vz [m/s]"); axs[1].grid(True)
        axs[2].plot(t,d["state"],"m"); axs[2].set_ylabel("state"); axs[2].set_ylim(-0.1,1.1); axs[2].grid(True)
        axs[3].plot(t,d["u_h_applied"],"c"); axs[3].set_ylabel("u_h [m]"); axs[3].grid(True)
        axs[4].plot(t,d["F_contact"],"orange"); axs[4].set_ylabel("F [N]"); axs[4].set_xlabel("t [s]"); axs[4].grid(True)
        fig.suptitle("Hopper 1-D Summary"); plt.tight_layout()
        if save: Path(path).parent.mkdir(parents=True, exist_ok=True); plt.savefig(path,dpi=160)
        # only show if GUI mode; backend is Agg so this is harmless if headless
        if show and self.sim_mode==p.GUI: plt.show()
        plt.close(fig)
