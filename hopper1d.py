# hopper1d.py
# Mr. Springs — PyBullet v1 (strict 1-D vertical hopper)
# 1-link MultiBody with a PRISMATIC joint along +Z.
# Robust STANCE/FLIGHT with contact-based switching.

import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import math, random

# Hybrid States
FLIGHT, STANCE, FAULT = 0, 1, 2

class HopperSim:
    def __init__(self, cfg):
        self.c = cfg
        # Seed for reproducibility
        random.seed(cfg.get("seed", 1)); np.random.seed(cfg.get("seed", 1))
        # Connect to PyBullet (GUI or DIRECT)
        self.client = p.connect(p.GUI if cfg.get("gui", True) else p.DIRECT)
        
        # Setup world
        p.setAdditionalSearchPath(pybullet_data.getDataPath()); p.resetSimulation()
        p.setGravity(0, 0, -cfg["g"]); p.setTimeStep(cfg["dt"])
        self._build_world(); self._build_body()
        self._reset_state(); self._init_logs()

    def reset(self):
        # Reset joint state to a defined starting position/velocity
        start_z = self.c["l0"] + 0.10
        p.resetJointState(self.body, 0, targetValue=start_z, targetVelocity=-1.0)
        self._reset_state()

    def step(self):
        dt = self.c["dt"]
        z, vz = self._get_z_vz()
        contacting = self._contacting()

        # --- FLIGHT ---
        if self.state == FLIGHT:
            # Append to flight window for peak detection
            self.flight_z.append(z)
            if len(self.flight_z) > 3: self.flight_z.pop(0)

            # (1) vz zero-crossing detector for apex
            if (not self.apex_found) and (self.prev_vz > 0.0) and (vz <= 0.0):
                self.logs["apex_h"].append(z); self.apex_found = True

            # (2) 3-sample local-maximum detector on z (robust to step timing)
            if (not self.apex_found) and len(self.flight_z) == 3:
                z0, z1, z2 = self.flight_z
                if (z1 >= z0) and (z1 >= z2) and (z1 - min(z0, z2) > 1e-4):
                    self.logs["apex_h"].append(z1); self.apex_found = True

            # Event: Enter stance on any contact
            if contacting:
                self.state = STANCE; self.stance_t = 0.0; self.apex_found = False
                self.flight_z.clear()

            self._apply_force(0.0) # No active external force in flight

        # --- STANCE ---
        elif self.state == STANCE:
            self.stance_t += dt
            # Stance phase fraction (capped at 1.0 based on nominal 0.12s)
            frac = min(1.0, self.stance_t / 0.12)
            
            # Predictive control: h_pred is used to calculate the control pulse
            h_pred = self._predict_next_apex(z, vz)
            uh = (self._height_controller(h_pred) if self._pulse_active(frac) else 0.0) * self.c["omega"]
            
            # Spring-Damper Force (Primary contact force)
            Fsd = self._spring_damper_force(z, vz)
            
            # Apply total force: Spring-Damper + Control Pulse
            self._apply_force(Fsd + uh)

            # Event: Liftoff (leave contact AND moving up)
            if (not contacting) and (vz > 0.0):
                self.state = FLIGHT; self.stance_t = 0.0
                self.flight_z = []      # start a fresh flight window
                self.apex_found = False # look for a new apex

        # Guards
        if self._guard_fault(): self.state = FAULT

        # Advance sim + bookkeeping
        p.stepSimulation(); self.t += dt; self.prev_vz = vz
        self.logs["t"].append(self.t); self.logs["z"].append(z)
        self.logs["vz"].append(vz); self.logs["state"].append(self.state)
        
        # Log intended control signal (uh before scaling/pulsing) for analysis
        intended_uh = 0.0
        if self.state == STANCE:
            # Re-calculate the intended control value without the pulse/omega filter for logging
            intended_uh = self.c["kh"]*(self.c["h_star"] - self._predict_next_apex(z, vz))
        self.logs["uh"].append(intended_uh); self.logs["Fpeak"].append(self.peakF)

    # ---------- Build ----------
    def _build_world(self): 
        # Load the ground plane
        self.plane = p.loadURDF("plane.urdf")
        # Adjust friction/restitution of the plane if needed (default is good for plane.urdf)

    def _build_body(self):
        m = self.c["m"]; half = [0.08,0.08,0.05]
        # Create collision and visual shapes (a simple box for the body)
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=[0.9,0.4,0.2,1])
        
        # Create the MultiBody with one prismatic joint along Z
        self.body = p.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=-1,
            basePosition=[0,0,0], baseOrientation=[0,0,0,1],
            linkMasses=[m], linkCollisionShapeIndices=[col], linkVisualShapeIndices=[vis],
            linkPositions=[[0,0,0]], linkOrientations=[[0,0,0,1]],
            linkInertialFramePositions=[[0,0,0]], linkInertialFrameOrientations=[[0,0,0,1]],
            linkParentIndices=[0], linkJointTypes=[p.JOINT_PRISMATIC], linkJointAxis=[[0,0,1]]
        )
        # Free the slider: disable default motor control
        p.setJointMotorControl2(self.body, 0, controlMode=p.VELOCITY_CONTROL, force=0.0)
        
        # Tune dynamics for the link (friction, damping, restitution)
        p.changeDynamics(self.body, 0, linearDamping=0.0, angularDamping=1.0,
                         restitution=0.0, lateralFriction=1.0)

    # ---------- State & logs ----------
    def _reset_state(self):
        self.state = FLIGHT; self.apex_found = False; self.prev_vz = 0.0
        self.stance_t = 0.0; self.peakF = 0.0; self.t = 0.0
        self.flight_z = []  # rolling window of last 3 z samples during FLIGHT

    def _init_logs(self):
        self.logs = dict(t=[], z=[], vz=[], state=[], uh=[], apex_h=[], Fpeak=[])

    # ---------- Sensing / contact ----------
    def _get_z_vz(self):
        # Get position/velocity of the single link (index 0)
        ls = p.getLinkState(self.body, 0, computeLinkVelocity=1)
        pos = ls[0]; lin = ls[6]; return pos[2], lin[2]

    def _contacting(self):
        # Check for contact points between the hopper link and the ground plane
        cps = p.getContactPoints(bodyA=self.body, bodyB=self.plane, linkIndexA=0, linkIndexB=-1)
        return len(cps) > 0

    # ---------- Forces / control ----------
    def _apply_force(self, fz):
        # Apply external force (Fz) to the link in the Z direction
        p.applyExternalForce(self.body, 0, [0,0,fz], [0,0,0], p.LINK_FRAME)
        # Log the peak force experienced during the current step
        self.peakF = max(self.peakF, abs(fz))

    def _spring_damper_force(self, z, vz):
        # Calculate spring-damper force F_sd = k*(l0 - z) - b*vz
        k,b,l0 = self.c["k"], self.c["b"], self.c["l0"]; return k*(l0 - z) - b*vz

    def _pulse_active(self, frac):
        # Check if the current stance phase fraction is within the pulse window
        ph, w = self.c["pulse_phase"], self.c["pulse_width"]; return (ph <= frac <= ph+w)

    def _height_controller(self, h_pred):
        # Calculate the control effort u_h = k_h * (h* - h_pred)
        return self.c["kh"] * (self.c["h_star"] - h_pred)

    def _predict_next_apex(self, z, vz):
        # Predict apex height based on current height and vertical velocity (h_pred = z + vz^2 / 2g)
        g = self.c["g"]; return z if vz <= 0.0 else z + (vz*vz)/(2*g)

    def _guard_fault(self):
        # Check for fault: Peak force exceeds F_max*m*g or is NaN
        FmaxN = self.c["F_max"]*self.c["m"]*self.c["g"]; return (self.peakF > FmaxN) or math.isnan(self.peakF)

    # ---------- Viz ----------
    def quick_plots(self, show=True, save=False, path="plots.png"):
        t = np.array(self.logs["t"]); z = np.array(self.logs["z"])
        vz = np.array(self.logs["vz"]); uh = np.array(self.logs["uh"])
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(3,1, figsize=(8,7), constrained_layout=True)
        # Plot Z (COM Height)
        axs[0].plot(t,z); axs[0].axhline(self.c["h_star"], linestyle="--", label="h*"); 
        axs[0].legend(); axs[0].set_ylabel("z [m]")
        # Plot Vz (Vertical Velocity)
        axs[1].plot(t,vz); axs[1].set_ylabel("vz [m/s]")
        # Plot Uh (Intended Control Signal)
        axs[2].plot(t,uh); axs[2].set_ylabel("u_h (intended)"); axs[2].set_xlabel("t [s]")
        
        for ax in axs: ax.grid(True, alpha=0.3)
        if save: plt.savefig(path, dpi=160)
        if show: plt.show(); plt.close(fig)