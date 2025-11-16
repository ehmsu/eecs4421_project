# EECS 4421: 1-D Vertical Jumper ("Mr. Springs")

This project simulates a 1-D vertical hopper robot using PyBullet. The robot is based on a spring-powered jumping mechanism with wheels, rods, and springs. The goal is to achieve stable vertical hopping and regulate the apex height to a target value ($h^\star$) using a Raibert-style controller.

## Project Overview

**Robot Model:** The simulation uses a jumping robot URDF model with:
- Main body (`base_link`)
- Two wheels attached horizontally (Y-axis)
- Two spring-loaded rods with springs at the bottom
- Various mechanical components (gears, motors, frames, etc.)

**Orientation:** The robot is oriented with wheels at the top (Z+) and springs pointing downward (Z-), rotated -90° around the X-axis from the original CAD model.

## V1 Project Scope (1-Dimensional)

**Addressing Professor Feedback:** This V1 simulation is **strictly 1-dimensional** (vertical motion along the Z-axis).
* The robot is constrained to vertical motion only using programmatic clamping.
* Wheels are **passive and not used for locomotion** or balance in this V1. They only serve to visually align the robot in a hypothetical "ring world."
* Future V2 work might explore 2-D planar motion, but that is explicitly out of scope for this version.

## Core Physics & Control

* **Engine:** PyBullet
* **Physics:** Hybrid discrete-state simulation (STANCE vs. FLIGHT).
* **Motion Constraint:** The robot's base link is programmatically clamped to 1D motion (X=0, Y=0, no rotation). This is done via `_clamp_to_1d()` which resets position/velocity drift each timestep.
* **Landing Model:** During STANCE, contact is modeled as a linear spring-damper ($F = K_h(L_{rest} - z) - B_h v_z$) applied at the ground ($z=0$, relative to the spring's rest length $L_{rest}=0.30$ m).
* **Contact Handling:** Collision between the robot base and ground plane is disabled. Contact forces are handled entirely by the custom spring-damper model.
* **Control:** A Raibert-style apex height regulator. A small "pulse" ($u_h$) is applied during the stance phase to modulate the spring's rest length. This pulse is proportional to the error between the target apex ($h^\star$) and the predicted apex ($\hat{h}_{\text{apex}}$).
* **Energy Harvesting:** We are **not** currently modeling energy harvesting from the spring-damper system. A flag (`ENERGY_RECOVERY`) exists in the code, but it is set to `False`.

## How to Run

This repository is designed to be run on a Linux (Ubuntu) system with Python 3.10+.

1. **Set up the environment:**
   ```bash
   # Create virtual environment
   python3 -m venv .venv
   source .venv/bin/activate
   
   # Install dependencies
   pip install -r requirements.txt
   ```

2. **Run the main GUI simulation:**
   ```bash
   # Activate the environment (if not already active)
   source .venv/bin/activate
   
   # Run the simulation with GUI
   python3 run_gui.py
   ```
   
   This will:
   * Open a PyBullet window showing the robot.
   * Run the simulation for 12 seconds, which starts at $h^\star=0.5$ m and steps to $h^\star=0.7$ m at $t=5$ s.
   * Save detailed logs to `logs/run1.csv`.
   * Save apex data to `logs/run1_apex.csv`.
   * Show and save plots to `logs/run1_plots.png`.

3. **Run in headless mode (no GUI):**
   ```bash
   python3 run_gui.py --headless
   ```

## Using Your Own URDF Model

The simulation automatically searches for a `.urdf` file in the `assets/` directory.

1. **Current model:** The simulation uses `assets/jumping_robot_description_root/eecs4421_1d_hopper.urdf`, which is generated from xacro files in `assets/jumping_robot_description_root/urdf/`.

2. **To use your model:** 
   * Place your URDF file (and any meshes) in the `assets/` directory.
   * The code (`hopper1d.py`) will find the *first* `.urdf` file it encounters in `assets/**` and load it.
   * Alternatively, set the `MR_SPRINGS_URDF` environment variable to specify a custom path.

3. **URDF requirements:**
   * The URDF should have `base_link` as the root link (no "world" link).
   * The robot will be automatically rotated -90° around the X-axis (wheels up, springs down).
   * Motion is constrained to 1D via programmatic clamping (no prismatic joint required).

4. **Fallback:** If no URDF is found in the `assets/` directory, the simulation will generate a default fallback model (`assets/mr_springs.urdf`), which is a simple box.

## Environment Variables

You can customize the simulation behavior using environment variables:

```bash
# Override URDF path
export MR_SPRINGS_URDF=/path/to/your/robot.urdf

# Run in headless mode
export MR_SPRINGS_HEADLESS=1

# Simulation duration (seconds)
export SIM_T=12.0

# Step change time (seconds)
export STEP_T=5.0

# Initial and final target heights (meters)
export H0=0.5
export H1=0.7

# Control parameters
export HOP_K=4000          # Spring stiffness (N/m)
export HOP_B=30            # Damping (N*s/m)
export HOP_KP=0.30         # Proportional gain for apex control
export HOP_UH=0.08         # Max control input (m)
export HOP_LREST=0.30      # Spring rest length (m)
export HOP_TST=0.15        # Stance time estimate (s)
export HOP_PSTART=0.10     # Pulse phase start (fraction of stance)
export HOP_PWIDTH=0.20     # Pulse width (fraction of stance)
export HOP_FMAX=500.0      # Max force scalar (multiple of m*g)
export HOP_Z0=0.60         # Initial height (m)
```

## Project Structure

```
eecs4421_project/
├── hopper1d.py              # Main simulation class
├── run_gui.py               # GUI launcher script
├── requirements.txt         # Python dependencies
├── README.md               # This file
├── assets/                 # Robot models
│   └── jumping_robot_description_root/
│       ├── eecs4421_1d_hopper.urdf  # Generated URDF
│       ├── meshes/         # STL mesh files
│       └── urdf/           # Xacro source files
│           ├── eecs4421_1d_wrapper.xacro
│           ├── jumping_robot_urdf_backup_copy.xacro
│           ├── jumping_robot_urdf_backup_copy.trans
│           └── materials.xacro
└── logs/                   # Simulation logs and plots
    ├── run1.csv
    ├── run1_apex.csv
    └── run1_plots.png
```

## Implementation Details

### Motion Constraint (Clamp Mode)

The robot uses "clamp mode" where:
- The base link is free to move in PyBullet (`useFixedBase=False`).
- Each timestep, `_clamp_to_1d()` resets X/Y position to 0 and angular velocity to 0.
- Only Z-position and Z-velocity are preserved.
- This allows the robot to fall under gravity and hop vertically.

### Spring-Damper Model

During STANCE phase:
- Spring force: $F_{spring} = K_h (L_{rest} - z)$
- Damping force: $F_{damp} = -B_h v_z$
- Total force: $F = F_{spring} + F_{damp}$ (clamped to be non-negative)
- Applied to the base link in the +Z direction (upward)

### State Machine

- **FLIGHT:** Ballistic motion under gravity. Apex is detected when velocity crosses zero from positive to negative.
- **STANCE:** Contact with ground. Spring-damper force is applied. Raibert control pulse is injected during mid-stance.

### Control Law

The Raibert-style height controller:
- Predicts next apex: $\hat{h}_{\text{apex}} = h + \frac{v_z^2}{2g}$
- Computes error: $e = h^\star - \hat{h}_{\text{apex}}$
- Applies control: $u_h = k_{raibert} \cdot e$ (clamped to $[-u_{h,max}, u_{h,max}]$)
- Modulates spring rest length: $L_{eff} = L_{rest} + u_h$ during pulse window

## Notes

- The robot model currently only shows `base_link` in simulation because the `.trans` file is empty. Once the 3D modeling person exports the complete URDF from Fusion 360 with all parts defined, the full robot assembly will appear.
- The simulation uses a timestep of 1/240 seconds (240 Hz).
- Contact stiffness and damping are reduced to allow the custom spring-damper model to dominate contact forces.
