# EECS 4421: 1-D Vertical Jumper ("Mr. Springs")

This project simulates a 1-D vertical hopper, "Mr. Springs," using PyBullet. The goal is to achieve stable vertical hopping and regulate the apex height to a target value ($h^\star$) using a Raibert-style controller.

## V1 Project Scope (1-Dimensional)

**Addressing Professor Feedback:** This V1 simulation is **strictly 1-dimensional** (vertical motion along the Z-axis).
* Any 3D model (like a wheeled robot) is constrained to a prismatic Z-joint.
* Wheels, if present in the model, are **passive and not used for locomotion** or balance in this V1. They only serve to visually align the robot in a hypothetical "ring world."
* Future V2 work might explore 2-D planar motion, but that is explicitly out of scope for this version.

## Core Physics & Control

* **Engine:** PyBullet
* **Physics:** Hybrid discrete-state simulation (STANCE vs. FLIGHT).
* **Landing Model:** During STANCE, contact is modeled as a linear spring-damper ($F = k(\Delta z) + b(\Delta v_z)$) applied at the ground ($z=0$, relative to the spring's rest length).
* **Control:** A Raibert-style apex height regulator. A small "pulse" ($u_h$) is applied during the stance phase to modulate the spring's rest length. This pulse is proportional to the error between the target apex ($h^\star$) and the last measured apex ($h_{\text{apex}}$).
* **Energy Harvesting:** We are **not** currently modeling energy harvesting from the spring-damper system. A flag (`ENERGY_RECOVERY`) exists in the code, but it is set to `False`.

## How to Run

This repository is designed to be run on a Linux (Ubuntu) system with Python 3.10.

1.  **Set up the environment and install dependencies:**
    ```bash
    # This script creates a .venv, installs requirements, and runs a test
    bash setup_and_run.sh
    ```

2.  **Run the main GUI simulation:**
    (The setup script does this, but you can run it manually)
    ```bash
    # Activate the environment
    source .venv/bin/activate
    # Run the 12-second simulation with GUI
    python run_gui.py
    ```
    This will:
    * Open a PyBullet window.
    * Run the simulation, which starts at $h^\star=0.5$m and steps to $h^\star=0.7$m at $t=5$s.
    * Save detailed logs to `logs/run1.csv`.
    * Save apex data to `logs/run1_apex.csv`.
    * Show and save plots to `logs/run1_plots.png`.

## Using Your Own URDF Model

The simulation automatically searches for a `.urdf` file in the `assets/` directory (e.g., `assets/jumping_robot/my_robot.urdf`).

1.  **To use your model:** Copy your URDF package (including any meshes) into the `assets/` directory.
    * Example: `cp -r ~/Downloads/my_robot_urdf/ assets/my_robot`
2.  **How it works:** The code (`hopper1d.py`) will find the *first* `.urdf` file it encounters in `assets/**` and load it. It will then scan this model for the *first available* **prismatic joint** and use that joint for all 1-D motion.
3.  **Fallback:** If no URDF is found in the `assets/` directory, the simulation will load a default fallback model (`assets/mr_springs.urdf`), which is a simple red box on a prismatic joint.

## Evaluation Workflow

You can test the controller's performance against the functional requirements (FRs).

1.  **Evaluate FR-1 (Step Response):**
    After running `run_gui.py`, evaluate the resulting apex log file. The command expects the apex CSV path, the initial target height, and the final (step) target height.
    ```bash
    # Run this after run_gui.py
    python eval_metrics.py logs/run1_apex.csv 0.5 0.7
    ```
    This will print the Overshoot, Steady-State Error, and Settling Time (in hops) for the step change.

2.  **Run a Parameter Sweep:**
    To find optimal control parameters, you can run a sweep.
    ```bash
    python sweep.py
    ```
    This runs the simulation in headless mode (`p.DIRECT`) for a grid of parameters defined in the script. It saves a summary of metrics to `logs/sweeps/summary.csv` and prints the top 2 best-performing configurations.
