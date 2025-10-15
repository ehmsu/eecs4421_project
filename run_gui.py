import time
from hopper1d import HopperSim

if __name__ == "__main__":
    # --- Configuration based on project goals and initial tuning ---
    cfg = {
        "gui": True,
        "dt": 1.0/240.0,
        "sim_steps": 240*12,          # ~12 s simulated
        # Physical Parameters
        "m": 2.0,                     # Mass [kg]
        "k": 500.0,                   # Spring stiffness [N/m] (Corrected)
        "b": 8.0,                     # Damping [N-s/m]
        "g": 9.81,                    # Gravity [m/s^2]
        "l0": 0.35,                   # Rest length [m]
        
        # Controller Parameters (Raibert-style height regulator)
        "h_star": 0.55,               # Target apex height [m]
        "kh": 220.0,                  # Height loop gain [scaled] (Corrected)
        "pulse_width": 0.08,          # Control pulse width [fraction of stance]
        "pulse_phase": 0.05,          #0.30 Control pulse start phase [fraction of stance]
        "omega": 0.15,                # Control scaling factor (delta l0 modulation)

        # Energy and Guards
        "eta": 0.0,                   # Energy-recovery fraction (reserved/unused in v1)
        "F_max": 12.0,                 # Peak force guard (in mg units, e.g., 12*m*g) (Restored)
        "seed": 1                     # Simulation seed
    }

    sim = HopperSim(cfg)
    sim.reset()
    
    # t0 = time.time()
    # # Run simulation loop
    # for _ in range(cfg["sim_steps"]):
    #     sim.step()
        
    # print(f"Sim time: {time.time() - t0:.2f}s; hops logged: {len(sim.logs['apex_h'])}")
    # sim.quick_plots(show=True, save=False)

    # In run_gui.py...

    t0 = time.time()
    # Run simulation loop
    for _ in range(cfg["sim_steps"]):
        sim.step()
        time.sleep(cfg["dt"]) # <--- ADD THIS LINE to slow down for visualization
        
    print(f"Sim time: {time.time() - t0:.2f}s; hops logged: {len(sim.logs['apex_h'])}")
    sim.quick_plots(show=True, save=False)