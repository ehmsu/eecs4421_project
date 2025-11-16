import argparse, os, time, logging, pybullet as p
from pathlib import Path
from hopper1d import Hopper1D

LOG_DIR = Path("logs")
SIM_DURATION_S = float(os.getenv("SIM_T", "12.0"))
STEP_CHANGE_TIME_S = float(os.getenv("STEP_T", "5.0"))
H_TARGET_INITIAL = float(os.getenv("H0", "0.5"))
H_TARGET_FINAL   = float(os.getenv("H1", "0.7"))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="Run without GUI (DIRECT)")
    args=parser.parse_args()

    headless = args.headless or os.environ.get("MR_SPRINGS_HEADLESS")=="1"
    mode = p.DIRECT if headless else p.GUI

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    logging.info("Starting 1-D Hopper Simulation... (headless=%s)" % headless)

    sim = Hopper1D(mode=mode)

    # --- Read tunables from environment (fallbacks = reasonable conservative defaults) ---
    sim.K_h = float(os.getenv("HOP_K", "3500"))          # N/m
    sim.B_h = float(os.getenv("HOP_B", "120"))           # N*s/m
    sim.k_raibert = float(os.getenv("HOP_KP", "0.30"))   # proportional on apex error
    sim.u_h_max = float(os.getenv("HOP_UH", "0.08"))     # m
    sim.L_rest = float(os.getenv("HOP_LREST", "0.30"))   # m
    sim.T_stance_estimate = float(os.getenv("HOP_TST", "0.12"))  # s
    sim.pulse_phase_start = float(os.getenv("HOP_PSTART", "0.12")) # fraction of T_stance
    sim.pulse_width = float(os.getenv("HOP_PWIDTH", "0.20"))       # fraction of T_stance
    sim.F_max_scalar = float(os.getenv("HOP_FMAX", "20.0"))        # multiple of m*g
    sim.F_peak_guard = sim.F_max_scalar * sim.mass * 9.81

    # optional start height
    z0 = float(os.getenv("HOP_Z0", "0.60"))
    sim._reset_height(z0)

    sim.set_target_height(H_TARGET_INITIAL)

    total_steps = int(SIM_DURATION_S / sim.dt)
    step_change_step = int(STEP_CHANGE_TIME_S / sim.dt)

    try:
        for i in range(total_steps):
            t = i * sim.dt
            if i == step_change_step:
                sim.set_target_height(H_TARGET_FINAL)
            sim.step(t)
            if not headless:
                time.sleep(sim.dt)
    except KeyboardInterrupt:
        logging.info("Interrupted by user.")
    finally:
        sim.close()
        log_base = LOG_DIR / "run1"
        sim.save_csv(log_base)
        print("\n--- Simulation Summary ---")
        print(f"Hops logged: {len(sim.apex_history)}")
        if sim.apex_history:
            print(f"Final apex height: {sim.apex_history[-1][1]:.3f} m (Target: {H_TARGET_FINAL:.2f} m)")
        plot_path = LOG_DIR / "run1_plots.png"
        sim.quick_plots(show=not headless, save=True, path=str(plot_path))

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    main()
