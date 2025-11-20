#!/usr/bin/env python3
"""GUI test for disturbance rejection - visual demonstration."""

import argparse, os, time, logging, pybullet as p
from pathlib import Path
from hopper1d import Hopper1D

LOG_DIR = Path("logs/disturbances")
SIM_DURATION_S = 12.0  # Longer to see multiple hops
DISTURBANCE_TIME_S = 2.0  # Apply after 2 seconds (should have hopped by then)
DISTURBANCE_DURATION_S = 0.1

def main():
    parser = argparse.ArgumentParser(description="Test robot disturbance rejection with GUI")
    parser.add_argument("--force", type=float, default=2.0, help="Disturbance force magnitude (N)")
    parser.add_argument("--direction", choices=['x', 'y'], default='x', help="Direction of push")
    parser.add_argument("--time", type=float, default=3.0, help="Time to apply disturbance (s)")
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    logging.info(f"Testing disturbance: {args.force:.2f}N in {args.direction} direction at t={args.time:.1f}s")
    
    # Always use GUI for this script
    sim = Hopper1D(mode=p.GUI)
    
    # Set control parameters to match normal operation (from run_gui.py defaults)
    sim.K_h = float(os.getenv("HOP_K", "3500"))
    sim.B_h = float(os.getenv("HOP_B", "120"))
    sim.k_raibert = float(os.getenv("HOP_KP", "0.30"))
    sim.u_h_max = float(os.getenv("HOP_UH", "0.08"))
    sim.L_rest = float(os.getenv("HOP_LREST", "0.30"))
    sim.T_stance_estimate = float(os.getenv("HOP_TST", "0.12"))
    sim.pulse_phase_start = float(os.getenv("HOP_PSTART", "0.12"))
    sim.pulse_width = float(os.getenv("HOP_PWIDTH", "0.20"))
    sim.F_max_scalar = float(os.getenv("HOP_FMAX", "20.0"))
    sim.F_peak_guard = sim.F_max_scalar * sim.mass * 9.81
    
    # Start robot lower so it actually lands and hops
    z0 = float(os.getenv("HOP_Z0", "0.40"))  # Lower starting height
    sim._reset_height(z0)
    sim.set_target_height(0.5)
    
    # Don't schedule disturbance yet - we'll apply it during actual hopping
    total_steps = int(SIM_DURATION_S / sim.dt)
    disturbance_step = int(args.time / sim.dt)
    
    # Track if we've had at least one hop before applying disturbance
    hops_before_disturbance = 0
    disturbance_applied = False
    
    print(f"\n=== Disturbance Test (GUI) ===")
    print(f"Force: {args.force:.2f}N in {args.direction} direction")
    print(f"Will apply during hopping (after robot has landed at least once)")
    print(f"Watch the robot continue hopping after being pushed!\n")
    
    try:
        for i in range(total_steps):
            t = i * sim.dt
            sim.step(t)
            
            # Count hops before applying disturbance
            if not disturbance_applied:
                hops_before_disturbance = len(sim.apex_history)
                # Apply disturbance after robot has hopped at least once and is in flight or stance
                if hops_before_disturbance >= 1 and i >= disturbance_step:
                    # Apply during a hop - check if robot is in stance or just after landing
                    if sim.state == 1 or (sim.state == 0 and sim.has_landed_once):  # STANCE or FLIGHT after landing
                        if args.direction == 'x':
                            sim.apply_disturbance(force_x=args.force, force_y=0.0, duration=DISTURBANCE_DURATION_S)
                        else:
                            sim.apply_disturbance(force_x=0.0, force_y=args.force, duration=DISTURBANCE_DURATION_S)
                        disturbance_applied = True
                        print(f">>> DISTURBANCE APPLIED at t={t:.2f}s (during hop #{hops_before_disturbance+1}) <<<")
            
            time.sleep(sim.dt)  # Real-time visualization
    except KeyboardInterrupt:
        logging.info("Interrupted by user.")
    finally:
        sim.close()
        
        # Save results
        test_name = f"disturb_{args.force:.1f}N_{args.direction}_gui"
        log_base = LOG_DIR / test_name
        sim.save_csv(log_base)
        
        # Generate summary
        print(f"\n=== Test Results ===")
        print(f"Maximum drift: {sim.max_drift:.4f}m")
        print(f"Recovered: {sim.recovery_time is not None}")
        if sim.recovery_time:
            print(f"Recovery time: {sim.recovery_time:.3f}s")
        print(f"Number of hops: {len(sim.apex_history)}")
        if sim.apex_history:
            print(f"Final apex height: {sim.apex_history[-1][1]:.3f}m")
        
        plot_path = LOG_DIR / f"{test_name}_plots.png"
        sim.quick_plots(show=False, save=True, path=str(plot_path))
        print(f"\nPlot saved to: {plot_path}")

if __name__ == "__main__":
    main()

