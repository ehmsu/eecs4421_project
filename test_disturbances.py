#!/usr/bin/env python3
"""Test script for disturbance rejection - applies horizontal pushes and measures recovery."""

import argparse, os, time, logging, pybullet as p
import numpy as np
from pathlib import Path
from hopper1d import Hopper1D

LOG_DIR = Path("logs/disturbances")
SIM_DURATION_S = 10.0  # Duration for disturbance tests
DISTURBANCE_TIME_S = 3.0  # When to apply disturbance (after robot has started hopping)
DISTURBANCE_DURATION_S = 0.1  # How long to apply force

def test_disturbance(force_magnitude, direction='x', headless=True, save_plots=True):
    """Test a single disturbance scenario.
    
    Args:
        force_magnitude: Force in Newtons
        direction: 'x' or 'y' for horizontal direction
        headless: Run without GUI
        save_plots: Save plots to file
    
    Returns:
        dict with test results
    """
    mode = p.DIRECT if headless else p.GUI
    
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    logging.info(f"Testing disturbance: {force_magnitude:.2f}N in {direction} direction")
    
    sim = Hopper1D(mode=mode)
    
    # Set reasonable control parameters
    sim.K_h = float(os.getenv("HOP_K", "4000"))
    sim.B_h = float(os.getenv("HOP_B", "45"))
    sim.k_raibert = float(os.getenv("HOP_KP", "0.30"))
    sim.u_h_max = float(os.getenv("HOP_UH", "0.08"))
    sim.L_rest = float(os.getenv("HOP_LREST", "0.30"))
    sim.T_stance_estimate = float(os.getenv("HOP_TST", "0.055"))
    sim.pulse_phase_start = float(os.getenv("HOP_PSTART", "0.25"))
    sim.pulse_width = float(os.getenv("HOP_PWIDTH", "0.50"))
    sim.F_max_scalar = float(os.getenv("HOP_FMAX", "500.0"))
    sim.F_peak_guard = sim.F_max_scalar * sim.mass * 9.81
    
    z0 = float(os.getenv("HOP_Z0", "0.60"))
    sim._reset_height(z0)
    sim.set_target_height(0.5)
    
    # Schedule disturbance
    if direction == 'x':
        sim.apply_disturbance(force_x=force_magnitude, force_y=0.0, duration=DISTURBANCE_DURATION_S)
    else:
        sim.apply_disturbance(force_x=0.0, force_y=force_magnitude, duration=DISTURBANCE_DURATION_S)
    
    total_steps = int(SIM_DURATION_S / sim.dt)
    disturbance_step = int(DISTURBANCE_TIME_S / sim.dt)
    
    try:
        for i in range(total_steps):
            t = i * sim.dt
            # Apply disturbance at specified time
            if i == disturbance_step:
                # Disturbance will be applied in step() method
                pass
            sim.step(t)
            if not headless:
                time.sleep(sim.dt)
    except KeyboardInterrupt:
        logging.info("Interrupted by user.")
    finally:
        sim.close()
        
        # Save results
        test_name = f"disturb_{force_magnitude:.1f}N_{direction}"
        log_base = LOG_DIR / test_name
        sim.save_csv(log_base)
        
        # Generate summary
        results = {
            'force_magnitude': force_magnitude,
            'direction': direction,
            'max_drift': sim.max_drift,
            'recovery_time': sim.recovery_time,
            'recovered': sim.recovery_time is not None,
            'num_hops': len(sim.apex_history),
            'final_apex': sim.apex_history[-1][1] if sim.apex_history else None
        }
        
        if save_plots:
            plot_path = LOG_DIR / f"{test_name}_plots.png"
            sim.quick_plots(show=not headless, save=True, path=str(plot_path))
        
        return results

def main():
    parser = argparse.ArgumentParser(description="Test robot disturbance rejection")
    parser.add_argument("--force", type=float, default=2.0, help="Disturbance force magnitude (N)")
    parser.add_argument("--direction", choices=['x', 'y'], default='x', help="Direction of push")
    parser.add_argument("--sweep", action="store_true", help="Sweep force values from 0.5N to 10N")
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    
    if args.sweep:
        # Test multiple force magnitudes
        forces = np.linspace(0.5, 10.0, 10)  # 0.5N to 10N in 10 steps
        results = []
        
        print("\n=== Disturbance Rejection Sweep ===")
        print(f"Testing forces: {forces}")
        print(f"Direction: {args.direction}\n")
        
        for force in forces:
            result = test_disturbance(force, args.direction, headless=args.headless)
            results.append(result)
            print(f"Force: {force:.2f}N | Max drift: {result['max_drift']:.4f}m | "
                  f"Recovered: {result['recovered']} | Recovery time: {result['recovery_time']:.3f}s" 
                  if result['recovery_time'] else f"Force: {force:.2f}N | Max drift: {result['max_drift']:.4f}m | Recovered: No")
        
        # Find maximum recoverable force
        recoverable_forces = [r for r in results if r['recovered']]
        if recoverable_forces:
            max_recoverable = max(recoverable_forces, key=lambda x: x['force_magnitude'])
            print(f"\n=== Summary ===")
            print(f"Maximum recoverable force: {max_recoverable['force_magnitude']:.2f}N")
            print(f"Maximum drift observed: {max([r['max_drift'] for r in results]):.4f}m")
        else:
            print("\n=== Summary ===")
            print("No forces were fully recovered from (within 0.01m threshold)")
            print(f"Maximum drift observed: {max([r['max_drift'] for r in results]):.4f}m")
        
        # Save summary CSV
        import csv
        summary_path = LOG_DIR / "disturbance_sweep_summary.csv"
        with open(summary_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=results[0].keys())
            writer.writeheader()
            writer.writerows(results)
        print(f"\nSummary saved to: {summary_path}")
    else:
        # Single test
        result = test_disturbance(args.force, args.direction, headless=args.headless)
        print("\n=== Disturbance Test Results ===")
        print(f"Force: {result['force_magnitude']:.2f}N in {result['direction']} direction")
        print(f"Maximum drift: {result['max_drift']:.4f}m")
        print(f"Recovered: {result['recovered']}")
        if result['recovery_time']:
            print(f"Recovery time: {result['recovery_time']:.3f}s")
        print(f"Number of hops: {result['num_hops']}")
        if result['final_apex']:
            print(f"Final apex height: {result['final_apex']:.3f}m")

if __name__ == "__main__":
    main()

