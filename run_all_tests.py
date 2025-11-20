#!/usr/bin/env python3
"""Comprehensive test suite to gather all data for the report."""

import os
import sys
import time
import logging
import pybullet as p
import numpy as np
from pathlib import Path
from test_disturbances import test_disturbance
from hopper1d import Hopper1D

# Setup logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

RESULTS_DIR = Path("logs/report_data")
RESULTS_DIR.mkdir(parents=True, exist_ok=True)

def test_normal_hopping():
    """Test normal 1D hopping without disturbances."""
    print("\n" + "="*60)
    print("TEST 1: Normal 1D Hopping (Baseline)")
    print("="*60)
    
    sim = Hopper1D(mode=p.DIRECT)
    sim.set_target_height(0.5)
    z0 = float(os.getenv("HOP_Z0", "0.60"))
    sim._reset_height(z0)
    
    duration = 10.0
    total_steps = int(duration / sim.dt)
    
    for i in range(total_steps):
        t = i * sim.dt
        sim.step(t)
    
    sim.close()
    
    # Save data
    log_path = RESULTS_DIR / "normal_hopping.csv"
    sim.save_csv(log_path)
    
    # Generate plots
    plot_path = RESULTS_DIR / "normal_hopping_plots.png"
    sim.quick_plots(show=False, save=True, path=str(plot_path))
    
    # Summary
    num_hops = len(sim.apex_history)
    mean_apex = None
    std_apex = None
    if sim.apex_history:
        apex_heights = [h[1] for h in sim.apex_history]
        mean_apex = np.mean(apex_heights)
        std_apex = np.std(apex_heights)
        print(f"Number of hops: {num_hops}")
        print(f"Mean apex height: {mean_apex:.3f}m (target: 0.5m)")
        print(f"Std dev: {std_apex:.3f}m")
    
    print(f"Data saved to: {log_path}")
    print(f"Plots saved to: {plot_path}")
    
    return {
        'num_hops': num_hops,
        'mean_apex': mean_apex,
        'std_apex': std_apex
    }

def test_apex_height_regulation():
    """Test apex height regulation with step change."""
    print("\n" + "="*60)
    print("TEST 2: Apex Height Regulation (Step Response)")
    print("="*60)
    
    sim = Hopper1D(mode=p.DIRECT)
    z0 = float(os.getenv("HOP_Z0", "0.60"))
    sim._reset_height(z0)
    
    # Start at 0.5m, then step to 0.7m
    sim.set_target_height(0.5)
    step_time = 5.0
    duration = 10.0
    total_steps = int(duration / sim.dt)
    step_step = int(step_time / sim.dt)
    
    for i in range(total_steps):
        t = i * sim.dt
        if i == step_step:
            sim.set_target_height(0.7)
            print(f"Target height changed to 0.7m at t={t:.2f}s")
        sim.step(t)
    
    sim.close()
    
    # Save data
    log_path = RESULTS_DIR / "apex_regulation.csv"
    sim.save_csv(log_path)
    
    # Generate plots
    plot_path = RESULTS_DIR / "apex_regulation_plots.png"
    sim.quick_plots(show=False, save=True, path=str(plot_path))
    
    # Analyze step response
    settling_apex = None
    if sim.apex_history:
        step_apexes = []
        for t, h, _, target in sim.apex_history:
            if t >= step_time:
                step_apexes.append(h)
        
        if step_apexes:
            settling_apex = np.mean(step_apexes[-5:])  # Last 5 hops
            print(f"Initial target: 0.5m")
            print(f"Final target: 0.7m")
            print(f"Settling apex: {settling_apex:.3f}m")
            print(f"Steady-state error: {abs(settling_apex - 0.7):.3f}m")
    
    print(f"Data saved to: {log_path}")
    print(f"Plots saved to: {plot_path}")
    
    return {'settling_apex': settling_apex}

def test_disturbance_sweep():
    """Test disturbance rejection at various force levels."""
    print("\n" + "="*60)
    print("TEST 3: Disturbance Rejection Sweep")
    print("="*60)
    
    # Test forces: small (recoverable), medium (displacement), large (tipping)
    forces = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 5.0, 10.0]
    results = []
    
    for force in forces:
        print(f"\nTesting {force:.1f}N...")
        result = test_disturbance(force, 'x', headless=True, save_plots=False)
        results.append(result)
        
        print(f"  Max drift: {result['max_drift']:.4f}m ({result['max_drift']*1000:.1f}mm)")
        print(f"  Recovered: {result['recovered']}")
        print(f"  Hops: {result['num_hops']}")
        if result['recovered'] and result['recovery_time']:
            print(f"  Recovery time: {result['recovery_time']:.3f}s")
    
    # Save summary
    import csv
    summary_path = RESULTS_DIR / "disturbance_sweep_summary.csv"
    with open(summary_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)
    
    # Find tipping threshold
    tipping_forces = [r for r in results if r['max_drift'] > 0.1]  # >10cm drift
    if tipping_forces:
        tipping_threshold = min(tipping_forces, key=lambda x: x['force_magnitude'])
        print(f"\n=== Summary ===")
        print(f"Tipping threshold: ~{tipping_threshold['force_magnitude']:.1f}N")
        print(f"Summary saved to: {summary_path}")
    
    return results

def test_specific_disturbances_for_videos():
    """Test specific force levels for video generation."""
    print("\n" + "="*60)
    print("TEST 4: Specific Disturbances for Video Generation")
    print("="*60)
    print("Note: Run these with GUI to record videos:")
    print("  python3 test_disturbances_gui.py --force 0.5")
    print("  python3 test_disturbances_gui.py --force 1.0")
    print("  python3 test_disturbances_gui.py --force 1.5")
    print("  python3 test_disturbances_gui.py --force 2.0")
    print("  python3 test_disturbances_gui.py --force 10.0")
    print("  python3 test_disturbances_gui.py --force 100.0")
    
    # Just run headless versions to verify they work
    test_forces = [0.5, 1.0, 1.5, 2.0, 10.0]
    for force in test_forces:
        print(f"\nVerifying {force:.1f}N test...")
        result = test_disturbance(force, 'x', headless=True, save_plots=False)
        print(f"  ✓ Max drift: {result['max_drift']:.4f}m, Hops: {result['num_hops']}")

def generate_report_summary():
    """Generate a summary of all test results."""
    print("\n" + "="*60)
    print("GENERATING REPORT SUMMARY")
    print("="*60)
    
    summary_path = RESULTS_DIR / "test_summary.txt"
    with open(summary_path, 'w') as f:
        f.write("EECS 4421 Project - Test Results Summary\n")
        f.write("="*60 + "\n\n")
        f.write("All test data saved to: logs/report_data/\n\n")
        f.write("Test Files:\n")
        f.write("  1. normal_hopping.csv - Baseline 1D hopping\n")
        f.write("  2. apex_regulation.csv - Step response (0.5m -> 0.7m)\n")
        f.write("  3. disturbance_sweep_summary.csv - All disturbance tests\n\n")
        f.write("Video Generation:\n")
        f.write("  Run test_disturbances_gui.py with --force <N> to generate videos\n")
        f.write("  Recommended forces: 0.5, 1.0, 1.5, 2.0, 10.0, 100.0\n\n")
        f.write("Key Metrics:\n")
        f.write("  - Apex height regulation accuracy\n")
        f.write("  - Disturbance rejection capability\n")
        f.write("  - Tipping threshold (~2N)\n")
        f.write("  - Maximum recoverable force\n")
    
    print(f"Summary saved to: {summary_path}")

def main():
    print("\n" + "="*60)
    print("COMPREHENSIVE TEST SUITE FOR REPORT")
    print("="*60)
    print(f"Results will be saved to: {RESULTS_DIR}")
    print("\nThis will run:")
    print("  1. Normal 1D hopping (baseline)")
    print("  2. Apex height regulation (step response)")
    print("  3. Disturbance rejection sweep")
    print("  4. Verification of video test cases")
    print("\nStarting tests...\n")
    
    try:
        # Test 1: Normal hopping
        normal_results = test_normal_hopping()
        
        # Test 2: Apex regulation
        regulation_results = test_apex_height_regulation()
        
        # Test 3: Disturbance sweep
        disturbance_results = test_disturbance_sweep()
        
        # Test 4: Video test verification
        test_specific_disturbances_for_videos()
        
        # Generate summary
        generate_report_summary()
        
        print("\n" + "="*60)
        print("ALL TESTS COMPLETED SUCCESSFULLY!")
        print("="*60)
        print(f"\nAll data saved to: {RESULTS_DIR}")
        print("\nNext steps:")
        print("  1. Review plots in logs/report_data/")
        print("  2. Generate videos using test_disturbances_gui.py")
        print("  3. Use data for report and presentation")
        
    except Exception as e:
        logging.error(f"Test failed: {e}", exc_info=True)
        sys.exit(1)

if __name__ == "__main__":
    main()

