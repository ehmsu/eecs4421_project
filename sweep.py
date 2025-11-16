import pybullet as p
import pandas as pd
import itertools
from pathlib import Path
from hopper1d import Hopper1D

SWEEP_LOG_DIR = Path("logs/sweeps")
SIM_DURATION_S = 12.0
STEP_CHANGE_TIME_S = 5.0
H_TARGET_INITIAL = 0.5
H_TARGET_FINAL = 0.7

PARAM_GRID = {
    'K_h': [2000.0, 3000.0],
    'B_h': [20.0, 30.0],
    'k_raibert': [0.15, 0.25],
    'T_stance_estimate': [0.15]
}

def calc_metrics(apex, h0, h1, band=0.05):
    res = {'overshoot_pct': float('inf'), 'sse': float('inf'), 'settling_hops': float('inf'), 'total_hops': 0}
    if not apex: return res
    import pandas as pd
    df = pd.DataFrame(apex, columns=['t_apex','apex_h','F_peak_stance','h_target'])
    res['total_hops'] = len(df)
    idx = df[df['h_target']==h1].index
    if len(idx)==0: return res
    i0 = idx[0]
    post = df.iloc[i0:].reset_index(drop=True)
    if post.empty: return res
    peak = post['apex_h'].max()
    over = max(0.0, peak - h1)
    res['overshoot_pct'] = (over / max(1e-9,(h1-h0))) * 100.0
    res['sse'] = abs(h1 - post['apex_h'].iloc[-1])
    b = band*(h1-h0); lo, hi = h1-b, h1+b
    for i in range(len(post)):
        if lo <= post['apex_h'].iloc[i] <= hi:
            if (post['apex_h'].iloc[i:] >= lo).all() and (post['apex_h'].iloc[i:] <= hi).all():
                res['settling_hops'] = i+1; break
    return res

def main():
    SWEEP_LOG_DIR.mkdir(parents=True, exist_ok=True)
    keys = list(PARAM_GRID.keys())
    combos = list(itertools.product(*[PARAM_GRID[k] for k in keys]))
    rows = []
    for run_idx, vals in enumerate(combos):
        params = dict(zip(keys, vals))
        sim = Hopper1D(mode=p.DIRECT)
        for k,v in params.items(): setattr(sim, k, v)
        sim.set_target_height(H_TARGET_INITIAL)
        steps = int(SIM_DURATION_S/sim.dt); step_change = int(STEP_CHANGE_TIME_S/sim.dt)
        for i in range(steps):
            if i==step_change: sim.set_target_height(H_TARGET_FINAL)
            sim.step(i*sim.dt)
        sim.close()
        base = SWEEP_LOG_DIR / f"run_{run_idx}"
        sim.save_csv(base)
        m = calc_metrics(sim.apex_history, H_TARGET_INITIAL, H_TARGET_FINAL)
        rows.append({**params, **m})
    import pandas as pd
    df = pd.DataFrame(rows)
    out = SWEEP_LOG_DIR / "summary.csv"
    df.to_csv(out, index=False)
    if not df.empty:
        df = df.sort_values(by=['sse','settling_hops'])
        print("\n--- Sweep Results (Top 2 by SSE) ---")
        print(df.head(2).to_string())

if __name__ == "__main__":
    main()
