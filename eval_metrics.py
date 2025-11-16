import pandas as pd, argparse

def analyze(apex_csv, h0, h1, band=0.05):
    df = pd.read_csv(apex_csv)
    step_idx = df.index[df["h_target"]==h1]
    if len(step_idx)==0: 
        print("No step found."); return
    step_idx = step_idx[0]
    post = df.iloc[step_idx:].reset_index(drop=True)
    peak = post["apex_h"].max()
    overshoot = max(0.0, (peak - h1)/(h1-h0)*100.0)
    sse = abs(h1 - post["apex_h"].iloc[-1])
    ub = h1 + band*(h1-h0); lb = h1 - band*(h1-h0)
    settle = float("inf")
    for i in range(len(post)):
        if lb <= post["apex_h"].iloc[i] <= ub and (post["apex_h"].iloc[i:]<=ub).all() and (post["apex_h"].iloc[i:]>=lb).all():
            settle = i+1; break
    print("--- FR-1: Step Response ---")
    print(f"Overshoot (%): {overshoot:.2f}")
    print(f"Steady-State Error (m): {sse:.4f}")
    print(f"Settling Time (hops): {settle}")

if __name__=="__main__":
    ap, a = argparse.ArgumentParser(), None
    ap.add_argument("apex_csv"); ap.add_argument("h0", type=float); ap.add_argument("h1", type=float)
    a = ap.parse_args(); analyze(a.apex_csv, a.h0, a.h1)
