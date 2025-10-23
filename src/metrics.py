import numpy as np, pandas as pd

def settling_time(y, t, yref, band=0.05):
    tol = band*abs(yref)
    idx = np.where(np.abs(y - yref) <= tol)[0]
    if idx.size == 0: return np.nan
    # First time it enters band and stays
    for k in idx:
        if np.all(np.abs(y[k:] - yref) <= tol):
            return t[k]
    return np.nan

def apexes(df):
    z = df["z"].to_numpy(); t = df["t"].to_numpy()
    sgn = np.sign(np.gradient(z, t))
    idx = np.where((sgn[:-1] > 0) & (sgn[1:] <= 0))[0] + 1
    return t[idx], z[idx]
