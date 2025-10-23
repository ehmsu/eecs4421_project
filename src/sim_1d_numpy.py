import numpy as np, pandas as pd, matplotlib.pyplot as plt
from controller import ApexController
from fsm import State, touchdown, liftoff

# Parameters (tweak to taste)
params = dict(m=1.5, k=800.0, b=8.0, l0=0.6, g=9.81)
dt = 1/1000.0
T  = 12.0
h_star_1, h_star_2, step_time = 0.55, 0.70, 6.0  # step at 6 s

# State
z, zdot = 0.60, 0.0
state = State.FLIGHT
ctrl  = ApexController(k_h=50.0, alpha=1.0)

rows, last_sign = [], np.sign(zdot)
apex_list = []
h_star = h_star_1

for i in range(int(T/dt)):
    t = i*dt
    if t >= step_time: h_star = h_star_2

    # FSM transitions
    if state == State.FLIGHT and touchdown(z, zdot, params["l0"]):
        state = State.STANCE
    elif state == State.STANCE and liftoff(z, zdot, **params):
        state = State.FLIGHT

    # Dynamics
    if state == State.FLIGHT:
        zddot = -params["g"]
    elif state == State.STANCE:
        l_eff = ctrl.stance_rest_length(z, zdot, params, h_star)
        spring = params["k"] * (l_eff - z)
        damper = -params["b"] * zdot
        zddot = (spring + damper - params["m"]*params["g"]) / params["m"]
    else:
        zddot = 0.0

    # Integrate
    zdot += zddot*dt
    z    += zdot*dt

    # Apex detection (sign flip of zdot in flight)
    s = np.sign(zdot)
    if state == State.FLIGHT and (last_sign > 0 and s <= 0):
        apex_list.append((t, z))
    last_sign = s

    rows.append((t, z, zdot, int(state==State.STANCE), h_star))

df = pd.DataFrame(rows, columns=["t","z","zdot","in_stance","h_star"])

# Plot
plt.figure()
plt.plot(df.t, df.z, label="height z(t)")
plt.plot(df.t, df.h_star, "--", label="target h*")
plt.xlabel("time [s]"); plt.ylabel("height [m]"); plt.legend(); plt.tight_layout(); plt.show()

# Console summary
after = [h for (tt,h) in apex_list if tt>step_time]
if after:
    import statistics as S
    print(f"apex after step: mean={S.mean(after):.3f} m  std={S.pstdev(after):.3f} m")
