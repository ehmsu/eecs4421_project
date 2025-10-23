import pybullet as p, pybullet_data, time
import numpy as np, pandas as pd, os
from controller import ApexController
from fsm import State, touchdown, liftoff

LOG_DIR = os.path.join("..","data","logs")
os.makedirs(LOG_DIR, exist_ok=True)

def main(gui=True):
    cid = p.connect(p.GUI if gui else p.DIRECT)
    p.setGravity(0,0,-9.81); p.setTimeStep(1/240.0); p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    hopper = p.loadURDF(os.path.join("src","urdf","hopper.urdf"), [0,0,0.6])
    idx = 0  # prismatic joint index

    params = dict(m=1.5, k=800.0, b=8.0, l0=0.6, g=9.81)
    ctrl = ApexController(k_h=50.0, alpha=1.0)

    state = State.FLIGHT
    h_star = 0.55
    step_time = 6.0

    t=0.0; dt = 1/240.0
    rows=[]; last_sign=np.sign(0.0)

    # turn off default motor
    p.setJointMotorControl2(hopper, idx, controlMode=p.VELOCITY_CONTROL, force=0.0)

    while t<12.0:
        if t>=step_time: h_star=0.70

        z, zdot, _, _ = p.getJointState(hopper, idx)

        if state==State.FLIGHT and touchdown(z, zdot, params["l0"]):
            state=State.STANCE
        elif state==State.STANCE and liftoff(z, zdot, **params):
            state=State.FLIGHT

        if state==State.STANCE:
            l_eff = ctrl.stance_rest_length(z, zdot, params, h_star)
            spring = params["k"] * (l_eff - z)
            damper = -params["b"] * zdot
            force  = spring + damper - params["m"]*params["g"]
        else:
            force  = 0.0

        # Apply vertical force at the base frame
        p.applyExternalForce(hopper, -1, [0,0,force], [0,0,0], p.LINK_FRAME)

        p.stepSimulation()
        if gui: time.sleep(1/480.0)
        t += dt
        rows.append((t,z,zdot,int(state==State.STANCE),h_star))

    df = pd.DataFrame(rows, columns=["t","z","zdot","in_stance","h_star"])
    out = os.path.join(LOG_DIR, "bullet_1d_log.csv")
    df.to_csv(out, index=False)
    print(f"Saved {out}")
    p.disconnect()

if __name__=="__main__":
    main(gui=True)
