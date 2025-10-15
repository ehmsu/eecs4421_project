# Mr. Springs — PyBullet v1 (strict 1-D vertical)

## Setup
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

## Run (GUI)
python run_gui.py

## What this does
- Locks the body to a prismatic joint along +Z (1-D only).
- Uses a spring-damper contact + a stance pulse u_h to regulate apex height.
- Logs t, z, vz, state, u_h, apexes; shows quick plots.
