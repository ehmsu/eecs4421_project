#!/bin/bash
# FILE: setup_and_run.sh (Final Path Correction)

set -e # Exit immediately if a command fails

echo "--- 1. Setting up Python 3.10+ Virtual Environment ---"
# ... (Lines 1-25 are unchanged)
if [ -d ".venv" ]; then
    echo ".venv directory already exists. Re-using."
else
    echo "Creating .venv..."
    python3 -m venv .venv
fi

# Activate the venv
source .venv/bin/activate
echo "Virtual environment activated."

echo "--- 2. Installing Dependencies (including xacro) ---"
pip install -q -U pip
pip install -q -r requirements.txt
echo "Dependencies installed."

echo "--- 3. Setting up 'assets' Directory and Locating Model ---"

# --- NEW ROBUST COPY LOGIC ---

# The location where your OS or previous attempts placed the files.
SOURCE_DIR="assets/jumping_robot_urdf_backup_copy_description" 
# The required, standardized location for xacro to run.
TARGET_ASSETS_DIR="assets/jumping_robot_description_root"

mkdir -p ${TARGET_ASSETS_DIR}/urdf 
echo "Ensured ${TARGET_ASSETS_DIR}/urdf directory exists."

if [ -d "$SOURCE_DIR" ]; then
    echo "Found existing source files at ${SOURCE_DIR}. Moving to target location..."
    # Copy the CONTENTS of the user's directory into the required target directory
    cp -r ${SOURCE_DIR}/* ${TARGET_ASSETS_DIR}/
    echo "Files successfully moved/copied to ${TARGET_ASSETS_DIR}/."
    
    # We remove the old, unwanted directory to clean up
    rm -rf ${SOURCE_DIR}
else
    # Fallback to the original, highly specific download path (in case files were never copied)
    # This path is complex, but we keep it as a final attempt for fresh downloads
    USER_MODEL_DIR="~/Downloads/jumping_robot_urdf_backup_copy_description(1)\ (1)/jumping_robot_urdf_backup_copy_description"
    
    echo "Attempting rsync from complex download path: ${USER_MODEL_DIR} ..."
    if rsync -a --ignore-existing "${USER_MODEL_DIR}/" "${TARGET_ASSETS_DIR}/" 2>/dev/null; then
        echo "User model successfully copied to ${TARGET_ASSETS_DIR}/."
    else
        echo "FAIL: Could not copy model from either location. Using fallback box for simulation."
    fi
fi

# --- END NEW ROBUST COPY LOGIC ---

echo "--- 4. Creating 1-D Wrapper Xacro ---"
# ... (This section is unchanged and relies on TARGET_ASSETS_DIR/urdf)
cat << 'EOF' > ${TARGET_ASSETS_DIR}/urdf/eecs4421_1d_wrapper.xacro
<?xml version="1.0"?>
<robot name="eecs4421_1d_hopper" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <link name="world" />

  <xacro:include filename="jumping_robot_urdf_backup_copy.xacro" />

  <joint name="world_z_joint" type="prismatic">
    <parent link="world" />
    <child link="base_link" /> 
    <origin xyz="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="0.0" upper="5.0" effort="100.0" velocity="10.0" />
  </joint>

</robot>
EOF

echo "Wrapper 'eecs4421_1d_wrapper.xacro' created."

echo "--- 5. Generating Final .urdf File ---"
XACRO_SOURCE="${TARGET_ASSETS_DIR}/urdf/eecs4421_1d_wrapper.xacro"
XACRO_TARGET="${TARGET_ASSETS_DIR}/eecs4421_1d_hopper.urdf"

if [ -f "$XACRO_SOURCE" ]; then
    echo "Converting $XACRO_SOURCE -> $XACRO_TARGET"
    xacro --inorder "$XACRO_SOURCE" > "$XACRO_TARGET"
    echo "Conversion complete."
else
    echo "WARNING: Could not find $XACRO_SOURCE. Simulation will use the fallback box."
fi


echo "--- 6. Running Main GUI Simulation (FR-1 Test) ---"
echo "This will run for 12 seconds..."
python3 run_gui.py

echo "--- 7. Running Evaluation Script (FR-1 Metrics) ---"
# ... (Lines 141-150 are unchanged)
python3 eval_metrics.py logs/run1_apex.csv 0.5 0.7

echo "--- 8. Running Parameter Sweep (Example) ---"
echo "This will run a headless sweep. See logs/sweeps/summary.csv for results."
python3 sweep.py

echo "--- Setup and Run Complete! ---"
echo
echo "Next steps:"
echo "1. Activate the venv: source .venv/bin/activate"
echo "2. Run the sim again: python3 run_gui.py"
echo "3. Check the sweep results: cat logs/sweeps/summary.csv"
echo "4. Deactivate venv when done: deactivate"