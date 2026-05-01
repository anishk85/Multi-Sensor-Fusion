# Multi-Sensor Fusion for Husarion Lynx UGV

Research project comparing two sensor fusion pipelines on a Husarion Lynx mobile robot simulated in Gazebo. The main contribution is a **Recurrent Quantum Neural Network (RQNN)** pre-filter that denoises IMU, GPS, and odometry streams before an Extended Kalman Filter fuses them.

## Pipelines

| Pipeline | Launch File | Description |
|---|---|---|
| **Baseline EKF** | `baseline_ekf.launch.py` | Raw sensors → dual-EKF (control group) |
| **Hybrid QF-EKF** | `hybrid_qf_ekf.launch.py` | RQNN pre-filter → dual-EKF (proposed) |

All pipelines share a dual-EKF + NavSat architecture:

```
Raw Sensors (IMU / GPS / Wheel / Visual / LiDAR)
        │
  [quantum_filter_node]  ← hybrid only; baseline uses covariance relay nodes
        │
  EKF #1 (odom frame, no GPS)  →  /odometry/local  +  odom→base_link TF
        │
  EKF #2 (map frame, no GPS direct)  →  /odometry/global  +  map→odom TF
        │
  NavSat Transform (GPS lat/lon → /odometry/gps) ──→ EKF #2
```

---

## System Requirements

- **OS**: Ubuntu 22.04
- **ROS 2**: Humble Hawksbill
- **Gazebo**: Harmonic (gz-harmonic) or Fortress
- **Python**: 3.10+
- **GPU**: Optional but recommended for RTAB-Map visual odometry

---

## Installation

### 1. Install ROS 2 Humble

Follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

```bash
sudo apt install ros-humble-desktop ros-humble-ros-base
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Gazebo Harmonic

```bash
sudo apt install gz-harmonic
# ROS–Gazebo bridge
sudo apt install ros-humble-ros-gz
```

### 3. Install ROS 2 Dependencies

```bash
sudo apt install -y \
    ros-humble-robot-localization \
    ros-humble-rtabmap-ros \
    ros-humble-nav2-common \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-joy \
    ros-humble-gz-ros2-control \
    ros-humble-twist-mux \
    python3-colcon-common-extensions \
    python3-rosdep
```

### 4. Install Python Dependencies

```bash
pip install \
    rosbags \
    numpy \
    scipy \
    pandas \
    matplotlib \
    jupyterlab \
    ipykernel
```

> `rosbags` is the pure-Python library used by notebooks to read `.db3` bag files without a live ROS environment.

### 5. Clone and Build

```bash
git clone <repo-url> ~/Multi-sensor-fusion
cd ~/Multi-sensor-fusion

# Initialize rosdep
sudo rosdep init   # skip if already done
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

Add to `~/.bashrc` so every terminal has the workspace sourced:

```bash
echo "source ~/Multi-sensor-fusion/install/setup.bash" >> ~/.bashrc
```

---

## Environment Variables

Set these before launching (add to `~/.bashrc` or export per session):

```bash
# Required for Gazebo to find robot models/meshes
export GZ_SIM_RESOURCE_PATH=~/Multi-sensor-fusion/src/husarion_ugv_description/models:$GZ_SIM_RESOURCE_PATH

# Required for ros_gz bridge to locate world files
export GZ_SIM_WORLD_PATH=~/Multi-sensor-fusion/src/husarion_ugv_description/worlds:$GZ_SIM_WORLD_PATH

# Tells build scripts which variant to compile (simulation vs hardware)
export HUSARION_ROS_BUILD_TYPE=simulation

# Bag output directory (used by recording scripts)
export BAG_OUTPUT_DIR=/home/rosdata/rosbags
mkdir -p $BAG_OUTPUT_DIR
```

Apply:

```bash
source ~/.bashrc
```

---

## Launching Simulations

Always source the workspace first:

```bash
source ~/Multi-sensor-fusion/install/setup.bash
```

### Baseline EKF (control group — raw sensors, no RQNN)

```bash
ros2 launch husarion_ugv_description baseline_ekf.launch.py
```

Fuses: wheel odom + visual odom + lidar odom + IMU + GPS (all raw, covariance-fixed via relay nodes).

### Hybrid QF-EKF (proposed — RQNN pre-filter + dual EKF)

```bash
ros2 launch husarion_ugv_description hybrid_qf_ekf.launch.py
```

Starts: Gazebo → odom sources (RTAB-Map) + quantum_filter_node → dual EKF → navsat.

### Gazebo Only (no EKF)

```bash
ros2 launch husarion_ugv_description gz_lynx.launch.py
```

---

## Robot Teleoperation

### Keyboard

```bash
# From project root
python3 custom_teleop.py
```

Keys: `W/S` forward/back, `A/D` turn, `Q/Z` speed up/down, `E/C` turn rate up/down, `Space` stop.

### Xbox Controller

```bash
ros2 launch husarion_ugv_description xbox_teleop.launch.py
```

---

## Recording ROS Bag Data

Launch the simulation first, then in a **separate terminal**:

### Record Baseline (raw sensors + EKF outputs)

```bash
bash ~/Multi-sensor-fusion/record_baseline_bag.sh
```

Saves to: `/home/rosdata/rosbags/moving_bot_baseline_test_YYYYMMDD_HHMMSS`

Topics recorded: `/clock`, `/tf`, `/tf_static`, `/cmd_vel`, `/joint_states`, `/imu/data`, `/gps/fix`, `/diff_drive_controller/odom`, `/odometry/visual`, `/odometry/lidar`, `/odometry/local`, `/odometry/global`, `/ground_truth/odom`

### Record Hybrid (raw + RQNN-filtered + EKF outputs)

```bash
bash ~/Multi-sensor-fusion/record_hybrid_bag.sh
```

Saves to: `/home/rosdata/rosbags/moving_bot_live_test_YYYYMMDD_HHMMSS`

Additional topics: `/imu/data/filtered`, `/gps/fix/filtered`, `/diff_drive_controller/odom/filtered`, `/odometry/visual/filtered`, `/odometry/lidar/filtered`

### Playback

```bash
ros2 bag play /home/rosdata/rosbags/<bag_name> --clock
```

---

## Jupyter Notebooks — Setup and Path Updates

### Start JupyterLab

```bash
cd ~/Multi-sensor-fusion
jupyter lab
```

### Update Bag Paths

Each notebook has a config cell near the top with hardcoded bag paths. **Update these to match your recorded bags.**

#### [notebooks/baseline_ekf_analysis.ipynb](notebooks/baseline_ekf_analysis.ipynb)

```python
# Cell id: "config"
BAG_PATH = '/home/rosdata/rosbags/moving_bot_baseline_test_YYYYMMDD_HHMMSS'  # ← update
PLOT_DIR = '/home/anish/Multi-sensor-fusion/plots/baseline'                   # ← update if different username
```

#### [notebooks/qf_ekf_analysis.ipynb](notebooks/qf_ekf_analysis.ipynb)

```python
BAG_PATH = '/home/rosdata/rosbags/moving_bot_live_test_YYYYMMDD_HHMMSS'      # ← update
PLOT_DIR = '/home/anish/Multi-sensor-fusion/plots/qf_ekf'
```

#### [notebooks/ekf_comparison.ipynb](notebooks/ekf_comparison.ipynb)

```python
BASELINE_BAG = '/home/rosdata/rosbags/moving_bot_baseline_test_YYYYMMDD_HHMMSS'  # ← update
HYBRID_BAG   = '/home/rosdata/rosbags/moving_bot_live_test_YYYYMMDD_HHMMSS'      # ← update
```

### Plot Output Directories

Plots are written to:

```
plots/baseline/    — trajectory, velocity, yaw, IMU noise, drift, GPS innovation, covariance
plots/qf_ekf/      — RQNN filter effectiveness, phase lag, trajectory, drift, innovation
plots/comparison/  — side-by-side baseline vs hybrid for all metrics
```

Update `PLOT_DIR` in each notebook if your username is different from `anish`.

---

## Real-Time Visualization

### RViz2

An RViz config is provided:

```bash
rviz2 -d ~/Multi-sensor-fusion/src/husarion_ugv_description/rviz/husarion_ugv.rviz
```

### Foxglove Studio

A pre-built Foxglove dashboard layout is included:

1. Open [Foxglove Studio](https://foxglove.dev/download)
2. Connect to `ws://localhost:8765` (Foxglove bridge auto-starts with the launch files)
3. Import layout: `File → Import Layout → foxglove_baseline_layout.json`

---

## RQNN Retuning (optional)

The RQNN filter loads pretrained PSO-optimized hyperparameters from [src/husarion_ugv_description/config/rqnn_pretrained_params.json](src/husarion_ugv_description/config/rqnn_pretrained_params.json). To retune on new bag data:

```bash
python3 scripts/retune_rqnn_pso.py \
    --bag /home/rosdata/rosbags/<your_bag_name> \
    --out src/husarion_ugv_description/config/rqnn_pretrained_params.json
```

This reads raw sensor channels from the bag, runs PSO optimization per channel (IMU ax/ay/az/gx/gy/gz, wheel/visual/lidar vx/wz), and overwrites the params JSON. Rebuild the workspace after updating configs:

```bash
colcon build --symlink-install --packages-select husarion_ugv_description
```

---

## Project Structure

```
Multi-sensor-fusion/
├── src/husarion_ugv_description/
│   ├── launch/
│   │   ├── baseline_ekf.launch.py      — control group pipeline
│   │   ├── hybrid_qf_ekf.launch.py     — RQNN + EKF pipeline
│   │   ├── odom_sources.launch.py      — RTAB-Map visual + lidar odom
│   │   ├── gz_lynx.launch.py           — Gazebo simulation
│   │   └── xbox_teleop.launch.py       — Xbox controller
│   ├── config/
│   │   ├── baseline_ekf.yaml           — EKF params (control group)
│   │   ├── hybrid_qf_ekf.yaml          — EKF params (RQNN pipeline)
│   │   └── rqnn_pretrained_params.json — PSO-optimized RQNN weights
│   └── scripts/
│       ├── quantum_filter_node.py      — ROS 2 RQNN pre-filter node
│       ├── imu_covariance_relay.py     — Gazebo IMU covariance fix
│       └── gps_covariance_relay.py     — Gazebo GPS covariance fix
├── qnn_eeg_filtering/
│   ├── rqnn_model.py                   — RQNN filter (Schrödinger / Crank-Nicolson)
│   ├── pso_optimizer.py                — PSO hyperparameter search
│   └── RQNN_EXPLAINED.md              — math derivation + intuition
├── notebooks/
│   ├── baseline_ekf_analysis.ipynb     — analyze control group bag
│   ├── qf_ekf_analysis.ipynb           — analyze hybrid pipeline bag
│   └── ekf_comparison.ipynb            — side-by-side comparison
├── scripts/
│   └── retune_rqnn_pso.py              — PSO retuning from bag data
├── plots/
│   ├── baseline/                       — output plots for baseline
│   ├── qf_ekf/                         — output plots for hybrid
│   └── comparison/                     — comparison plots
├── record_baseline_bag.sh              — bag recording: baseline pipeline
├── record_hybrid_bag.sh                — bag recording: hybrid pipeline
├── custom_teleop.py                    — keyboard teleoperation
└── foxglove_baseline_layout.json       — Foxglove dashboard layout
```

---

## Troubleshooting

**`ModuleNotFoundError: No module named 'qnn_eeg_filtering'`**
The RQNN module is not on the Python path. Run from the project root or add it:
```bash
export PYTHONPATH=~/Multi-sensor-fusion:$PYTHONPATH
```

**Gazebo can't find robot model**
```bash
export GZ_SIM_RESOURCE_PATH=~/Multi-sensor-fusion/src/husarion_ugv_description/models:$GZ_SIM_RESOURCE_PATH
```

**EKF output drifts immediately at launch**
Sensors have zero covariance from Gazebo bridge. The covariance relay nodes (baseline) or quantum_filter_node (hybrid) fix this automatically — check they started correctly:
```bash
ros2 node list | grep -E "covariance|quantum"
```

**Bag path not found in notebook**
Check the bag directory:
```bash
ls /home/rosdata/rosbags/
```
Then update `BAG_PATH` in the notebook config cell to the correct timestamped folder name.

**`colcon build` fails on `gz_ros2_control`**
Make sure `gz-harmonic` is installed and `GZ_VERSION=harmonic` is set:
```bash
export GZ_VERSION=harmonic
colcon build --symlink-install
```
