# Task 2 AirSim Stereo-Inertial VIO Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run a prebuilt Windows AirSim Blocks environment, stream stereo images and IMU into ROS Noetic on WSL2, estimate pose with VINS-Fusion, visualize pose/path/point cloud in RViz, and record estimate plus AirSim ground truth for optional evo evaluation.

**Architecture:** Windows runs the precompiled AirSim Blocks simulator and exposes the RPC server on port 41451. WSL2 runs the AirSim ROS wrapper, normalizes stereo+IMU topics through `stereo_imu_relay.py`, feeds VINS-Fusion, adapts its output to stable project topics, and visualizes/records the results. AirSim ground truth is recorded on an isolated branch and never enters the estimator.

**Tech Stack:** Windows AirSim v1.8.1 Blocks, WSL2 Ubuntu 20.04, ROS Noetic, AirSim `airsim_ros_pkgs`, VINS-Fusion, OpenCV, RViz, evo.

**Spec:** `docs/superpowers/specs/2026-09-02-task2-airsim-vio-design.md`

## Global Constraints

- Use Windows precompiled AirSim Blocks; do not install Unreal Engine for the mandatory task.
- Run ROS Noetic, AirSim ROS wrapper, VINS-Fusion, and RViz inside the existing WSL2 Ubuntu 20.04 environment.
- Mandatory task only: stereo image + IMU VIO and RViz visualization. Do not implement EGO-Planner until mandatory acceptance is complete.
- AirSim configuration uses vehicle `Drone1`, cameras `left` and `right`, image size 640×480, horizontal FOV 90°, stereo baseline 0.20 m, and one enabled IMU.
- Use RPC port 41451 and ENU world coordinates in the ROS wrapper.
- AirSim ground-truth odometry is evaluation-only and must not feed VINS-Fusion.
- VINS-Fusion runs `imu: 1`, `num_of_cam: 2`, with fixed camera extrinsics from `config/vins/airsim_stereo_imu.yaml`.
- Keep a simple WSLg GUI alive when needed to avoid the known `[WARN:COPY MODE]` RViz issue encountered in Task 1.
- Do not claim runtime success, topic frequencies, ATE/RPE, or visualization output until observed on the user's machine.

---

## Existing File Map

No new runtime component is intended initially; the repository already contains the mandatory Task 2 pipeline.

- `airsim/settings_stereo_imu.json` — Windows AirSim sensor/vehicle configuration.
- `scripts/install_noetic_dependencies.sh` — ROS/C++/Python dependencies including `catkin-tools`.
- `scripts/build_task2_workspace.sh` — builds AirSim libraries/ROS wrapper and links VINS-Fusion into the main catkin workspace.
- `scripts/detect_wsl_host.sh` — identifies the Windows host address reachable from WSL2.
- `scripts/check_task2_topics.sh` — checks required Task 2 ROS topics/frequencies.
- `catkin_ws/src/slam_practice/launch/airsim_noetic_wsl.launch` — AirSim ROS wrapper launch, RPC 41451, ENU mode.
- `catkin_ws/src/slam_practice/launch/task2_airsim_vins.launch` — complete mandatory pipeline launch.
- `catkin_ws/src/slam_practice/scripts/stereo_imu_relay.py` — stereo approximate synchronization, grayscale conversion, topic normalization.
- `catkin_ws/src/slam_practice/config/vins/airsim_stereo_imu.yaml` — VINS-Fusion stereo+IMU estimator and extrinsic configuration.
- `catkin_ws/src/slam_practice/config/vins/airsim_cam0.yaml` — left-camera calibration.
- `catkin_ws/src/slam_practice/config/vins/airsim_cam1.yaml` — right-camera calibration.
- `catkin_ws/src/slam_practice/scripts/vins_output_adapter.py` — stable odometry/path/TF interface.
- `catkin_ws/src/slam_practice/scripts/trajectory_recorder.py` — VINS TUM trajectory recorder.
- `catkin_ws/src/slam_practice/scripts/airsim_gt_recorder.py` — AirSim ENU ground-truth TUM recorder.
- `catkin_ws/src/slam_practice/rviz/task2_vins.rviz` — mandatory visualization configuration.

Any code/config patch is deferred until a reproducible runtime or build failure identifies the exact incompatibility.

---

### Task 1: Start a Known-Good Windows AirSim Blocks Environment

**Files:**
- Consume: `airsim/settings_stereo_imu.json`
- Windows destination: `%USERPROFILE%\Documents\AirSim\settings.json`
- Windows simulator: extracted AirSim v1.8.1 `Blocks.zip`

**Interfaces:**
- Produces: AirSim RPC server reachable on TCP port `41451`.
- Produces: vehicle `Drone1` with cameras `left`, `right`, and sensor `Imu` configured by `settings.json`.

- [ ] **Step 1: Download the official precompiled Blocks package**

Use the AirSim v1.8.1 release asset `Blocks.zip` from:

```text
https://github.com/microsoft/AirSim/releases/download/v1.8.1/Blocks.zip
```

Expected download size is roughly 136 MiB compressed.

- [ ] **Step 2: Extract Blocks on the Windows filesystem**

Recommended destination:

```text
C:\AirSim\Blocks
```

Verify that the extracted directory contains the Windows executable for the Blocks environment.

- [ ] **Step 3: Install the repository AirSim settings as the active Windows settings file**

From WSL, determine the Windows username/path if necessary:

```bash
cmd.exe /C echo %USERPROFILE%
```

Create the AirSim settings directory from Windows Explorer or PowerShell:

```powershell
New-Item -ItemType Directory -Force "$env:USERPROFILE\Documents\AirSim"
```

Copy the repository file to:

```text
%USERPROFILE%\Documents\AirSim\settings.json
```

The copied file must configure `SimMode: Multirotor`, `Drone1`, `left`, `right`, and `Imu`.

- [ ] **Step 4: Launch Blocks**

Run the extracted Blocks executable and keep the simulator open.

Expected: the AirSim scene opens without asking for Unreal Engine or project compilation.

- [ ] **Step 5: Verify the RPC server from Windows**

In PowerShell while Blocks is running:

```powershell
Test-NetConnection 127.0.0.1 -Port 41451
```

Expected:

```text
TcpTestSucceeded : True
```

If false, do not proceed to ROS; inspect AirSim startup/settings and Windows firewall first.

---

### Task 2: Prepare Only the Mandatory WSL2 Dependencies

**Files:**
- Consume: `scripts/install_noetic_dependencies.sh`
- Consume/build: `third_party/AirSim`
- Consume/build: `third_party/VINS-Fusion`
- Existing Task 1 dependency: `third_party/ORB_SLAM3`

**Interfaces:**
- Produces: ROS Noetic build dependencies and `catkin build` support.
- Produces: local AirSim source tree for `AirLib` and `airsim_ros_pkgs`.
- Produces: local VINS-Fusion source tree.

- [ ] **Step 1: Enter the repository and load ROS Noetic**

```bash
cd ~/projects/ros-slam-airsim-practice
source /opt/ros/noetic/setup.bash
printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
```

Expected:

```text
ROS_DISTRO=noetic
```

- [ ] **Step 2: Install/check required Ubuntu packages**

```bash
./scripts/install_noetic_dependencies.sh
```

Expected: command exits with status 0 and prints `ROS Noetic 实践依赖安装完成。`

- [ ] **Step 3: Inspect existing third-party trees before cloning**

```bash
for d in AirSim VINS-Fusion ORB_SLAM3; do
  if [ -d "third_party/$d/.git" ]; then
    echo "$d: present"
  else
    echo "$d: missing"
  fi
done
```

- [ ] **Step 4: Clone only missing mandatory repositories**

If AirSim is missing:

```bash
git clone --depth 1 https://github.com/microsoft/AirSim.git third_party/AirSim
```

If VINS-Fusion is missing:

```bash
git clone --depth 1 https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git third_party/VINS-Fusion
```

ORB-SLAM3 should already exist from Task 1; if absent, restore it using the repository's Task 1 setup rather than introducing a second copy.

- [ ] **Step 5: Verify required sources exist**

```bash
test -f third_party/AirSim/setup.sh && echo 'AirSim source OK'
test -f third_party/VINS-Fusion/vins_estimator/CMakeLists.txt && echo 'VINS-Fusion source OK'
```

Expected: both `OK` lines appear.

---

### Task 3: Build AirSim ROS Wrapper and VINS-Fusion

**Files:**
- Consume: `scripts/build_task2_workspace.sh`
- Build output: `third_party/AirSim/AirLib/lib/x64/Release/libAirLib.a`
- Build output: `third_party/AirSim/ros/devel/setup.bash`
- Symlink: `catkin_ws/src/VINS-Fusion -> third_party/VINS-Fusion`
- Build output: `catkin_ws/devel/setup.bash`

**Interfaces:**
- Produces ROS package: `airsim_ros_pkgs` / executable `airsim_node`.
- Produces ROS package: `vins` / executable `vins_node`.
- Preserves existing `slam_practice` and `orbslam3_ros` packages.

- [ ] **Step 1: Run the repository Task 2 builder**

```bash
cd ~/projects/ros-slam-airsim-practice
source /opt/ros/noetic/setup.bash
./scripts/build_task2_workspace.sh
```

Do not patch build scripts on the first failure. Capture the first full error and diagnose its originating component before changing anything.

- [ ] **Step 2: Verify AirSim ROS workspace was produced**

```bash
test -f third_party/AirSim/ros/devel/setup.bash && echo 'AirSim ROS build OK'
```

- [ ] **Step 3: Verify the main workspace was produced**

```bash
test -f catkin_ws/devel/setup.bash && echo 'Main catkin build OK'
```

- [ ] **Step 4: Source both workspaces in dependency order**

```bash
source /opt/ros/noetic/setup.bash
source ~/projects/ros-slam-airsim-practice/third_party/AirSim/ros/devel/setup.bash
source ~/projects/ros-slam-airsim-practice/catkin_ws/devel/setup.bash
```

- [ ] **Step 5: Verify ROS can resolve both packages**

```bash
rospack find airsim_ros_pkgs
rospack find vins
rospack find slam_practice
```

Expected: all three commands print valid paths and exit 0.

---

### Task 4: Prove Windows AirSim → WSL2 ROS Sensor Transport Before Starting VINS

**Files:**
- Consume: `scripts/detect_wsl_host.sh`
- Consume: `catkin_ws/src/slam_practice/launch/airsim_noetic_wsl.launch`
- Consume: `airsim/settings_stereo_imu.json`

**Interfaces:**
- Consumes: AirSim RPC endpoint `<WindowsHostIP>:41451`.
- Produces: `/airsim_node/Drone1/left/Scene`.
- Produces: `/airsim_node/Drone1/right/Scene`.
- Produces: `/airsim_node/Drone1/imu/Imu`.
- Produces: `/airsim_node/Drone1/odom_local_enu`.

- [ ] **Step 1: Keep the Windows Blocks simulator running**

Do not start VINS yet.

- [ ] **Step 2: Resolve the Windows host address from WSL2**

```bash
cd ~/projects/ros-slam-airsim-practice
./scripts/detect_wsl_host.sh
export WSL_HOST_IP=$(awk '/^nameserver/{print $2; exit}' /etc/resolv.conf)
echo "$WSL_HOST_IP"
```

Expected: a non-empty IPv4 address.

- [ ] **Step 3: Check RPC reachability from WSL2**

```bash
python3 - <<'PY'
import os, socket
host = os.environ['WSL_HOST_IP']
s = socket.socket()
s.settimeout(3)
try:
    s.connect((host, 41451))
    print(f'RPC reachable: {host}:41451')
finally:
    s.close()
PY
```

Expected: `RPC reachable: ...:41451`.

- [ ] **Step 4: Launch only the AirSim ROS wrapper**

```bash
source /opt/ros/noetic/setup.bash
source ~/projects/ros-slam-airsim-practice/third_party/AirSim/ros/devel/setup.bash
source ~/projects/ros-slam-airsim-practice/catkin_ws/devel/setup.bash
roslaunch slam_practice airsim_noetic_wsl.launch host:=$WSL_HOST_IP
```

Expected: `airsim_node` remains alive and connects without RPC timeout/refused errors.

- [ ] **Step 5: Verify required sensor topics in another terminal**

```bash
source /opt/ros/noetic/setup.bash
source ~/projects/ros-slam-airsim-practice/third_party/AirSim/ros/devel/setup.bash
source ~/projects/ros-slam-airsim-practice/catkin_ws/devel/setup.bash
rostopic list | grep -E '/airsim_node/Drone1/(left/Scene|right/Scene|imu/Imu|odom_local_enu)'
```

Expected: all four required topics appear.

- [ ] **Step 6: Verify each stream is live**

Run individually:

```bash
rostopic hz /airsim_node/Drone1/left/Scene
rostopic hz /airsim_node/Drone1/right/Scene
rostopic hz /airsim_node/Drone1/imu/Imu
```

Let each command collect several samples, then stop it with `Ctrl+C`. Record the observed values instead of assuming expected frequencies.

- [ ] **Step 7: Inspect one message from each sensor class**

```bash
rostopic echo -n 1 /airsim_node/Drone1/left/Scene/header
rostopic echo -n 1 /airsim_node/Drone1/right/Scene/header
rostopic echo -n 1 /airsim_node/Drone1/imu/Imu
```

Acceptance: image headers have changing timestamps and IMU contains finite angular velocity and linear acceleration values.

---

### Task 5: Start the Stereo+IMU Relay and Validate VINS Inputs

**Files:**
- Consume: `catkin_ws/src/slam_practice/scripts/stereo_imu_relay.py`
- Consume: `catkin_ws/src/slam_practice/config/vins/airsim_stereo_imu.yaml`

**Interfaces:**
- Consumes: `/airsim_node/Drone1/left/Scene`, `/right/Scene`, `/imu/Imu`.
- Produces: `/vins_fusion/cam0/image_raw`, `/vins_fusion/cam1/image_raw`, `/vins_fusion/imu`.
- Stereo pairs must be normalized to a shared timestamp when accepted by the relay.

- [ ] **Step 1: Keep the AirSim wrapper running from Task 4**

Open a new terminal and source the same ROS workspaces.

- [ ] **Step 2: Run only the relay node**

```bash
rosrun slam_practice stereo_imu_relay.py
```

Expected: node remains alive without repeated image-conversion or synchronization errors.

- [ ] **Step 3: Confirm normalized VINS topics exist**

```bash
rostopic list | grep '^/vins_fusion/'
```

Expected:

```text
/vins_fusion/cam0/image_raw
/vins_fusion/cam1/image_raw
/vins_fusion/imu
```

- [ ] **Step 4: Verify left and right output frequencies**

```bash
rostopic hz /vins_fusion/cam0/image_raw
rostopic hz /vins_fusion/cam1/image_raw
```

Record observed values.

- [ ] **Step 5: Verify output images are mono8**

```bash
rostopic echo -n 1 /vins_fusion/cam0/image_raw/encoding
rostopic echo -n 1 /vins_fusion/cam1/image_raw/encoding
```

Expected both:

```text
"mono8"
```

- [ ] **Step 6: Verify accepted stereo pair timestamps match**

Capture one header from each output topic immediately after data begins:

```bash
rostopic echo -n 1 /vins_fusion/cam0/image_raw/header
rostopic echo -n 1 /vins_fusion/cam1/image_raw/header
```

Expected: relay-generated accepted pairs use a common timestamp. If not, stop before launching VINS and debug the relay.

---

### Task 6: Run VINS-Fusion and Achieve Stable Stereo-Inertial Initialization

**Files:**
- Consume: `catkin_ws/src/slam_practice/config/vins/airsim_stereo_imu.yaml`
- Consume: `catkin_ws/src/slam_practice/config/vins/airsim_cam0.yaml`
- Consume: `catkin_ws/src/slam_practice/config/vins/airsim_cam1.yaml`
- Consume: `catkin_ws/src/slam_practice/launch/task2_airsim_vins.launch`

**Interfaces:**
- Consumes: normalized `/vins_fusion/*` topics.
- Produces: `/vins_estimator/odometry`, `/vins_estimator/path`, `/vins_estimator/point_cloud`.
- Produces stable project interface `/slam_practice/vins/odometry`, `/slam_practice/vins/path`.

- [ ] **Step 1: Stop the standalone wrapper/relay tests cleanly**

Use `Ctrl+C` in their terminals so the integrated launch owns each node exactly once.

- [ ] **Step 2: Prepare the result directory**

```bash
mkdir -p ~/slam_results/task2
rm -f ~/slam_results/task2/vins_estimate.tum ~/slam_results/task2/airsim_groundtruth.tum
```

- [ ] **Step 3: Apply the Task 1 WSLg workaround before RViz if needed**

If `[WARN:COPY MODE]` returns, from Windows PowerShell run:

```powershell
wsl --shutdown
```

Reopen WSL, wait a few seconds, start `glxgears`, leave it open, then continue.

- [ ] **Step 4: Launch the complete mandatory pipeline**

```bash
source /opt/ros/noetic/setup.bash
source ~/projects/ros-slam-airsim-practice/third_party/AirSim/ros/devel/setup.bash
source ~/projects/ros-slam-airsim-practice/catkin_ws/devel/setup.bash
export WSL_HOST_IP=$(awk '/^nameserver/{print $2; exit}' /etc/resolv.conf)
roslaunch slam_practice task2_airsim_vins.launch host:=$WSL_HOST_IP
```

- [ ] **Step 5: Move the multirotor sufficiently for VIO initialization**

Use AirSim's available keyboard/controller input or API control to introduce both translation and rotation rather than only hovering. Keep motion smooth and remain inside the Blocks scene.

- [ ] **Step 6: Confirm VINS outputs appear**

In another sourced terminal:

```bash
rostopic list | grep -E '^/vins_estimator/(odometry|path|point_cloud)$|^/slam_practice/vins/(odometry|path)$'
```

Expected: estimator and adapted output topics appear.

- [ ] **Step 7: Verify pose is live**

```bash
rostopic hz /slam_practice/vins/odometry
```

Acceptance: a sustained nonzero pose update rate after VINS initialization.

- [ ] **Step 8: Run the repository Task 2 topic checker**

```bash
cd ~/projects/ros-slam-airsim-practice
./scripts/check_task2_topics.sh
```

Capture the full output for the experiment record.

---

### Task 7: Validate RViz Visualization and Record Mandatory Evidence

**Files:**
- Consume: `catkin_ws/src/slam_practice/rviz/task2_vins.rviz`
- Runtime outputs: `~/slam_results/task2/vins_estimate.tum`
- Runtime outputs: `~/slam_results/task2/airsim_groundtruth.tum`

**Interfaces:**
- RViz displays `/slam_practice/vins/path`, `/slam_practice/vins/odometry`, `/vins_estimator/point_cloud`, and `/vins_fusion/cam0/image_raw`.

- [ ] **Step 1: Confirm RViz is responsive**

Acceptance: the RViz window can be focused and manipulated; no `[WARN:COPY MODE]` broken-window state.

- [ ] **Step 2: Confirm the left-camera panel updates**

Acceptance: `/vins_fusion/cam0/image_raw` visibly changes as the drone moves.

- [ ] **Step 3: Confirm pose/path visualization updates continuously**

Acceptance: the current pose moves and `/slam_practice/vins/path` grows while AirSim flies.

- [ ] **Step 4: Confirm VINS feature point cloud appears**

Acceptance: `/vins_estimator/point_cloud` contains visible points when the camera observes textured scene regions.

- [ ] **Step 5: Capture the required evidence**

Save screenshots/recording containing:

```text
AirSim Blocks scene
left/right camera evidence
sensor/topic frequency terminal
VINS initialization/output terminal
RViz pose + path + point cloud
```

The mandatory assignment requirement is satisfied by the real-time RViz pose-estimation visualization; retain a screen recording if possible.

- [ ] **Step 6: Stop with Ctrl+C and verify trajectory files**

```bash
ls -lh ~/slam_results/task2/vins_estimate.tum ~/slam_results/task2/airsim_groundtruth.tum
wc -l ~/slam_results/task2/vins_estimate.tum ~/slam_results/task2/airsim_groundtruth.tum
head -1 ~/slam_results/task2/vins_estimate.tum
tail -1 ~/slam_results/task2/vins_estimate.tum
```

Acceptance: both files are non-empty valid TUM trajectories; estimate timestamps span the demonstrated motion interval.

---

### Task 8: Optional Quantitative Check of the Mandatory VIO Result

This is not required by the assignment wording but strengthens the report and reuses Task 1 tooling.

**Files:**
- Consume: `~/slam_results/task2/vins_estimate.tum`
- Consume: `~/slam_results/task2/airsim_groundtruth.tum`
- Consume: Task 1 `evaluate_trajectory.py`/evo environment.
- Produce: `~/slam_results/task2/evaluation/`

**Interfaces:**
- Ground truth and VINS estimate are compared after rigid SE(3) alignment.
- Stereo-inertial evaluation must not use monocular scale correction.

- [ ] **Step 1: Activate the evo environment**

```bash
cd ~/projects/ros-slam-airsim-practice
source .venv-evo/bin/activate
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
```

- [ ] **Step 2: Remove stale evaluation output**

```bash
rm -rf ~/slam_results/task2/evaluation
mkdir -p ~/slam_results/task2/evaluation
```

- [ ] **Step 3: Run stereo-inertial evaluation**

```bash
rosrun slam_practice evaluate_trajectory.py \
  ~/slam_results/task2/airsim_groundtruth.tum \
  ~/slam_results/task2/vins_estimate.tum \
  ~/slam_results/task2/evaluation \
  --sensor stereo-inertial
```

- [ ] **Step 4: Record only the measured outputs**

Inspect generated metrics/plots and copy the observed ATE/RPE values into the report. Do not reuse Task 1 numbers or invent placeholders as results.

---

## Completion Gate

The mandatory Task 2 is complete only when all of the following are observed on the user's machine:

- Windows AirSim Blocks runs with the configured `Drone1` stereo cameras and IMU.
- WSL2 AirSim ROS wrapper connects to the Windows RPC endpoint.
- Both stereo image streams and IMU are live in ROS.
- Relay outputs valid synchronized mono8 stereo streams and IMU to VINS topics.
- VINS-Fusion initializes and continuously publishes odometry.
- RViz visibly updates pose, path, left image, and VINS point cloud during motion.
- The required real-time pose-estimation visualization has been recorded or screenshotted.
- Estimate and AirSim ground-truth trajectories are saved without feeding GT into VINS.

Only after this gate is satisfied should the project decide whether to start the optional EGO-Planner obstacle-avoidance task.
