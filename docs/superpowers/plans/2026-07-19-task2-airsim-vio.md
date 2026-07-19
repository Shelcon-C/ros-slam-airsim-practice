# Task 2 AirSim Stereo-Inertial VIO Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect Windows AirSim to ROS Noetic in WSL2, normalize synchronized stereo and IMU streams, run VINS-Fusion, visualize its live output in RViz, and record estimate and ground-truth trajectories.

**Architecture:** AirSim remains on Windows while its official ROS Wrapper, bridge nodes, VINS-Fusion, and RViz run in WSL2. The wrapper publishes ENU data. A thin relay synchronizes stereo images and preserves IMU timestamps. Output adapters provide stable topics and trajectory files independently of VINS private topic names.

**Tech Stack:** AirSim, ROS Noetic, `airsim_ros_pkgs`, VINS-Fusion, Python 3, message_filters, cv_bridge, RViz, evo.

## Global Constraints

- AirSim runs on Windows and ROS runs in Ubuntu 20.04 WSL2.
- VINS input uses 640x480, 20 Hz stereo images and IMU at 200 Hz.
- Coordinate system is ENU; camera optical axes follow OpenCV convention.
- AirSim truth is excluded from estimator inputs and used only for evaluation.

---

### Task 1: Stereo timing and geometry core

**Files:**
- Create: `catkin_ws/src/slam_practice/src/slam_practice/synchronization.py`
- Create: `catkin_ws/src/slam_practice/src/slam_practice/geometry.py`
- Test: `catkin_ws/src/slam_practice/tests/test_synchronization.py`
- Test: `catkin_ws/src/slam_practice/tests/test_geometry.py`

**Interfaces:**
- Produces: `validate_stereo_delta(left, right, tolerance=0.003)`, `ned_to_enu_vector`, quaternion normalization, pose composition, and AirSim optical-to-body rotation.

- [ ] Write tests for accepted and rejected 3 ms stereo pairs, NED-to-ENU vector conversion, normalized quaternions, and camera-to-body pose composition.
- [ ] Run tests; expect missing-symbol failures.
- [ ] Implement pure functions using only Python and NumPy, with explicit finite-value checks.
- [ ] Re-run all tests; expect pass.
- [ ] Commit with `feat(task2): add synchronization and geometry core`.

### Task 2: AirSim relay and trajectory nodes

**Files:**
- Create: `catkin_ws/src/slam_practice/scripts/stereo_imu_relay.py`
- Create: `catkin_ws/src/slam_practice/scripts/vins_output_adapter.py`
- Create: `catkin_ws/src/slam_practice/scripts/airsim_gt_recorder.py`

**Interfaces:**
- Consumes: AirSim left/right Scene images, IMU, VINS odometry/path, and AirSim ENU odometry.
- Produces: `/vins_fusion/cam0/image_raw`, `/vins_fusion/cam1/image_raw`, `/vins_fusion/imu`, `/slam_practice/vins/*`, and TUM files.

- [ ] Add script contract tests for required subscriber and publisher topic parameters, 3 ms ApproximateTimeSynchronizer, `mono8` conversion, TF publication, and TUM ordering.
- [ ] Run tests; expect failure while scripts are absent.
- [ ] Implement relay, stable output adapter with bounded path, and truth recorder using core utilities.
- [ ] Run compileall and all unit tests; expect pass.
- [ ] Commit with `feat(task2): add AirSim VIO bridge nodes`.

### Task 3: AirSim and VINS-Fusion configurations

**Files:**
- Create: `airsim/settings_stereo_imu.json`
- Create: `catkin_ws/src/slam_practice/config/vins/airsim_cam0.yaml`
- Create: `catkin_ws/src/slam_practice/config/vins/airsim_cam1.yaml`
- Create: `catkin_ws/src/slam_practice/config/vins/airsim_stereo_imu.yaml`
- Create: `catkin_ws/src/slam_practice/launch/airsim_noetic_wsl.launch`
- Create: `catkin_ws/src/slam_practice/launch/task2_airsim_vins.launch`
- Create: `catkin_ws/src/slam_practice/rviz/task2_vins.rviz`

**Interfaces:**
- Produces: AirSim camera/sensor setup and one launch entry point for bridge, VINS, recording, and RViz.

- [ ] Add tests that parse JSON and XML, assert 0.20 m baseline, equal camera intrinsics, IMU enabled, ENU parameters, exact VINS topics, two cameras, IMU enabled, and no truth topic in VINS config.
- [ ] Run tests; expect missing-file failures.
- [ ] Implement configurations based on AirSim ROS Wrapper and VINS-Fusion upstream schemas. Use `vins_node` with a single config argument and namespace `vins_estimator`.
- [ ] Run parsers, script syntax checks, and all unit tests.
- [ ] Commit with `feat(task2): add AirSim and VINS launch configuration`.

### Task 4: WSL diagnostics and practice record

**Files:**
- Create: `scripts/detect_wsl_host.sh`
- Create: `scripts/check_task2_topics.sh`
- Create: `docs/practice_records/02_task2_airsim_vio.md`
- Create: `docs/practice_records/04_results.md`

**Interfaces:**
- Produces: preflight checks and a node-by-node execution/evidence record.

- [ ] Add shell contract tests for host detection, port 41451 probing, required ROS topics, stereo rate, IMU rate, and timestamp delta checks.
- [ ] Run contract tests; expect missing-file failures.
- [ ] Implement diagnostics without changing firewall state. Document Windows startup, settings placement, WSL host discovery, build, launch, RViz evidence, rosbag recording, evo evaluation, and failure branches.
- [ ] Run `bash -n`, compileall, configuration tests, and all unit tests.
- [ ] Commit with `docs(task2): add diagnostics and VIO practice record`.

