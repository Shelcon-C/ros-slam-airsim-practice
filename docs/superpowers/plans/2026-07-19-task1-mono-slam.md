# Task 1 Monocular SLAM Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a replaceable EuRoC-to-ORB-SLAM3 monocular pipeline with calibration, ROS publishing, trajectory recording, evo ATE/RPE evaluation, and per-node practice records.

**Architecture:** Pure Python modules own dataset parsing, TUM formatting, calibration serialization, and evo command construction so they can be tested without ROS. Thin ROS scripts call those modules. A separate C++ catkin package wraps the upstream ORB-SLAM3 library and publishes pose, odometry, path, and tracked map points.

**Tech Stack:** Ubuntu 20.04, ROS Noetic, Python 3, OpenCV, C++14, ORB-SLAM3, evo, `unittest`.

## Global Constraints

- Current dataset is EuRoC MAV MH_01_easy; the future assigned video must be swappable without changing SLAM or evaluation nodes.
- Monocular evaluation uses `--align --correct_scale`.
- No dataset or third-party repository is committed.
- Unexecuted metrics and screenshots remain explicitly marked as pending.

---

### Task 1: Testable dataset and trajectory core

**Files:**
- Create: `catkin_ws/src/slam_practice/setup.py`
- Create: `catkin_ws/src/slam_practice/src/slam_practice/dataset.py`
- Create: `catkin_ws/src/slam_practice/src/slam_practice/trajectory.py`
- Test: `catkin_ws/src/slam_practice/tests/test_dataset.py`
- Test: `catkin_ws/src/slam_practice/tests/test_trajectory.py`

**Interfaces:**
- Produces: `load_euroc_camera_index(root: Path) -> list[Frame]`, `convert_euroc_groundtruth(source, destination) -> int`, and `format_tum_pose(stamp, position, quaternion_xyzw) -> str`.

- [ ] Write tests using a temporary EuRoC directory. Assert timestamp ordering, missing-image errors, quaternion reordering from EuRoC `qw,qx,qy,qz` to TUM `qx,qy,qz,qw`, and nine TUM columns.
- [ ] Run `python3 -m unittest discover -s catkin_ws/src/slam_practice/tests -p 'test_*.py' -v`; expect import or symbol failures.
- [ ] Implement immutable `Frame`, strict CSV parsing, path validation, monotonic timestamp validation, TUM formatting, and ground-truth conversion.
- [ ] Re-run the tests; expect all dataset and trajectory tests to pass.
- [ ] Commit with `feat(task1): add EuRoC parsing and trajectory core`.

### Task 2: Calibration and evo evaluation core

**Files:**
- Create: `catkin_ws/src/slam_practice/src/slam_practice/calibration.py`
- Create: `catkin_ws/src/slam_practice/src/slam_practice/evaluation.py`
- Test: `catkin_ws/src/slam_practice/tests/test_calibration.py`
- Test: `catkin_ws/src/slam_practice/tests/test_evaluation.py`

**Interfaces:**
- Produces: `orbslam_yaml_text(CameraCalibration) -> str` and `build_evo_commands(gt, estimate, output_dir, monocular) -> list[list[str]]`.

- [ ] Write tests asserting ORB-SLAM3 `Camera1.*` fields and two evo commands. The monocular commands must contain `--align` and `--correct_scale`; stereo-inertial commands must omit scale correction.
- [ ] Run the two test modules; expect failures because modules do not exist.
- [ ] Implement serialization with finite-value and positive-dimension checks. Implement deterministic `evo_ape`, `evo_rpe`, and `evo_res` command construction without `shell=True`.
- [ ] Re-run all tests; expect pass.
- [ ] Commit with `feat(task1): add calibration and evo evaluation core`.

### Task 3: ROS data, recording, calibration, and evaluation nodes

**Files:**
- Create: `catkin_ws/src/slam_practice/package.xml`
- Create: `catkin_ws/src/slam_practice/CMakeLists.txt`
- Create: `catkin_ws/src/slam_practice/scripts/euroc_mono_publisher.py`
- Create: `catkin_ws/src/slam_practice/scripts/trajectory_recorder.py`
- Create: `catkin_ws/src/slam_practice/scripts/euroc_groundtruth_to_tum.py`
- Create: `catkin_ws/src/slam_practice/scripts/calibrate_camera.py`
- Create: `catkin_ws/src/slam_practice/scripts/evaluate_trajectory.py`

**Interfaces:**
- Consumes: core APIs from Tasks 1 and 2.
- Produces: `/camera/mono/image_raw`, `/camera/mono/camera_info`, and TUM trajectory files.

- [ ] Add a static test that imports each script with ROS imports replaced by explicit runtime-only guards and verifies every file has a `main()` function.
- [ ] Run the test and confirm missing scripts fail.
- [ ] Implement the publisher with original frame intervals divided by `~playback_rate`; implement a generic PoseStamped/Odometry recorder; implement calibration CLI using OpenCV checkerboard detection and reprojection error; implement safe evo subprocess execution.
- [ ] Run `python3 -m compileall catkin_ws/src/slam_practice` and all unit tests; expect pass.
- [ ] Commit with `feat(task1): add ROS nodes and command-line tools`.

### Task 4: ORB-SLAM3 ROS wrapper

**Files:**
- Create: `catkin_ws/src/orbslam3_ros/package.xml`
- Create: `catkin_ws/src/orbslam3_ros/CMakeLists.txt`
- Create: `catkin_ws/src/orbslam3_ros/src/mono_node.cpp`

**Interfaces:**
- Consumes: `/camera/mono/image_raw`, ORB vocabulary, and ORB settings.
- Produces: `/orbslam3/pose`, `/orbslam3/odometry`, `/orbslam3/path`, `/orbslam3/tracked_points`.

- [ ] Add `tests/test_cpp_contract.py` to assert that the wrapper calls `TrackMonocular`, inverts `Tcw`, gates publishing on `Tracking::OK`, and advertises all four required topics.
- [ ] Run the contract test; expect failure because the wrapper does not exist.
- [ ] Implement the node with cv_bridge, TF broadcasting, bounded path storage, tracked map-point publishing, clean ORB shutdown, and keyframe trajectory saving.
- [ ] Re-run contract tests. In a ROS environment run `catkin_make -DORB_SLAM3_ROOT=$HOME/third_party/ORB_SLAM3`; expect a successful build.
- [ ] Commit with `feat(task1): add ORB-SLAM3 monocular ROS wrapper`.

### Task 5: Configurations, launch, installation, and records

**Files:**
- Create: `catkin_ws/src/slam_practice/config/orbslam3/euroc_mono.yaml`
- Create: `catkin_ws/src/slam_practice/config/camera/euroc_cam0.yaml`
- Create: `catkin_ws/src/slam_practice/launch/task1_euroc_mono.launch`
- Create: `scripts/install_noetic_dependencies.sh`
- Create: `scripts/fetch_third_party.sh`
- Create: `scripts/download_euroc.sh`
- Create: `scripts/build_workspace.sh`
- Create: `docs/practice_records/01_task1_mono_slam.md`

**Interfaces:**
- Produces: one-command launch and a reproducible node-by-node record.

- [ ] Add configuration tests that parse camera YAML and XML launch, verify official EuRoC intrinsics, and confirm launch arguments expose dataset, vocabulary, viewer, playback rate, output path, and future custom config.
- [ ] Run tests; expect missing-file failures.
- [ ] Add official EuRoC intrinsics, launch all task-one nodes, write idempotent installation/download/build scripts, and document inputs, outputs, commands, expected observations, screenshot positions, failures, and replacement procedure.
- [ ] Run XML/JSON/config tests, `bash -n scripts/*.sh`, compileall, and all unit tests.
- [ ] Commit with `docs(task1): add launch workflow and practice record`.

