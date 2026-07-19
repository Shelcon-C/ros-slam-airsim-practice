# Optional EGO-Planner AirSim Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Feed AirSim depth and VINS pose to EGO-Planner, accept RViz target points, and convert EGO position commands into bounded AirSim velocity commands.

**Architecture:** A perception adapter republishes the AirSim DepthPlanner image with the current VINS-derived camera pose. EGO-Planner runs without its bundled simulator. A feedback controller combines planned feed-forward velocity with VINS position error and publishes `airsim_ros_pkgs/VelCmd` in ENU.

**Tech Stack:** ROS Noetic, EGO-Planner, AirSim ROS Wrapper, VINS-Fusion, Python 3, NumPy, RViz.

## Global Constraints

- Planner odometry comes from VINS-Fusion, not AirSim truth.
- Control starts disabled and requires an explicit enable service.
- Horizontal speed is limited to 1.5 m/s and vertical speed to 0.8 m/s by default.
- Stale odometry or command data produces a zero-velocity command.

---

### Task 1: Test-driven bounded controller core

**Files:**
- Create: `catkin_ws/src/slam_practice/src/slam_practice/control.py`
- Test: `catkin_ws/src/slam_practice/tests/test_control.py`

**Interfaces:**
- Produces: `compute_velocity_command(current, target, feedforward, kp, max_xy, max_z)`.

- [ ] Write tests for zero error, proportional feedback, horizontal norm saturation, independent vertical saturation, and non-finite input rejection.
- [ ] Run test; expect missing-module failure.
- [ ] Implement the minimal deterministic controller.
- [ ] Re-run all tests; expect pass.
- [ ] Commit with `feat(ego): add bounded position controller core`.

### Task 2: Perception and control bridge nodes

**Files:**
- Create: `catkin_ws/src/slam_practice/scripts/depth_pose_adapter.py`
- Create: `catkin_ws/src/slam_practice/scripts/ego_position_controller.py`

**Interfaces:**
- Consumes: AirSim DepthPlanner image, VINS odometry, and `/planning/pos_cmd`.
- Produces: `/ego_bridge/depth`, `/ego_bridge/camera_pose`, and `/airsim_node/vel_cmd_world_frame`.

- [ ] Add contract tests for `32FC1` depth, VINS-only pose input, enable service, watchdog timeout, VelocityCommand limits, and AirSim VelCmd output.
- [ ] Run tests; expect absent-file failure.
- [ ] Implement the adapter and controller; default to disabled and publish zero velocity on timeout or shutdown.
- [ ] Run compileall and all tests.
- [ ] Commit with `feat(ego): add AirSim perception and control bridges`.

### Task 3: Planner launch and practice record

**Files:**
- Create: `catkin_ws/src/slam_practice/launch/task2_ego_airsim.launch`
- Create: `catkin_ws/src/slam_practice/rviz/task2_ego.rviz`
- Create: `scripts/check_ego_topics.sh`
- Create: `docs/practice_records/03_task2_ego_planner.md`

**Interfaces:**
- Produces: one launch entry for EGO without its bundled simulator and a complete demonstration checklist.

- [ ] Add XML and contract tests asserting `/move_base_simple/goal`, `/vins_estimator/odometry`, `/ego_bridge/depth`, `/ego_bridge/camera_pose`, `/planning/pos_cmd`, and no bundled simulator include.
- [ ] Run tests; expect missing-file failures.
- [ ] Base the launch arguments on EGO-Planner `advanced_param.xml`, start `traj_server` and `waypoint_generator`, then start both bridge nodes and RViz.
- [ ] Document staged validation: hover, empty-world goal, static obstacle, multi-obstacle route, emergency disable, and final recording.
- [ ] Run all static checks and unit tests.
- [ ] Commit with `docs(ego): add planner launch and demonstration record`.

