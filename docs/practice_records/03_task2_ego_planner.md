# 实践记录三（选做）：EGO-Planner 指点避障飞行

## 1. 当前记录状态

| 检查层级 | 状态 | 说明 |
|---|---|---|
| 控制限幅、坐标变换单元测试 | 已完成 | 纯函数测试覆盖零误差、速度前馈、水平/垂直限幅和非法输入 |
| Launch 与接口静态检查 | 已完成 | 检查 VINS、深度、目标、轨迹指令和 AirSim 控制接口，确认未启动 EGO 自带仿真器 |
| WSL2 编译与 AirSim 实际避障 | 待执行 | 必须在含障碍的 AirSim 场景逐级验证 |
| 避障飞行录屏 | 待执行 | 录屏必须同时包含 AirSim 和 RViz 规划轨迹 |

## 2. 安全约束

控制节点启动后默认为 **DISABLED**，只有手动调用服务才输出非零速度：

```bash
rosservice call /ego_position_controller/set_enabled "data: true"
```

以下任一条件发生时自动发送零速度：

- 控制器没有解锁；
- VINS 位姿或 EGO 指令未到达；
- 任一输入超过 0.25 s 未更新；
- 输入包含 NaN/Inf；
- 节点关闭。

默认水平速度上限 1.5 m/s、垂直速度上限 0.8 m/s。紧急停止命令：

```bash
rosservice call /ego_position_controller/set_enabled "data: false"
```

## 3. 节点 1：`depth_pose_adapter`

### 做了什么

把 AirSim `DepthPlanner` 浮点深度图转换为 EGO-Planner 可接收的 `32FC1` 图像，并把同一时刻的 VINS 机体位姿通过固定外参转换成左相机光学系位姿。两条输出使用相同时间戳，供 EGO 地图模块同步。

| 类型 | 名称 |
|---|---|
| 订阅 | `/airsim_node/Drone1/left/DepthPlanner` |
| 订阅 | `/slam_practice/vins/odometry` |
| 发布 | `/ego_bridge/depth` |
| 发布 | `/ego_bridge/camera_pose` |

深度与 VINS 时间差超过 0.10 s 时丢弃该帧。相机位姿**不使用** `/airsim_node/Drone1/odom_local_enu` 真值。

```bash
rostopic hz /ego_bridge/depth
rostopic echo -n 1 /ego_bridge/camera_pose
```

## 4. 节点 2：`ego_planner_node`

### 做了什么

使用 EGO-Planner 上游 `advanced_param.xml` 启动规划器，地图大小为 40×40×5 m，相机参数与 AirSim 640×480、90° FOV 配置一致。里程计来自 VINS，深度与相机位姿来自适配节点。

| 类型 | 名称 |
|---|---|
| 订阅 | `/slam_practice/vins/odometry` |
| 订阅 | `/ego_bridge/depth` |
| 订阅 | `/ego_bridge/camera_pose` |
| 发布 | `/grid_map/occupancy` |
| 发布 | 规划 B 样条与可视化 Marker |

本 Launch 不包含 EGO-Planner 的 `simulator.xml`，AirSim 是唯一仿真环境。

## 5. 节点 3：`waypoint_generator`

### 做了什么

接收 RViz “2D Nav Goal” 工具的目标点，把 `/move_base_simple/goal` 送给 EGO 规划状态机。目标使用 `world`/ENU 坐标系。

| 类型 | 名称 |
|---|---|
| 订阅 | `/move_base_simple/goal` |
| 订阅 | `/slam_practice/vins/odometry` |
| 输出 | EGO 目标触发接口 |

首次测试时应选择无人机前方 2–3 m、与当前高度接近的空旷目标，不要直接选择障碍物后方远点。

## 6. 节点 4：`traj_server`

### 做了什么

采样规划器生成的 B 样条，输出 EGO `quadrotor_msgs/PositionCommand`：

| 类型 | 名称 |
|---|---|
| 订阅 | EGO 规划轨迹 |
| 订阅 | `/slam_practice/vins/odometry` |
| 发布 | `/planning/pos_cmd` |

解锁控制前先确认位置、速度、yaw 字段均为有限值且频率稳定：

```bash
rostopic hz /planning/pos_cmd
rostopic echo -n 1 /planning/pos_cmd
```

## 7. 节点 5：`ego_position_controller`

### 做了什么

根据 PositionCommand 目标位置、前馈速度和 VINS 当前位姿计算：

```text
v_cmd = v_feedforward + kp × (p_target - p_current)
```

随后独立限制水平速度模长、垂直速度和 yaw rate，并发布 AirSim 世界坐标系速度命令。

| 类型 | 名称 |
|---|---|
| 订阅 | `/planning/pos_cmd` |
| 订阅 | `/slam_practice/vins/odometry` |
| 发布 | `/airsim_node/vel_cmd_world_frame` |
| 服务 | `/ego_position_controller/set_enabled` |

## 8. 完整构建与启动

```bash
source /opt/ros/noetic/setup.bash
./scripts/install_noetic_dependencies.sh
./scripts/fetch_third_party.sh
./scripts/build_ego_workspace.sh

source third_party/AirSim/ros/devel/setup.bash
source catkin_ws/devel/setup.bash
export WSL_HOST_IP=$(awk '/^nameserver/{print $2; exit}' /etc/resolv.conf)
roslaunch slam_practice task2_ego_airsim.launch host:=$WSL_HOST_IP
```

另开终端：

```bash
source /opt/ros/noetic/setup.bash
source third_party/AirSim/ros/devel/setup.bash
source catkin_ws/devel/setup.bash
./scripts/check_ego_topics.sh
```

## 9. 分阶段验证记录

### 阶段 A：输入与悬停

1. 确认 VINS 已初始化并持续输出。
2. 检查深度图中近障碍距离符合 AirSim 场景。
3. 不发送目标，不解锁控制，确认无人机没有收到非零速度。

**需插入证据：** `ego_01_depth_pose_topics.png`。

### 阶段 B：空场短距离目标

1. 在 RViz 发送 2–3 m 空旷目标。
2. 检查 `/planning/pos_cmd` 和规划轨迹 Marker。
3. 确认方向与 ENU 坐标一致后解锁控制。
4. 到达目标后立即禁用控制。

**需插入证据：** `ego_02_empty_world_goal.png`。

### 阶段 C：单个静态障碍

1. 在起点和目标间放置一个明显障碍。
2. 先保持禁用，只观察规划轨迹是否绕开占据栅格。
3. 确认安全距离后以默认低速解锁。

**需插入证据：** `ego_03_single_obstacle.png`。

### 阶段 D：多障碍与最终录屏

录屏必须同时展示：

- AirSim 无人机和障碍环境；
- RViz 占据地图、VINS 路径、目标点和规划轨迹；
- 终端中的解锁/停止操作；
- 无人机绕障并进入目标邻域的过程。

建议录屏文件名：`task2_ego_airsim_demo.mp4`。大体积录屏不提交 Git，报告中给出文件或网盘位置。

## 10. 实验结果表

| 场景 | 目标距离/m | 最小障碍距离/m | 到达时间/s | 是否触发重规划 | 是否成功 |
|---|---:|---:|---:|---|---|
| 空场短距离 | 待运行 | 不适用 | 待运行 | 待运行 | 待运行 |
| 单静态障碍 | 待运行 | 待运行 | 待运行 | 待运行 | 待运行 |
| 多障碍 | 待运行 | 待运行 | 待运行 | 待运行 | 待运行 |

## 11. 参考实现

- [EGO-Planner 官方仓库](https://github.com/ZJU-FAST-Lab/ego-planner)
- [EGO-Planner 上游 AirSim 外接时复用的参数入口](https://github.com/ZJU-FAST-Lab/ego-planner/blob/master/src/planner/plan_manage/launch/advanced_param.xml)
- [AirSim ROS Wrapper](https://github.com/microsoft/AirSim/blob/main/docs/airsim_ros_pkgs.md)
