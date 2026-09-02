# Task 2 AirSim 双目视觉—惯导里程计设计

## 1. 当前阶段目标

只完成课程任务二的必做部分：

1. Windows 端运行 AirSim 预编译场景，不安装 Unreal Engine，不自行编译 AirSim 场景；
2. WSL2 Ubuntu 20.04 + ROS Noetic 运行 AirSim ROS Wrapper；
3. 从 AirSim 接入左右目 Scene 图像与 IMU；
4. 使用 VINS-Fusion 的双目 + IMU 模式估计位姿；
5. 在 RViz 实时显示估计位姿、轨迹、左目图像与特征点云；
6. 记录 VINS 估计轨迹与 AirSim 仿真真值，作为额外的定量评估依据。

EGO-Planner 指点避障不属于当前阶段。只有必做任务完成并验收后才决定是否继续。

## 2. 技术路线

选择“Windows 预编译 AirSim Blocks + WSL2 ROS Noetic + AirSim ROS Wrapper + VINS-Fusion”。

不采用以下路线：

- 不安装 Unreal Engine 并自行构建 AirSim 场景，因为必做任务只要求 AirSim 仿真环境及传感器接入，预编译场景已经满足需求；
- 不在当前阶段扩展现有 ORB-SLAM3 wrapper 到 STEREO_INERTIAL，因为仓库已有完整 VINS-Fusion 双目惯导链路，继续使用该实现风险更低、改动更少。

## 3. 系统架构

```text
Windows 11
AirSim Blocks
  ├─ Drone1 / left / Scene
  ├─ Drone1 / right / Scene
  ├─ Drone1 / Imu
  └─ odom_local_enu (仿真真值)
          │
          │ RPC 41451
          ▼
WSL2 Ubuntu 20.04 + ROS Noetic
  airsim_ros_pkgs / airsim_node
          │
          ├─ left image ─┐
          ├─ right image ├─> stereo_imu_relay ─> VINS-Fusion
          └─ IMU ────────┘                         │
                                                   ├─ odometry
                                                   ├─ path
                                                   └─ point cloud
                                                        │
                                             vins_output_adapter
                                                        │
                                                        ├─ RViz
                                                        └─ trajectory_recorder

  odom_local_enu ─────────────────────────────> airsim_gt_recorder
```

AirSim Ground Truth 只用于实验后评估，不进入 VINS-Fusion 输入，避免估计结果被真值污染。

## 4. AirSim 传感器配置

复用仓库 `airsim/settings_stereo_imu.json`：

- Vehicle: `Drone1`
- VehicleType: `SimpleFlight`
- 左相机位置：Y = -0.1 m
- 右相机位置：Y = +0.1 m
- 双目基线：0.20 m
- 图像分辨率：640 × 480
- 水平 FOV：90°
- IMU：启用

由针孔模型，90° 水平 FOV、640 px 宽度对应理论初始焦距约为 320 px。实际运行时需验证 AirSim ROS 输出的 CameraInfo 与 VINS 配置一致。

## 5. ROS 数据接口

AirSim 原始 Topic：

- `/airsim_node/Drone1/left/Scene`
- `/airsim_node/Drone1/right/Scene`
- `/airsim_node/Drone1/imu/Imu`
- `/airsim_node/Drone1/odom_local_enu`

`stereo_imu_relay` 规范化为：

- `/vins_fusion/cam0/image_raw`
- `/vins_fusion/cam1/image_raw`
- `/vins_fusion/imu`

VINS-Fusion 主要输出：

- `/vins_estimator/odometry`
- `/vins_estimator/path`
- `/vins_estimator/point_cloud`

项目稳定输出接口：

- `/slam_practice/vins/odometry`
- `/slam_practice/vins/path`
- TF `world -> body`

## 6. 时间与坐标系约束

- AirSim ROS Wrapper 使用 ENU 世界坐标；
- VINS 与后续 RViz 接口统一到项目的 `world` 坐标系；
- 左右图使用近似同步，允许最大约 3 ms 时间差；
- 成对左右图写入共同时间戳；
- IMU 保留 AirSim ROS Wrapper 的时间戳；
- 双目惯导具有可观测尺度，后续与 AirSim GT 比较时只做 SE(3) 对齐，不进行单目尺度修正。

## 7. 分阶段验收

### Stage A — Windows AirSim

通过条件：

- AirSim Blocks 能启动；
- `Drone1` 正常生成；
- `settings.json` 生效；
- AirSim RPC 41451 正常监听。

### Stage B — ROS-AirSim 接口

通过条件：

- WSL2 能连接 Windows AirSim；
- ROS 中能看到左目、右目、IMU 和 `odom_local_enu`；
- 图像分辨率、帧率和 IMU 频率合理；
- 左右图 Topic 持续更新。

### Stage C — VINS-Fusion

通过条件：

- VINS-Fusion 成功编译；
- 双目 + IMU 配置加载成功；
- 无长期初始化失败；
- 持续输出 odometry/path/point_cloud。

### Stage D — RViz 与结果保存

通过条件：

- RViz 实时显示位姿、路径、左目图像和特征地图；
- 保存 VINS TUM 轨迹；
- 保存 AirSim Ground Truth TUM 轨迹；
- 完成运行截图或录屏。

### Stage E — 可选附加评估

必做任务已经满足课程要求后，再使用 evo 对 VINS 与 AirSim GT 计算 ATE/RPE。该步骤用于增强实验报告，不影响“实时位姿估计可视化”的必做验收。

## 8. 故障定位顺序

严格按层排查，避免一次修改多个组件：

1. AirSim 是否单独正常运行；
2. WSL2 是否能访问 Windows RPC 41451；
3. AirSim ROS Wrapper 是否发布传感器 Topic；
4. stereo_imu_relay 是否正确同步并转发；
5. VINS 是否收到足够频率的数据并成功初始化；
6. RViz 是否正确订阅项目输出。

WSLg 若再次出现 `[WARN:COPY MODE]`，继续使用已验证的处理方式：Windows 执行 `wsl --shutdown`，重新启动 WSL 后先保持 `glxgears` 图形会话，再启动 RViz。

## 9. 当前实施边界

本阶段不做：

- Unreal Engine 安装；
- 自定义 Unreal 场景；
- EGO-Planner；
- 自动避障；
- 飞行控制器修改；
- ORB-SLAM3 Stereo-Inertial wrapper。

只有 Stage A–D 全部通过后，才讨论上述扩展。