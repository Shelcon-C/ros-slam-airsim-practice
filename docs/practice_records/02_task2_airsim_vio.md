# 实践记录二：AirSim 双目视觉—惯导里程计集成

## 1. 当前记录状态

| 检查层级 | 状态 | 说明 |
|---|---|---|
| 双目、IMU、外参与 Launch 静态检查 | 已完成 | JSON、YAML、XML 和接口契约均由仓库测试验证 |
| Ubuntu 20.04 + ROS Noetic 编译 | 待在 WSL2 执行 | AirSim 官方 Wrapper 支持 Noetic；VINS-Fusion 上游说明主要列出较早的 ROS 版本，需以本机编译结果为准 |
| Windows AirSim 与 WSL2 联调 | 待执行 | 需要启动 Windows 端 Unreal/AirSim 场景并放行 RPC 端口 |
| VINS 实时位姿与地图 | 待执行 | 运行后补 RViz、Topic 频率和终端截图，不填写模拟结果 |

## 2. 系统数据流

```text
Windows AirSim
  ├─ left/Scene ─┐
  ├─ right/Scene ├─> airsim_node ─> stereo_imu_relay ─> vins_estimator
  └─ imu/Imu ────┘                                      │
                                                        ├─> vins_output_adapter ─> RViz
  odom_local_enu ─> airsim_gt_recorder                  └─> trajectory_recorder
```

真值 `odom_local_enu` 只进入真值记录节点，**不进入 VINS-Fusion**，避免估计结果被真值污染。

## 3. 节点 1：`airsim_node`

### 做了什么

使用 AirSim 官方 `airsim_ros_pkgs`，连接 Windows 主机的 RPC 端口 41451。Launch 把世界坐标配置为 ENU，并发布双目 Scene 图像、IMU 与本地真值里程计。

| 类型 | 名称 |
|---|---|
| 参数 | `host_ip`、`host_port=41451`、`coordinate_system_enu=true` |
| 发布 | `/airsim_node/Drone1/left/Scene` |
| 发布 | `/airsim_node/Drone1/right/Scene` |
| 发布 | `/airsim_node/Drone1/imu/Imu` |
| 发布 | `/airsim_node/Drone1/odom_local_enu` |

对应文件：

- `airsim/settings_stereo_imu.json`
- `catkin_ws/src/slam_practice/launch/airsim_noetic_wsl.launch`

配置中双目分辨率为 640×480、水平 FOV 为 90°、基线为 0.20 m。由针孔模型 `fx=(width/2)/tan(FOV/2)` 得到初始 `fx=fy=320 px`。

## 4. 节点 2：`stereo_imu_relay`

### 做了什么

订阅 AirSim 左右图，使用 3 ms 容差近似同步；将图像统一转换为 `mono8`，给成对图像写入共同时间戳，并转发 ENU 配置下的 IMU。这样 VINS-Fusion 不依赖 AirSim 原始 Topic 命名。

| 类型 | 名称 |
|---|---|
| 订阅 | `/airsim_node/Drone1/left/Scene` |
| 订阅 | `/airsim_node/Drone1/right/Scene` |
| 订阅 | `/airsim_node/Drone1/imu/Imu` |
| 发布 | `/vins_fusion/cam0/image_raw` |
| 发布 | `/vins_fusion/cam1/image_raw` |
| 发布 | `/vins_fusion/imu` |

同步失败、图像转换失败时节点会限频告警并丢弃该帧，避免不配对图像进入估计器。

## 5. 节点 3：`vins_estimator`

### 做了什么

以 VINS-Fusion 双目+IMU模式运行。配置明确启用 `imu: 1`、`num_of_cam: 2`，并提供两台相机的针孔参数和 `body_T_cam0/body_T_cam1` 外参。

| 类型 | 名称 |
|---|---|
| 订阅 | `/vins_fusion/cam0/image_raw` |
| 订阅 | `/vins_fusion/cam1/image_raw` |
| 订阅 | `/vins_fusion/imu` |
| 发布 | `/vins_estimator/odometry` |
| 发布 | `/vins_estimator/path` |
| 发布 | `/vins_estimator/point_cloud` |

对应文件：

- `config/vins/airsim_stereo_imu.yaml`
- `config/vins/airsim_cam0.yaml`
- `config/vins/airsim_cam1.yaml`

如果初始化长期失败，按顺序检查：IMU 频率、左右图时间差、图像纹理、机体是否有充分的平移与转动、外参方向和重力方向。

## 6. 节点 4：`vins_output_adapter`

### 做了什么

把 VINS 的私有命名空间输出转换为项目稳定接口，同时维护有界路径并发布 `world → body` TF，供 RViz 和后续 EGO-Planner 使用。

| 类型 | 名称 |
|---|---|
| 订阅 | `/vins_estimator/odometry` |
| 发布 | `/slam_practice/vins/odometry` |
| 发布 | `/slam_practice/vins/path` |
| TF | `world → body` |

## 7. 节点 5：`trajectory_recorder`

### 做了什么

订阅适配后的 VINS 里程计，实时写入 TUM 格式轨迹：

```text
timestamp tx ty tz qx qy qz qw
```

默认输出为 `~/slam_results/task2/vins_estimate.tum`。重复或倒序时间戳会被跳过。

## 8. 节点 6：`airsim_gt_recorder`

### 做了什么

独立订阅 AirSim ENU 真值并写入 `~/slam_results/task2/airsim_groundtruth.tum`。此节点只为实验后评估服务，不与 VINS 输入相连。

运行完成后可复用任务一评估工具：

```bash
source .venv-evo/bin/activate
rosrun slam_practice evaluate_trajectory.py \
  ~/slam_results/task2/airsim_groundtruth.tum \
  ~/slam_results/task2/vins_estimate.tum \
  ~/slam_results/task2/evaluation --sensor stereo-inertial
```

双目—惯导轨迹具有尺度观测，因此评估只执行 SE(3) 对齐，不启用单目尺度校正。

## 9. RViz 实时可视化

`rviz/task2_vins.rviz` 默认显示：

- `/slam_practice/vins/path`：绿色估计轨迹；
- `/slam_practice/vins/odometry`：当前机体位姿；
- `/vins_estimator/point_cloud`：VINS 特征地图；
- `/vins_fusion/cam0/image_raw`：左目灰度图。

**需插入证据：**

1. `task2_01_airsim_stereo.png`：AirSim 场景与左右目画面；
2. `task2_02_topic_frequency.png`：左右图、IMU 和里程计频率；
3. `task2_03_vins_initialization.png`：VINS 初始化成功终端；
4. `task2_04_rviz_pose_map.png`：RViz 轨迹、位姿和点云；
5. `task2_05_trajectory_evaluation.png`：ATE/RPE 图和真实指标。

## 10. WSL2 完整运行步骤

### 10.1 Windows 端

把仓库的 `airsim/settings_stereo_imu.json` 复制为：

```text
%USERPROFILE%\Documents\AirSim\settings.json
```

启动带 AirSim 插件的 Unreal 场景，确认 `Drone1`、左右相机和 IMU 已创建。

### 10.2 WSL2 端构建

```bash
source /opt/ros/noetic/setup.bash
./scripts/install_noetic_dependencies.sh
./scripts/fetch_third_party.sh
./scripts/build_task2_workspace.sh
```

### 10.3 检查 Windows 主机连接

```bash
./scripts/detect_wsl_host.sh
export WSL_HOST_IP=$(awk '/^nameserver/{print $2; exit}' /etc/resolv.conf)
```

如果端口不通，确认 AirSim 已启动，并在 Windows 防火墙中允许该 AirSim/Unreal 程序接收 WSL2 连接。

### 10.4 启动完整链路

```bash
source /opt/ros/noetic/setup.bash
source third_party/AirSim/ros/devel/setup.bash
source catkin_ws/devel/setup.bash
mkdir -p ~/slam_results/task2
roslaunch slam_practice task2_airsim_vins.launch host:=$WSL_HOST_IP
```

另开终端执行：

```bash
source /opt/ros/noetic/setup.bash
source third_party/AirSim/ros/devel/setup.bash
source catkin_ws/devel/setup.bash
./scripts/check_task2_topics.sh
```

## 11. 实验结果表

| 场景 | 左/右图 Hz | IMU Hz | VINS 位姿 Hz | ATE RMSE/m | RPE 平移 RMSE/m | RPE 旋转 RMSE/deg |
|---|---:|---:|---:|---:|---:|---:|
| AirSim 默认场景 | 待运行 | 待运行 | 待运行 | 待运行 | 待运行 | 待运行 |

## 12. 参考实现

- [AirSim ROS Wrapper 文档](https://github.com/microsoft/AirSim/blob/main/docs/airsim_ros_pkgs.md)
- [VINS-Fusion 官方仓库](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
