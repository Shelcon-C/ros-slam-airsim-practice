# ROS SLAM 与 AirSim 实践

面向 **Ubuntu 20.04 + ROS Noetic + WSL2** 的完整实践仓库，包含：

1. EuRoC 左目图像 + ORB-SLAM3 单目定位、相机标定、TUM 轨迹记录、ATE/RPE 评估与 evo 可视化；
2. Windows AirSim + WSL2 ROS Wrapper + VINS-Fusion 双目视觉—惯导里程计与 RViz；
3. 选做 EGO-Planner 指点避障、深度/位姿桥接及带看门狗的 AirSim 速度控制。

> 当前仓库已完成代码、配置和静态自动测试。需要 GPU/GUI/ROS 的编译、真实轨迹、ATE/RPE 数字和录屏必须在你的 WSL2 + Windows AirSim 环境运行后生成；仓库没有填写伪造实验结果。

## 实践记录与报告

- [任务一：单目 SLAM 逐节点实践记录](docs/practice_records/01_task1_mono_slam.md)
- [任务二：AirSim 双目 VIO 逐节点实践记录](docs/practice_records/02_task2_airsim_vio.md)
- [选做：EGO-Planner 避障逐节点实践记录](docs/practice_records/03_task2_ego_planner.md)
- [轨迹精度与可视化结果报告模板](docs/practice_records/04_results_report_template.md)

## 代码文件索引

| 节点/工具 | 代码文件 | 作用 |
|---|---|---|
| EuRoC 图像发布 | `euroc_mono_publisher.py` | 按数据集时间戳发布左目图和 CameraInfo |
| ORB-SLAM3 单目封装 | `mono_node.cpp` | 跟踪、位姿/路径/地图点/TF 发布 |
| 通用轨迹记录 | `trajectory_recorder.py` | PoseStamped/Odometry 转 TUM 文件 |
| 相机标定 | `calibrate_camera.py` | 视频棋盘格标定、内参 YAML 与误差报告 |
| EuRoC 真值转换 | `euroc_groundtruth_to_tum.py` | 官方 CSV 转 TUM |
| ATE/RPE 评估 | `evaluate_trajectory.py` | evo 对齐、指标、PDF 图与 CSV |
| AirSim 双目/IMU桥接 | `stereo_imu_relay.py` | 3 ms 双目同步、灰度化、Topic 规范化 |
| VINS 输出适配 | `vins_output_adapter.py` | 稳定 odometry/path/TF 接口 |
| AirSim 真值记录 | `airsim_gt_recorder.py` | 独立记录 ENU 真值，仅用于评估 |
| EGO 深度/位姿适配 | `depth_pose_adapter.py` | DepthPlanner + VINS 转 EGO 同步输入 |
| EGO 飞行控制 | `ego_position_controller.py` | PositionCommand 转限幅 VelCmd，默认锁定 |

ROS Python 节点位于 `catkin_ws/src/slam_practice/scripts/`；可测试公共逻辑位于 `catkin_ws/src/slam_practice/src/slam_practice/`。

## 仓库结构

```text
.
├── airsim/                         # AirSim settings.json 模板
├── catkin_ws/src/
│   ├── orbslam3_ros/               # ORB-SLAM3 C++ ROS Wrapper
│   └── slam_practice/
│       ├── config/                 # 相机、ORB-SLAM3、VINS 配置
│       ├── launch/                 # 三条实验入口
│       ├── rviz/                   # 位姿、地图、规划可视化
│       ├── scripts/                # ROS 节点与离线工具
│       ├── src/slam_practice/      # 数据、轨迹、几何、控制纯逻辑
│       └── tests/                  # 自动测试
├── docs/practice_records/          # 每个节点做了什么、命令、证据占位
└── scripts/                        # 安装、下载、构建与联调检查
```

数据集、第三方源码、编译目录、真实结果和大体积录屏由 `.gitignore` 排除。

## 第一次准备

```bash
git clone https://github.com/Shelcon-C/ros-slam-airsim-practice.git
cd ros-slam-airsim-practice
source /opt/ros/noetic/setup.bash
./scripts/install_noetic_dependencies.sh
./scripts/fetch_third_party.sh
```

脚本会获取 [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)、[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)、[AirSim](https://github.com/microsoft/AirSim) 和 [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)；这些上游源码不会复制进本仓库。

## 任务一快速运行

```bash
source /opt/ros/noetic/setup.bash
./scripts/download_euroc.sh MH_01_easy
./scripts/build_workspace.sh
source catkin_ws/devel/setup.bash
mkdir -p ~/slam_results/task1

roslaunch slam_practice task1_euroc_mono.launch \
  dataset_root:=$PWD/datasets/euroc/MH_01_easy \
  vocabulary_path:=$PWD/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt
```

另开终端转换真值并评估：

```bash
source .venv-evo/bin/activate
source catkin_ws/devel/setup.bash
rosrun slam_practice euroc_groundtruth_to_tum.py \
  datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
  ~/slam_results/task1/mh01_groundtruth.tum \
  --sensor-yaml datasets/euroc/MH_01_easy/mav0/cam0/sensor.yaml

rosrun slam_practice evaluate_trajectory.py \
  ~/slam_results/task1/mh01_groundtruth.tum \
  ~/slam_results/task1/orbslam3_mh01.tum \
  ~/slam_results/task1/evaluation --sensor monocular
```

`--sensor-yaml` 使用 EuRoC 的 `T_BS` 把 IMU/body 真值转换到 cam0，相机估计与真值才处于相同刚体参考点。输出含 `ate_results.zip`、平移/旋转两份 RPE 结果包、三张 PDF 图、`metrics.csv` 和四份命令日志。

Ubuntu 20.04 自带 Python 3.8，因此安装脚本在隔离环境 `.venv-evo` 中固定使用兼容的 `evo==1.30.6`；当前最新版 evo 要求更高版本 Python，不直接安装进 ROS 系统 Python。

## 后续替换指定单目视频

当前先用 EuRoC `cam0` 左目完成流程。收到指定视频后：

1. 使用 `calibrate_camera.py` 对同一相机完成棋盘格标定；
2. 把视频抽帧并生成严格递增的 `timestamp,filename` 清单；
3. 替换 `camera_yaml`、`orb_settings` 和数据路径；
4. 保留 ORB Wrapper、轨迹记录和评估节点不变；
5. 有真值才计算 ATE/RPE，无真值不能声称绝对定位精度。

数据层与算法层已解耦，详见任务一实践记录的“后续替换指定视频”。

## 任务二快速运行

先将 `airsim/settings_stereo_imu.json` 复制到 Windows：

```text
%USERPROFILE%\Documents\AirSim\settings.json
```

启动 Windows AirSim 场景后，在 WSL2 执行：

```bash
source /opt/ros/noetic/setup.bash
./scripts/build_task2_workspace.sh
./scripts/detect_wsl_host.sh
export WSL_HOST_IP=$(awk '/^nameserver/{print $2; exit}' /etc/resolv.conf)
source third_party/AirSim/ros/devel/setup.bash
source catkin_ws/devel/setup.bash
roslaunch slam_practice task2_airsim_vins.launch host:=$WSL_HOST_IP
```

另开终端运行 `./scripts/check_task2_topics.sh`。RViz 应显示 VINS 位姿、路径、左目图和稀疏特征地图。

## 选做避障快速运行

```bash
source /opt/ros/noetic/setup.bash
./scripts/build_ego_workspace.sh
source third_party/AirSim/ros/devel/setup.bash
source catkin_ws/devel/setup.bash
roslaunch slam_practice task2_ego_airsim.launch host:=$WSL_HOST_IP
```

执行 `./scripts/check_ego_topics.sh`，在 RViz 发送短距离空旷目标并确认轨迹正确后，才手动解锁：

```bash
rosservice call /ego_position_controller/set_enabled "data: true"
```

紧急停止：

```bash
rosservice call /ego_position_controller/set_enabled "data: false"
```

## 静态自动验证

无需 ROS 即可运行公共逻辑和配置契约测试：

```bash
./scripts/run_static_tests.sh
```

该测试不等价于 ROS 编译和 AirSim 实际运行。真实验收状态始终记录在三份实践记录顶部。

## 许可与参考

本仓库自编代码采用 MIT License。第三方算法分别受各自上游许可证约束。本工程主要参考：

- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [AirSim ROS Wrapper 文档](https://github.com/microsoft/AirSim/blob/main/docs/airsim_ros_pkgs.md)
- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)
