# 实践记录一：单目视频 SLAM 精度评估

## 1. 当前记录状态

| 检查层级 | 状态 | 说明 |
|---|---|---|
| 代码与配置静态检查 | 已完成 | 单元测试、Python 语法、XML 与 Shell 语法由仓库测试验证 |
| Ubuntu 20.04 + ROS Noetic 编译 | 待在你的 WSL2 执行 | 需要安装 ROS、Pangolin 与 ORB-SLAM3 |
| EuRoC MH_01_easy 实际运行 | 待执行 | 运行后补充终端与 RViz 截图 |
| ATE/RPE 数值 | 待执行 | 不填写示例假数据 |

## 2. 节点 1：`euroc_mono_publisher`

### 做了什么

读取 `mav0/cam0/data.csv`，检查时间戳严格递增且每张图像存在，然后按原始帧间隔发布灰度图。相机标定参数由独立 YAML 发布为 CameraInfo，因此后续替换指定视频时不需要修改节点代码。

发布器最多等待 60 秒，直到 ORB-SLAM3 完成词典加载并真正订阅图像，防止序列开头在算法启动阶段丢失。

### 输入与输出

| 类型 | 名称 |
|---|---|
| 参数 | `~dataset_root`、`~camera_yaml`、`~playback_rate`、`~start_offset`、`~duration` |
| 发布 | `/camera/mono/image_raw` |
| 发布 | `/camera/mono/camera_info` |

### 检查命令

```bash
rostopic hz /camera/mono/image_raw
rostopic echo -n 1 /camera/mono/camera_info
rqt_image_view /camera/mono/image_raw
```

预期图像频率约为 `20 × playback_rate` Hz，分辨率为 752×480。

**需插入证据：** `task1_01_input_image.png`，包含 rqt_image_view 和 `rostopic hz`。

## 3. 节点 2：`orbslam3_mono_node`

### 做了什么

封装 ORB-SLAM3 `TrackMonocular`。算法返回 `Tcw`，节点求逆得到相机在世界中的 `Twc`，仅在跟踪状态为 OK 时发布结果。为兼顾算法自带 Pangolin 地图窗口和 ROS 可视化，同时发布位姿、里程计、路径与当前跟踪地图点。

| 类型 | 名称 |
|---|---|
| 订阅 | `/camera/mono/image_raw` |
| 发布 | `/orbslam3/pose` |
| 发布 | `/orbslam3/odometry` |
| 发布 | `/orbslam3/path` |
| 发布 | `/orbslam3/tracked_points` |
| TF | `map → camera_mono_optical_frame` |

```bash
rosnode info /orbslam3_mono
rostopic hz /orbslam3/pose
rostopic echo -n 1 /orbslam3/odometry
```

**需插入证据：** `task1_02_orb_viewer.png` 与 `task1_03_rviz_path_map.png`。

## 4. 节点 3：`trajectory_recorder`

### 做了什么

订阅 ORB-SLAM3 的 PoseStamped，按 TUM 格式实时写入：

```text
timestamp tx ty tz qx qy qz qw
```

若收到重复或倒序时间戳，节点跳过该条记录，防止 evo 关联失败。

```bash
head -n 3 ~/slam_results/task1/orbslam3_mh01.tum
awk 'NF != 8 {print NR, $0}' ~/slam_results/task1/orbslam3_mh01.tum
```

## 5. 工具 1：相机标定

EuRoC 阶段使用数据集官方内参。收到老师的指定视频后，需要使用同一相机拍摄棋盘格，再运行：

```bash
rosrun slam_practice calibrate_camera.py \
  --video calibration.mp4 \
  --board-cols 9 --board-rows 6 --square-size 0.024 \
  --fps 30 --output custom_mono.yaml --report calibration_report.json
```

报告必须记录有效棋盘格视角数量、OpenCV RMS 和平均重投影误差。常见失败包括棋盘格内角点数量填反、有效视角少于 10、只在同一角度拍摄以及运动模糊。

**需插入证据：** 棋盘格原图、角点检测图与 `calibration_report.json`。

## 6. 工具 2：真值转换与 ATE/RPE

```bash
rosrun slam_practice euroc_groundtruth_to_tum.py \
  ~/datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
  ~/slam_results/task1/mh01_groundtruth.tum \
  --sensor-yaml ~/datasets/euroc/MH_01_easy/mav0/cam0/sensor.yaml

rosrun slam_practice evaluate_trajectory.py \
  ~/slam_results/task1/mh01_groundtruth.tum \
  ~/slam_results/task1/orbslam3_mh01.tum \
  ~/slam_results/task1/evaluation --sensor monocular
```

EuRoC 真值原点在 IMU/body，`--sensor-yaml` 使用官方 `T_BS` 把它转换到 cam0，避免相机—IMU 杆臂误差混入 ATE/RPE。单目轨迹没有确定的绝对尺度，因此评估强制执行轨迹对齐和尺度校正，并分别计算 ATE 平移、RPE 平移和 RPE 旋转角。输出包括三份结果包、三张 PDF 图、`metrics.csv` 和四份命令日志。

运行上述评估命令前先执行 `source .venv-evo/bin/activate`（或在仓库外按绝对路径激活）。安装脚本固定 `evo==1.30.6`，以兼容 Ubuntu 20.04 的 Python 3.8。

## 7. 完整运行顺序

```bash
source /opt/ros/noetic/setup.bash
./scripts/install_noetic_dependencies.sh
./scripts/fetch_third_party.sh
./scripts/download_euroc.sh MH_01_easy
./scripts/build_workspace.sh
source catkin_ws/devel/setup.bash
mkdir -p ~/slam_results/task1

roslaunch slam_practice task1_euroc_mono.launch \
  dataset_root:=$PWD/datasets/euroc/MH_01_easy \
  vocabulary_path:=$PWD/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt
```

## 8. 后续替换指定视频

1. 用 `calibrate_camera.py` 获得相机 YAML。
2. 把视频抽帧并生成 `timestamp,filename` 清单，或新增视频数据适配器。
3. 启动时替换 `dataset_root`、`camera_yaml` 和 `orb_settings`。
4. 如果没有真值，只能展示轨迹与重投影等间接结果，不能合法计算 ATE/RPE。
5. 如果提供了真值，统一转换为 TUM 格式后复用现有评估工具。

## 9. 实验结果表

| 序列 | 帧数 | 成功跟踪时长 | ATE RMSE/m | RPE 平移 RMSE/m | RPE 旋转 RMSE/deg |
|---|---:|---:|---:|---:|---:|
| MH_01_easy | 待运行 | 待运行 | 待运行 | 待运行 | 待运行 |
