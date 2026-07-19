# 轨迹精度对比与可视化结果报告

> 本文件是可提交报告模板。`待运行` 必须由真实命令输出替换，并附对应终端/RViz/evo 截图；不得填写估算值或示例值。

## 1. 实验环境

| 项目 | 实际信息 |
|---|---|
| Windows 版本 | 待填写 |
| WSL 版本 | 待填写 |
| Ubuntu | 20.04 |
| ROS | Noetic |
| CPU/GPU | 待填写 |
| ORB-SLAM3 commit | 待填写：`git -C third_party/ORB_SLAM3 rev-parse HEAD` |
| VINS-Fusion commit | 待填写：`git -C third_party/VINS-Fusion rev-parse HEAD` |
| AirSim commit | 待填写：`git -C third_party/AirSim rev-parse HEAD` |

## 2. 相机标定结果

### 2.1 EuRoC 当前验证数据

| 参数 | 数值 |
|---|---:|
| 分辨率 | 752×480 |
| fx | 458.654 |
| fy | 457.296 |
| cx | 367.215 |
| cy | 248.375 |

### 2.2 后续指定视频

| 参数 | 实测值 |
|---|---:|
| 有效棋盘格视角 | 待标定 |
| OpenCV RMS | 待标定 |
| 平均重投影误差/pixel | 待标定 |
| fx/fy/cx/cy | 待标定 |

附图：棋盘格原图、角点检测图、标定报告 JSON。

## 3. 任务一：ORB-SLAM3 单目结果

### 3.1 轨迹评估方法

- 真值和估计统一为 TUM `timestamp tx ty tz qx qy qz qw`；
- evo 先按时间关联；
- 单目轨迹使用 `--align --correct_scale`；
- ATE 衡量全局轨迹一致性；
- RPE 以相邻帧（`--delta 1 --delta_unit f`）衡量局部运动误差。

### 3.2 精度表

| 数据序列/算法 | 有效轨迹帧 | ATE RMSE/m | ATE mean/m | RPE 平移 RMSE/m | RPE 旋转 RMSE/deg |
|---|---:|---:|---:|---:|---:|
| EuRoC MH_01_easy / ORB-SLAM3 mono | 待运行 | 待运行 | 待运行 | 待运行 | 待运行 |
| 指定视频 / ORB-SLAM3 mono | 待收到数据 | 待收到数据 | 待收到数据 | 待收到数据 | 待收到数据 |

### 3.3 可视化证据

- ORB-SLAM3 Viewer 稀疏地图：待插入；
- RViz 相机路径与地图点：待插入；
- `ate_plot.pdf`：运行工具自动生成；
- `rpe_translation_plot.pdf` 与 `rpe_rotation_plot.pdf`：运行工具自动生成；
- `metrics.csv` 与四份命令日志：作为数值溯源附件。

### 3.4 结果分析

待根据真实曲线分析初始化段、快速转动、纹理缺失、运动模糊和闭环对全局误差的影响。

## 4. 任务二：AirSim VINS-Fusion 结果

| 场景/算法 | 图像 Hz | IMU Hz | 位姿 Hz | ATE RMSE/m | RPE 平移 RMSE/m | RPE 旋转 RMSE/deg |
|---|---:|---:|---:|---:|---:|---:|
| AirSim / VINS-Fusion stereo+IMU | 待运行 | 待运行 | 待运行 | 待运行 | 待运行 | 待运行 |

双目惯导评估只使用 `--align`，不使用单目尺度校正。附图应包含：AirSim 双目画面、VINS 初始化终端、RViz 位姿/路径/点云、ATE/RPE 图。

## 5. 定位精度对比结论

| 对比项 | ORB-SLAM3 单目 | VINS-Fusion 双目+IMU |
|---|---|---|
| 尺度来源 | 轨迹评估时校正尺度 | 双目基线与 IMU 可观测尺度 |
| 输入 | 单目图像 | 同步双目 + IMU |
| ATE/RPE 实测优劣 | 待运行后结论 | 待运行后结论 |
| 初始化/快速运动表现 | 待分析 | 待分析 |

最终结论必须引用本报告第 3、4 节的真实指标和可视化，不以主观观感代替数值。

## 6. 选做避障结果

| 场景 | 目标距离/m | 最小障碍距离/m | 到达时间/s | 重规划次数 | 是否成功 |
|---|---:|---:|---:|---:|---|
| 空场短距离 | 待运行 | 不适用 | 待运行 | 待运行 | 待运行 |
| 单静态障碍 | 待运行 | 待运行 | 待运行 | 待运行 | 待运行 |
| 多障碍 | 待运行 | 待运行 | 待运行 | 待运行 | 待运行 |

录屏附件：`task2_ego_airsim_demo.mp4`（待生成）。
