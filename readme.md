# LiDAR-IMU 标定与 FAST-LIO2 建图

![License](https://img.shields.io/badge/License-MIT-blue.svg)
![ROS](https://img.shields.io/badge/ROS-Noetic-22314E.svg)
![C++](https://img.shields.io/badge/C%2B%2B-14-00599C.svg)
![Python](https://img.shields.io/badge/Python-3.8-3776AB.svg)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420.svg)
![Platform](https://img.shields.io/badge/Platform-Linux-yellow.svg)
![LiDAR](https://img.shields.io/badge/LiDAR-Livox%20MID360%20%7C%20RoboSense16-brightgreen.svg)
![IMU](https://img.shields.io/badge/IMU-Hipnuc%20%E8%B6%85%E6%A0%B8-orange.svg)

> 基于 [LI-Init](https://github.com/hku-mars/LiDAR_IMU_Init) 的 LiDAR-IMU 在线时空标定，并将标定结果应用于 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 进行建图验证。

## 📖 项目简介

本项目主要用于 **LiDAR 与外接 IMU 的时空标定**，涵盖外参（旋转、平移）、时间偏移以及 IMU 零偏的联合估计。标定完成后，将结果直接写入 FAST-LIO2 / FAST-LIVO2 的配置文件，实现高精度实时激光惯性建图与激光-惯性-视觉融合建图。

同时覆盖 **D435i 内置 IMU 内参标定** 与 **内置相机–IMU 外参 / 时间偏移联合优化**（基于 VINS-Mono），以及 **LiDAR–Camera 外参标定**（FAST-Calib / direct_visual_lidar_calibration），形成 LiDAR、Camera、IMU 三类传感器标定的完整闭环。
例如：

<img width="400" alt="629522958-44064b1e-27bb-43ea-94eb-430dd4f8e14e_compressed"
src="https://github.com/user-attachments/assets/11599551-8a58-4386-82f5-277cf04f73ce" />

**3D打印件**：[Mid360雷达+D435i相机3D打印支架](https://makerworld.com/zh/models/1838891-d435i-and-mid360-radar-vision-integrated-bracket-m)


目前包含三套标定方案：

| 方案 | LiDAR | IMU | 标定指南 |
|------|-------|-----|---------|
| 方案一 | Livox MID360 | 超核（Hipnuc）IMU | [Mid360_超核IMU_LiDAR_IMU_Init完整标定与排错指南.md](./Mid360_超核IMU_LiDAR_IMU_Init完整标定与排错指南.md) |
| 方案二 | RoboSense 16 线 | 外接 IMU | [RS16_外接IMU_LiDAR_IMU_Init标定指南.md](./RS16_外接IMU_LiDAR_IMU_Init标定指南.md) |
| 方案三 | D435i 内置 RGB 相机 | D435i 内置 IMU（BMI055） | [D435i中imu内参标定以及内置相机与imu内参优化联合标定指南.md](./D435i中imu内参标定以及内置相机与imu内参优化联合标定指南.md) |




## 😎 lidar与camera标定用的（附加）：

- **[direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)**（无目标板标定，MID360+D435i）：依赖安装与编译一键脚本 [setup_dvlc.sh](./setup_dvlc.sh)（GTSAM / Ceres 2.x / Iridescence）
- **[FAST-Calib](https://github.com/hku-mars/FAST-Calib)**（目标板标定，约 1 秒完成）：已集成于 `src/FAST-Calib`，标定结果可直接写入 FAST-LIVO2 配置



最终mid360雷达+D435i相机+超核imu跑了一下fast-lio2以及fast-livo2：


<img width="1705" height="903" alt="fast-livo2" src="https://github.com/user-attachments/assets/0e172978-f3de-4de5-86f4-8bf34a6e20dc" />




## ✨ 功能特点

- 🔧 **时空联合标定**：基于 LI-Init 算法，同时估计 LiDAR-IMU 外参旋转/平移、时间延迟与 IMU 零偏
- 🗺️ **FAST-LIO2 集成**：标定结果一键写入配置，直接驱动 FAST-LIO2 建图
- 🎯 **多传感器支持**：Livox MID360、RoboSense16、超核 IMU、D435i 等
- 🎥 **相机-IMU 标定**：D435i IMU 六面内参标定 + VINS-Mono 相机-IMU 外参/时间偏移联合优化
- 🛰️ **LiDAR-Camera 标定**：FAST-Calib（目标板，1 秒级）与 direct_visual_lidar_calibration（无目标板）双方案
- 🔗 **FAST-LIVO2 集成**：LiDAR-IMU 与 LiDAR-Camera 外参同时写入配置，驱动激光-惯性-视觉融合建图
- 📝 **完整排错文档**：附详尽的标定流程与常见问题排查指南

## 📁 目录结构

```
lidar_imu_camera_ws/
├── src/
│   ├── FAST_LIO/                 # FAST-LIO2 激光-惯性建图（已集成标定外参）
│   ├── FAST-LIVO2/               # FAST-LIVO2 激光-惯性-视觉融合建图
│   ├── FAST-Calib/               # LiDAR-Camera 目标板标定（FAST-LIVO2 配套）
│   ├── LiDAR_IMU_Init/           # LI-Init 时空标定程序
│   ├── VINS-Mono/                # VINS-Mono（相机-IMU 联合标定 / VIO）
│   ├── d435i_vins_calib/         # D435i RGB+IMU 专用启动 / 配置包
│   ├── direct_visual_lidar_calibration/  # LiDAR-Camera 无目标板标定（独立编译）
│   ├── livox_ros_driver2/        # Livox 雷达驱动（ROS1）
│   ├── rslidar_sdk/              # RoboSense 雷达驱动
│   ├── vikit_common/ vikit_ros/  # FAST-LIVO2 依赖（Vikit）
│   ├── hipnuc_imu/               # 超核 IMU 驱动
│   └── hipnuc_lib_package/       # 超核 IMU 解码库
├── results/                      # 标定结果记录（本机产物，未纳入 git）
├── D435i中imu内参标定以及内置相机与imu内参优化联合标定指南.md
├── Mid360_超核IMU_LiDAR_IMU_Init完整标定与排错指南.md
├── RS16_外接IMU_LiDAR_IMU_Init标定指南.md
├── run_vins_bag.sh               # 离线跑 VINS-Mono 一键脚本（自动开 4 终端）
├── setup_dvlc.sh                 # direct_visual_lidar_calibration 编译安装脚本
├── LICENSE
└── README.md
```

## 🔧 环境依赖

| 依赖 | 版本 |
|------|------|
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| C++ 标准 | C++14 |
| PCL | ≥ 1.8 |
| Eigen | ≥ 3.3 |

### 第三方 SDK（需手动安装）

以下 SDK 体积较大，未纳入仓库，请按官方说明安装到系统路径（`/usr/local/lib`）：

- **Livox-SDK2** — [github.com/Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
  ```bash
  cd ~ && git clone https://github.com/Livox-SDK/Livox-SDK2.git
  cd Livox-SDK2 && mkdir build && cd build
  cmake .. && make -j
  sudo make install
  ```

## 🚀 安装

```bash
# 克隆仓库
git clone https://github.com/Robot-Nav/lidar_imu_camera_ws.git
cd lidar_imu_camera_ws

# 编译（livox_ros_driver2 需指定 ROS1 版本）
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DROS_EDITION=ROS1 \
  -DCeres_DIR=/usr/lib/cmake/Ceres \
  -DSophus_DIR=/home/zjs/SophusConfig \
  -DCATKIN_BLACKLIST_PACKAGES="direct_visual_lidar_calibration"

source devel/setup.bash
```

> ⚠️ **Ceres 版本冲突**：本工作区大部分包（LiDAR_IMU_Init、VINS-Mono 等）使用 Ceres **1.x** 旧 API（`ceres::LocalParameterization`），而 [direct_visual_lidar_calibration](./src/direct_visual_lidar_calibration) 依赖其自带的 Sophus 2.x，要求 Ceres **2.x**（`ceres/manifold.h`）。两者互斥，当前统一使用 apt 的 **Ceres 1.14**，并通过 `CATKIN_BLACKLIST_PACKAGES` 排除 direct_visual_lidar_calibration。若源码安装过 Ceres 2.x 到 `/usr/local`，请先移走，避免头文件被优先搜索到（详见 [注意事项](#️-注意事项)）。

> 🛠️ **direct_visual_lidar_calibration 独立编译**：该包与主工作区依赖冲突，单独用 [setup_dvlc.sh](./setup_dvlc.sh) 安装（自动编译 GTSAM → Ceres 2.x → Iridescence → 工具本体，并修复 /usr/local 残缺 GLFW）。注意脚本最后会对主工作区执行 `catkin_make`，若需保持主工作区 Ceres 1.14，编译 DVLC 前后请按注意事项 4 移走 /usr/local 下的 Ceres 2.x。

## 📐 标定流程

详细步骤见标定指南文档。简要流程：

1. **录包或在线标定**：充分激励（六自由度运动），覆盖各轴旋转与平移
2. **离线标定**时记得关闭 `use_sim_time`
3. **获取结果**：标定完成后查看 `src/LiDAR_IMU_Init/result/Initialization_result.txt`，取 **Refinement result**（精化结果）
4. **写入配置**：将外参矩阵 `R`、平移 `T`、时间偏移填入 FAST-LIO2 配置文件

> 💡 标定结果中给出的是 **LiDAR → IMU** 的齐次变换矩阵，与 FAST-LIO2 的 `extrinsic_R` / `extrinsic_T` 约定一致（`P_imu = R * P_lidar + T`），可直接填入。

## 🗺️ FAST-LIO2 建图测试

以 MID360 + 超核 IMU 为例，三个终端依次启动：

**终端 1 — 启动 IMU**
```bash
source devel/setup.bash
roslaunch hipnuc_imu imu_msg.launch
```

**终端 2 — 启动 LiDAR**
```bash
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

**终端 3 — 启动 FAST-LIO2**
```bash
source devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

### 标定结果示例

下表为 MID360 + 超核 IMU 的标定结果（已写入 [`src/FAST_LIO/config/mid360.yaml`](./src/FAST_LIO/config/mid360.yaml)）：

| 参数 | 值 |
|------|-----|
| `extrinsic_T` | `[0.038238, 0.028072, 0.067594]` m |
| `extrinsic_R` | 见配置文件 |
| `time_offset_lidar_to_imu` | `-0.019056` s |
| `extrinsic_est_en` | `false`（已有标定，关闭在线估计） |

## 🎥 D435i 相机–IMU 标定（VINS-Mono）

完整流程见 [D435i中imu内参标定以及内置相机与imu内参优化联合标定指南.md](./D435i中imu内参标定以及内置相机与imu内参优化联合标定指南.md)，要点：

1. **IMU 内参**：用 RealSense 官方 `rs-imu-calibration.py` 做六面静止标定，把 Accel/Gyro Motion Intrinsic 写入 EEPROM（本机 `norm(fixed data)` = 9.805369，接近标准重力）
2. **相机内参**：固定 RGB `640×480@30Hz`，直接读取 `/camera/color/camera_info` 出厂参数，不做在线优化
3. **联合优化**：VINS-Mono 以出厂 `Color → Gyro` 外参为初值（`estimate_extrinsic=1`、`estimate_td=1`）离线优化外参与时间偏移
4. **一键运行**：录好 bag 后执行 `bash run_vins_bag.sh <bag路径> [倍速]`，自动打开 roscore / VINS / RViz / rosbag 四个终端窗口

D435i RGB→IMU 最终外参（Run1+Run2 均值，旋转差 0.054°、平移差 <3.1 mm，已固化到 [d435i_final.yaml](./src/VINS-Mono/config/d435i/d435i_final.yaml)）：

| 参数 | 值 |
|------|-----|
| `extrinsicTranslation` | `[-0.023231, 0.008525, 0.002254]` m |
| `td`（t_imu = t_image + td） | `0.00881` s |
| `extrinsicRotation` | 见 [d435i_final.yaml](./src/VINS-Mono/config/d435i/d435i_final.yaml) |
| 最终配置 | `estimate_extrinsic=0`、`estimate_td=0`（固化） |

> ⚠️ D435i IMU 0 帧问题：ROS apt 版 librealsense **2.50 无 IIO 后端**，需软链到 **2.58.3**，且**切勿屏蔽 `hid_sensor_hub`**；`accel_fps` 只支持 250/63，不要设为 200（详见指南 20.8 节）。

## 🛰️ LiDAR–Camera 标定

两套工具任选其一：

- **FAST-Calib**（目标板，约 1 秒完成）：`src/FAST-Calib`。单场景 `roslaunch fast_calib calib.launch`，至少 3 个场景后 `roslaunch fast_calib multi_calib.launch` 联合优化
- **direct_visual_lidar_calibration**（无目标板）：依赖安装与编译用 [setup_dvlc.sh](./setup_dvlc.sh)，可先用 `initial_guess_auto` 得初值、再用 `calibrate` 精化

FAST-LIVO2 配置 [mid360.yaml](./src/FAST-LIVO2/config/mid360.yaml) 已写入 LiDAR→IMU 外参 `extrinsic_R/T` 与 LiDAR→Camera 外参 `Rcl/Pcl`。

## 🔗 FAST-LIVO2 融合建图

MID360 + D435i + 超核 IMU 三传感器激光-惯性-视觉融合建图，外参已固化在 [mid360.yaml](./src/FAST-LIVO2/config/mid360.yaml)，依次启动：

**终端 1 — IMU**：`roslaunch hipnuc_imu imu_msg.launch`
**终端 2 — LiDAR**：`roslaunch livox_ros_driver2 msg_MID360.launch`
**终端 3 — D435i**：`roslaunch d435i_vins_calib d435i_rgb_imu.launch`
**终端 4 — FAST-LIVO2**：

```bash
source devel/setup.bash
roslaunch fast_livo mapping_mid360.launch
```

## 🧭 RS16 建图（rslidar_sdk）

RoboSense 16 线经 [rslidar_sdk](./src/rslidar_sdk) 驱动发布点云，配合外接 IMU 跑 FAST-LIO2：

```bash
source devel/setup.bash
roslaunch rslidar_sdk start.launch                 # RS16 驱动
roslaunch fast_lio mapping_robosense16.launch      # FAST-LIO2 建图
```

## ⚠️ 注意事项

1. **先标定后建图**：可录包离线标定，或直接在线标定；离线标定记得关闭 `use_sim_time` 仿真时间
2. **配置话题**：标定完成后，按所用雷达与 IMU 修改 FAST-LIO2 对应 yaml 的 `lid_topic` / `imu_topic` 等参数
3. **ROS 版本**：`livox_ros_driver2` 编译时须传 `-DROS_EDITION=ROS1`，否则会进入 ROS2 分支报错
4. **Ceres 版本冲突（重要）**：本工作区统一使用 **Ceres 1.14**（apt 安装）。`direct_visual_lidar_calibration` 因依赖 Sophus 2.x 需要 **Ceres 2.x**，与 LiDAR_IMU_Init / VINS-Mono 的旧 API（`ceres::LocalParameterization`，Ceres 2.x 已移除）互斥，故默认排除不编译。**切勿在 `/usr/local` 源码安装 Ceres 2.x**，否则 GCC 会优先搜索 `/usr/local/include` 导致所有包编译报 `LocalParameterization is not a member of ceres`。若已安装，请移走三处：`/usr/local/include/ceres`、`/usr/local/lib/cmake/Ceres`、`/usr/local/lib/libceres.a`，再重新编译。
5. **外参约定**：LI-Init 输出的变换矩阵为 LiDAR→IMU，无需取逆，直接填入 `extrinsic_R` / `extrinsic_T`
6. **D435i IMU 读取**：需要 librealsense ≥ 2.58.3（IIO 后端）；不要 blacklist `hid_sensor_hub`；`accel_fps` 用 250/63
7. **VINS 外参方向**：D435i 联合标定必须使用 `Color → Gyro`（`T_imu_cam`），方向错误会导致初始化失败 / 轨迹发散
8. **嵌套仓库已扁平化**：`src` 下的 VINS-Mono、direct_visual_lidar_calibration、rslidar_sdk 等已去除嵌套 `.git`，以普通源码收录；需要历史 / 升级请直接 clone 上游仓库

## 🌹 致谢

本项目基于以下开源工作构建，在此向相关团队致以诚挚谢意：

- **[FAST-LIO2](https://github.com/hku-mars/FAST_LIO)** — 感谢香港大学火星实验室（HKU MARS）Wei Xu 团队提供的快速直接激光惯性里程计，本项目在其基础上集成标定外参进行建图验证。
- **[LiDAR_IMU_Init (LI-Init)](https://github.com/hku-mars/LiDAR_IMU_Init)** — 同样感谢 HKU MARS 团队提供的鲁棒 LiDAR-IMU 在线初始化与时空标定算法。
- **[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)** / **[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)** — 感谢 Livox 官方提供的设备驱动。
- **[VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)** — 感谢港科大 Aerial Robotics 团队提供的单目视觉惯性里程计，本项目用于 D435i 相机-IMU 联合标定。
- **[FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)** / **[FAST-Calib](https://github.com/hku-mars/FAST-Calib)** — 感谢 HKU MARS 团队提供的激光-惯性-视觉融合里程计与 1 秒 LiDAR-Camera 标定工具。
- **[direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)** — 感谢 koide3 提供的无目标板 LiDAR-Camera 标定工具。
- **[rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)** — 感谢 RoboSense 官方提供的雷达驱动。
- **Intel RealSense（D435i）** — 感谢提供的相机 / IMU 硬件与 librealsense SDK 支持。
- **超核（Hipnuc）IMU** — 感谢提供的 IMU 硬件及驱动支持。

## 📄 License

本项目采用 [MIT License](./LICENSE) 开源协议。

