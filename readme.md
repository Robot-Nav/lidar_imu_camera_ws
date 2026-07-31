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

本项目主要用于 **LiDAR 与外接 IMU 的时空标定**，涵盖外参（旋转、平移）、时间偏移以及 IMU 零偏的联合估计。标定完成后，将结果直接写入 FAST-LIO2 的配置文件，实现高精度实时激光惯性建图。
例如：

<img width="3072" height="4096" alt="lidar-imu" src="https://github.com/user-attachments/assets/44064b1e-27bb-43ea-94eb-430dd4f8e14e" />


目前包含两套标定方案：

| 方案 | LiDAR | IMU | 标定指南 |
|------|-------|-----|---------|
| 方案一 | Livox MID360 | 超核（Hipnuc）IMU | [Mid360_超核IMU_LiDAR_IMU_Init完整标定与排错指南.md](./Mid360_超核IMU_LiDAR_IMU_Init完整标定与排错指南.md) |
| 方案二 | RoboSense 16 线 | 外接 IMU | [RS16_外接IMU_LiDAR_IMU_Init标定指南.md](./RS16_外接IMU_LiDAR_IMU_Init标定指南.md) |

## ✨ 功能特点

- 🔧 **时空联合标定**：基于 LI-Init 算法，同时估计 LiDAR-IMU 外参旋转/平移、时间延迟与 IMU 零偏
- 🗺️ **FAST-LIO2 集成**：标定结果一键写入配置，直接驱动 FAST-LIO2 建图
- 🎯 **多传感器支持**：Livox MID360、RoboSense16、超核 IMU 等
- 📝 **完整排错文档**：附详尽的标定流程与常见问题排查指南

## 📁 目录结构

```
lidar_imu_ws/
├── src/
│   ├── FAST_LIO/              # FAST-LIO2 源码（已集成标定外参）
│   ├── LiDAR_IMU_Init/        # LI-Init 标定程序
│   ├── livox_ros_driver2/     # Livox 雷达驱动（ROS1）
│   ├── hipnuc_imu/            # 超核 IMU 驱动
│   └── hipnuc_lib_package/    # 超核 IMU 解码库
├── Mid360_超核IMU_LiDAR_IMU_Init完整标定与排错指南.md
├── RS16_外接IMU_LiDAR_IMU_Init标定指南.md
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
git clone https://github.com/Robot-Nav/lidar_imu_ws.git
cd lidar_imu_ws

# 编译（livox_ros_driver2 需指定 ROS1 版本）
source /opt/ros/noetic/setup.bash
catkin_make -DROS_EDITION=ROS1 \
  -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver2;hipnuc_lib_package;hipnuc_imu;fast_lio"

source devel/setup.bash
```

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

## ⚠️ 注意事项

1. **先标定后建图**：可录包离线标定，或直接在线标定；离线标定记得关闭 `use_sim_time` 仿真时间
2. **配置话题**：标定完成后，按所用雷达与 IMU 修改 FAST-LIO2 对应 yaml 的 `lid_topic` / `imu_topic` 等参数
3. **ROS 版本**：`livox_ros_driver2` 编译时须传 `-DROS_EDITION=ROS1`，否则会进入 ROS2 分支报错
4. **外参约定**：LI-Init 输出的变换矩阵为 LiDAR→IMU，无需取逆，直接填入 `extrinsic_R` / `extrinsic_T`

## ♪(･ω･)ﾉ 致谢

本项目基于以下开源工作构建，在此向相关团队致以诚挚谢意：

- **[FAST-LIO2](https://github.com/hku-mars/FAST_LIO)** — 感谢香港大学火星实验室（HKU MARS）Wei Xu 团队提供的快速直接激光惯性里程计，本项目在其基础上集成标定外参进行建图验证。
- **[LiDAR_IMU_Init (LI-Init)](https://github.com/hku-mars/LiDAR_IMU_Init)** — 同样感谢 HKU MARS 团队提供的鲁棒 LiDAR-IMU 在线初始化与时空标定算法。
- **[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)** / **[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)** — 感谢 Livox 官方提供的设备驱动。
- **超核（Hipnuc）IMU** — 感谢提供的 IMU 硬件及驱动支持。

## 📄 License

本项目采用 [MIT License](./LICENSE) 开源协议。
