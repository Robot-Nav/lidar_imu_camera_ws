# D435i中IMU内参标定以及内置相机与IMU内参优化联合标定指南

> 适用环境：Ubuntu 20.04、ROS Noetic、Intel RealSense D435i  
> 工作空间：`~/lidar_imu_camera_ws`  
> 本机设备：D435i，序列号 `109622073033`，固件 `5.16.0.1`，IMU 型号 `BMI055`  
> RGB 固定配置：`640 × 480 @ 30 Hz`  
> 目标：完成 D435i 内置 IMU 确定性内参校准，读取 RGB 出厂内参和 RGB–IMU 出厂外参，并以出厂外参为初值，使用 VINS-Mono 优化 RGB–IMU 外参和时间偏移。

---

## 0. 先明确本指南实际优化的参数

本流程包含三个不同层次的参数，不能混为一谈。

### 0.1 D435i IMU 确定性内参

使用 RealSense 官方 `rs-imu-calibration.py` 六面静止标定，主要修正：

- 加速度计零偏；
- 加速度计比例因子；
- 三轴非正交和交叉轴误差；
- 陀螺仪静态零偏；
- 将最终 Motion Intrinsic 写入 D435i EEPROM。

写入成功后，librealsense 和 ROS 驱动发布的 IMU 数据会自动应用这些修正。

### 0.2 RGB 相机内参

本流程**不重新优化 RGB 相机内参**，而是直接读取 D435i 在固定分辨率下的出厂参数：

- `fx、fy、cx、cy`；
- `k1、k2、p1、p2`；
- 图像分辨率。

相机内参与分辨率绑定。后续必须保持 `640 × 480 @ 30 Hz`；改变分辨率后，应重新读取 `/camera/color/camera_info`。

### 0.3 RGB–IMU 联合优化

VINS-Mono 阶段优化的是：

- RGB 相机到 IMU 的旋转外参；
- RGB 相机到 IMU 的平移外参；
- RGB 与 IMU 的时间偏移 `td`；
- 运行时 IMU bias。

VINS-Mono **不会重新标定加速度计比例因子，也不会优化 RGB 相机内参**。

---

# 1. 最终技术路线

```text
安装并验证 librealsense
        ↓
编译与系统 SDK 同版本的 pyrealsense2
        ↓
D435i IMU 六面静止标定
        ↓
写入 EEPROM 并重新插拔验证
        ↓
固定 RGB 为 640×480@30 Hz
        ↓
读取 RGB 出厂内参
        ↓
读取 Color→Gyro 出厂外参
        ↓
构造 VINS-Mono 配置
        ↓
录制 RGB+IMU 六自由度运动数据
        ↓
estimate_extrinsic=1、estimate_td=1
        ↓
比较多组结果并固化
        ↓
estimate_extrinsic=0、estimate_td=0
```

本指南默认**跳过三小时 Allan 方差**。VINS 中先使用经验噪声初值；若后续 VIO 初始化不稳或轨迹噪声明显，再补做 Allan 方差。

---

# 2. 工作空间目录规划

在现有工作空间中创建目录：

```bash
cd ~/lidar_imu_camera_ws

mkdir -p bags/d435i/vins
mkdir -p results/d435i/factory
mkdir -p results/d435i/imu_calib
mkdir -p results/d435i/vins/run1
mkdir -p results/d435i/vins/run2
mkdir -p results/d435i/vins/run3
mkdir -p third_party
```

建议最终结构：

```text
~/lidar_imu_camera_ws
├── bags
│   └── d435i
│       └── vins
├── results
│   └── d435i
│       ├── factory
│       ├── imu_calib
│       └── vins
│           ├── run1
│           ├── run2
│           └── run3
├── src
│   ├── VINS-Mono
│   └── d435i_vins_calib
└── third_party
    └── librealsense
```

---

# 3. 安装并验证 RealSense SDK

## 3.1 检查设备

```bash
which rs-enumerate-devices
rs-enumerate-devices -s
```

本机正常输出：

```text
Device Name        Serial Number   Firmware Version
RealSense D435I    109622073033    5.16.0.1
```

查看完整设备参数：

```bash
rs-enumerate-devices -c \
  | tee ~/lidar_imu_camera_ws/results/d435i/factory/device_info.txt
```

## 3.2 USB 检查

```bash
lsusb -t
```

D435i 最好工作在 `5000M` 或更高速度的 USB 3.x 总线上。避免：

- 无源 USB HUB；
- 质量差的延长线；
- 同一控制器下同时连接多个高带宽相机或雷达；
- 将 RGB、Depth 同时开到高分辨率。

本机设备报告的 USB 类型为 `3.2`。

---

# 4. 统一 librealsense 与 pyrealsense2 版本

本机系统安装的 SDK 为 `librealsense2 2.58.3`。Ubuntu 20.04 的 Python 3.8 在 PyPI 上只能直接安装到旧版 `pyrealsense2 2.55.1.6486`，两者版本不一致时，六面采集可能成功，但 EEPROM 写入阶段可能异常退出。

因此采用源码编译 `pyrealsense2 2.58.3`。

## 4.1 准备源码

```bash
cd ~/lidar_imu_camera_ws/third_party

git clone --branch v2.58.3 --depth 1 \
  https://github.com/realsenseai/librealsense.git
```

已有源码时检查：

```bash
cd ~/lidar_imu_camera_ws/third_party/librealsense
git describe --tags --always
```

应输出：

```text
v2.58.3
```

## 4.2 安装编译依赖

```bash
sudo apt install -y \
  build-essential \
  cmake \
  python3-dev \
  python3-numpy \
  libusb-1.0-0-dev \
  libssl-dev
```

若之前通过 pip 安装了旧版：

```bash
python3 -m pip uninstall pyrealsense2
```

## 4.3 编译 Python Binding

```bash
cd ~/lidar_imu_camera_ws/third_party/librealsense

rm -rf build
mkdir build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF
```

检查：

```bash
grep -E "BUILD_PYTHON_BINDINGS|PYTHON_EXECUTABLE|PYTHON_INSTALL_DIR" \
  CMakeCache.txt
```

应看到类似：

```text
BUILD_PYTHON_BINDINGS:BOOL=ON
PYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3
```

编译：

```bash
make -j4
```

成功标志：

```text
[100%] Built target pyrealsense2
```

查找模块：

```bash
find . -name "pyrealsense2*.so"
```

本机生成位置：

```text
./Release/pyrealsense2.cpython-38-x86_64-linux-gnu.so
```

## 4.4 配置 PYTHONPATH

当前终端：

```bash
export PYTHONPATH=$HOME/lidar_imu_camera_ws/third_party/librealsense/build/Release:$PYTHONPATH
```

验证：

```bash
python3 -c "import pyrealsense2 as rs; print(rs.__file__)"
```

应指向：

```text
/home/zjs/lidar_imu_camera_ws/third_party/librealsense/build/Release/pyrealsense2.cpython-38-x86_64-linux-gnu.so
```

永久写入：

```bash
echo 'export PYTHONPATH=$HOME/lidar_imu_camera_ws/third_party/librealsense/build/Release:$PYTHONPATH' \
  >> ~/.bashrc

source ~/.bashrc
```

测试设备：

```bash
python3 - <<'PY'
import pyrealsense2 as rs

ctx = rs.context()
devices = ctx.query_devices()

print("RealSense 数量:", len(devices))
for dev in devices:
    print("名称:", dev.get_info(rs.camera_info.name))
    print("序列号:", dev.get_info(rs.camera_info.serial_number))
    print("固件:", dev.get_info(rs.camera_info.firmware_version))
PY
```

---

# 5. D435i IMU 六面内参标定

## 5.1 标定前注意事项

标定前关闭所有占用相机的程序：

- `realsense-viewer`；
- `realsense2_camera`；
- RViz 中的相机节点；
- 其他 pyrealsense2 程序。

检查：

```bash
rosnode list
```

不要在设备被其他进程占用时运行标定。

建议：

- 使用泡沫、纸盒槽、打印支架或直角块固定；
- 手可以用于切换姿态，但采集阶段应松手；
- 不要让镜头直接压在桌面；
- USB 线固定，避免拉扯；
- 每次摆好后等待设备停止晃动；
- 脚本提示 `Direction data collected.` 后再切换；
- 六面过程中不要按 `Ctrl+C`。

## 5.2 六个姿态的实际含义

RealSense 运动坐标通常可理解为：

```text
正对镜头：
+X 向右
+Y 向下
+Z 向镜头前方
```

官方脚本会直接给出英文摆放提示，应以终端提示为准。本机脚本顺序如下：

```text
[ 0. -1.  0.]  Mounting screw pointing down, device facing out
[ 1.  0.  0.]  Mounting screw pointing left, device facing out
[ 0.  1.  0.]  Mounting screw pointing up, device facing out
[-1.  0.  0.]  Mounting screw pointing right, device facing out
[ 0.  0. -1.]  Viewing direction facing down
[ 0.  0.  1.]  Viewing direction facing up
```

切换方向时出现：

```text
WARNING: MOVING
```

属于正常现象。摆稳后脚本会重新开始有效采样。

每个方向不需要手动计时。通常有效采样约几秒到十几秒；加上摆放和稳定，预留约 `10～30 秒/方向`。只有看到：

```text
Direction data collected.
```

才能换下一个方向。

## 5.3 保存标定前信息

```bash
rs-enumerate-devices -c \
  > ~/lidar_imu_camera_ws/results/d435i/factory/before_imu_calib.txt
```

## 5.4 运行标定

```bash
cd ~/lidar_imu_camera_ws/third_party/librealsense/tools/rs-imu-calibration

python3 rs-imu-calibration.py \
  -s 109622073033 \
  -g
```

六面采集完成后，脚本会询问是否保存原始数据：

```text
Would you like to save the raw data?
Enter footer for saving files...
```

输入：

```text
d435i_imu_calib
```

会生成：

```text
accel_d435i_imu_calib.txt
gyro_d435i_imu_calib.txt
```

备份：

```bash
cp accel_d435i_imu_calib.txt \
   gyro_d435i_imu_calib.txt \
   ~/lidar_imu_camera_ws/results/d435i/imu_calib/
```

## 5.5 判断标定质量

脚本会输出：

```text
norm (raw data  ): ...
norm (fixed data): ... A good calibration will be near 9.806650
```

本机最终结果：

```text
norm (fixed data): 9.805369
```

与标准重力加速度 `9.806650 m/s²` 的差值约为：

```text
0.001281 m/s²
```

说明标定结果良好。

不要只根据 `residuals` 的单个数值判断失败，应重点观察：

- `rank` 是否正常；
- 六个方向是否都有足够样本；
- `norm(fixed data)` 是否明显比 `norm(raw data)` 更接近 `9.806650`。

## 5.6 写入 EEPROM

出现：

```text
Would you like to write the results to the camera? (Y/N)
```

输入大写或小写：

```text
Y
```

成功输出：

```text
Writing calibration to device.
Device PID: 0B3A
Device name: RealSense D435I
Serial number: 109622073033
Firmware version: 5.16.0.1
SUCCESS: saved calibration to camera.
Done.
```

若在 `SUCCESS` 和 `Done.` 之后又出现：

```text
QFileSystemWatcher::removePaths: list is empty
FATAL: exception not rethrown
已放弃 (核心已转储)
```

这属于脚本退出阶段的 Qt/资源清理异常。只要已经出现：

```text
SUCCESS: saved calibration to camera.
```

并且重新插拔后 `rs-enumerate-devices -c` 能看到新的 Motion Intrinsic，就说明 EEPROM 已写入成功。

## 5.7 重新插拔并验证

1. 等程序返回终端；
2. 拔掉 D435i；
3. 等待约 5 秒；
4. 重新插入 USB 3.x；
5. 检查设备。

```bash
rs-enumerate-devices -s
```

保存写入后的完整信息：

```bash
rs-enumerate-devices -c \
  > ~/lidar_imu_camera_ws/results/d435i/factory/after_imu_calib.txt
```

---

# 6. 本机写入后的 IMU 内参

## 6.1 Gyro Motion Intrinsic

```text
Sensitivity:
 1.000000  0.000000  0.000000  -0.000039
 0.000000  1.000000  0.000000  -0.000012
 0.000000  0.000000  1.000000  -0.000027
```

可表示为：

```text
ω_corrected = M_g · [ωx, ωy, ωz, 1]^T
```

其中前三列接近单位阵，最后一列为静态零偏修正项。

## 6.2 Accel Motion Intrinsic

```text
Sensitivity:
 1.016180   0.007751   0.000911  -0.064920
 0.010672   1.009747  -0.053151   0.254651
-0.062725   0.048204   1.023140   0.065594
```

前三列包含：

- 三轴比例因子；
- 轴非正交误差；
- 交叉轴耦合。

最后一列为加速度计零偏修正项。

注意：这些参数已经写入相机，后续 `/camera/imu` 数据会由 librealsense 自动应用，不要在 VINS 前再手动乘一次矩阵，否则会重复校正。

---

# 7. 创建统一的 D435i ROS 启动文件

由于当前只进行 RGB–IMU VINS，不需要 Depth。关闭深度流可以避免 USB 带宽不足和：

```text
uvc streamer watchdog triggered
Depth stream start failure
control_transfer returned error
```

创建：

```bash
mkdir -p ~/lidar_imu_camera_ws/src/d435i_vins_calib/launch
mkdir -p ~/lidar_imu_camera_ws/src/d435i_vins_calib/config
mkdir -p ~/lidar_imu_camera_ws/src/d435i_vins_calib/scripts

nano ~/lidar_imu_camera_ws/src/d435i_vins_calib/launch/d435i_rgb_imu.launch
```

写入：

```xml
<launch>
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">

    <!-- 不使用深度与红外，降低USB负载 -->
    <arg name="enable_depth"  value="false"/>
    <arg name="enable_infra1" value="false"/>
    <arg name="enable_infra2" value="false"/>

    <!-- 固定RGB配置 -->
    <arg name="enable_color" value="true"/>
    <arg name="color_width"  value="640"/>
    <arg name="color_height" value="480"/>
    <arg name="color_fps"    value="30"/>

    <!-- D435i内置IMU -->
    <arg name="enable_gyro"  value="true"/>
    <arg name="gyro_fps"     value="200"/>
    <arg name="enable_accel" value="true"/>
    <arg name="accel_fps"    value="63"/>

    <!-- 合并为/camera/imu -->
    <arg name="unite_imu_method" value="linear_interpolation"/>

    <!-- 发布出厂TF -->
    <arg name="publish_tf" value="true"/>

    <!-- USB初始化异常时可临时改true -->
    <arg name="initial_reset" value="false"/>

  </include>
</launch>
```

启动：

```bash
source /opt/ros/noetic/setup.bash
source ~/lidar_imu_camera_ws/devel/setup.bash

roslaunch d435i_vins_calib d435i_rgb_imu.launch
```

检查：

```bash
rostopic list | grep camera
```

至少应包含：

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/imu
```

检查频率：

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/imu
```

目标：

```text
RGB ≈ 30 Hz
IMU ≈ 200 Hz
```

---

# 8. 读取并固定 RGB 相机内参

## 8.1 保存 ROS CameraInfo

```bash
rostopic echo -n 1 /camera/color/camera_info \
  > ~/lidar_imu_camera_ws/results/d435i/factory/color_camera_info_640x480.yaml
```

重点字段：

```yaml
width: 640
height: 480

D: [k1, k2, p1, p2, k3]

K: [fx, 0, cx,
    0, fy, cy,
    0, 0, 1]
```

VINS 使用：

```text
fx = K[0]
fy = K[4]
cx = K[2]
cy = K[5]

k1 = D[0]
k2 = D[1]
p1 = D[2]
p2 = D[3]
```

## 8.2 本机 `rs-enumerate-devices -c` 中的 640×480 RGB 参数

```text
Width:  640
Height: 480
PPX:    328.494903564453
PPY:    240.872558593750
Fx:     604.457275390625
Fy:     604.577331542969
Distortion: Inverse Brown Conrady
Coeffs: 0 0 0 0 0
```

因此可作为检查参考：

```yaml
projection_parameters:
  fx: 604.457275390625
  fy: 604.577331542969
  cx: 328.494903564453
  cy: 240.872558593750

distortion_parameters:
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
```

最终配置应优先使用实际 `/camera/color/camera_info` 输出；若两者有轻微差异，以 ROS 运行时参数为准。

---

# 9. 读取 RGB–IMU 出厂外参

## 9.1 变换方向

VINS-Mono 要求：

```text
T_imu_cam
```

也就是把相机坐标系中的点变换到 IMU 坐标系：

```text
p_imu = R_imu_cam · p_cam + t_imu_cam
```

对于 D435i，IMU 主体可使用 Gyro 坐标系，因此应读取：

```text
Extrinsic from "Color" To "Gyro"
```

不要误用：

```text
Extrinsic from "Gyro" To "Color"
```

## 9.2 本机出厂 `Color → Gyro` 外参

旋转矩阵：

```text
R_imu_cam =
[ 0.999982   -0.00550951  -0.00243105
  0.00550956  0.999985     0.0000109132
  0.00243095 -0.000024307   0.999997 ]
```

平移，单位米：

```text
t_imu_cam =
[-0.0203096531331539,
  0.00498574459925294,
  0.0112929595634341]
```

这组参数可直接作为 VINS 的外参初值。

## 9.3 使用 ROS TF 再验证方向

查看坐标系：

```bash
rosrun tf view_frames
```

或者：

```bash
rosrun tf tf_echo \
  camera_gyro_optical_frame \
  camera_color_optical_frame
```

这里 `target=gyro`、`source=color`，输出含义应为：

```text
T_gyro_color = T_imu_cam
```

不同 realsense-ros 版本的 frame 名称可能略有差异。以：

```bash
rostopic echo -n 1 /camera/imu/header/frame_id
rostopic echo -n 1 /camera/color/image_raw/header/frame_id
```

实际输出为准。

---

# 10. 准备 VINS-Mono

## 10.1 下载

```bash
cd ~/lidar_imu_camera_ws/src

git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
```

## 10.2 编译注意事项

你的主工作空间还包含 FAST-LIO、FAST-LIVO2、Livox 驱动和 LiDAR_IMU_Init。若其中任何无关包 CMake 配置失败，会导致整个 `catkin_make` 失败。

处理原则：

- 先修复无关包；
- 或临时将无关包移出 `src`；
- 或给当前不参与构建的包目录添加 `CATKIN_IGNORE`；
- 标定完成后再移除 `CATKIN_IGNORE`。

不要在同一工作空间反复混用 `catkin_make` 和 `catkin build`。

编译：

```bash
cd ~/lidar_imu_camera_ws

source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash
```

检查：

```bash
rospack find feature_tracker
rospack find vins_estimator
```

---

# 11. 创建完整 VINS 配置

创建：

```bash
nano ~/lidar_imu_camera_ws/src/d435i_vins_calib/config/d435i_vins.yaml
```

模板：

```yaml
%YAML:1.0

#===============================
# ROS topics
#===============================
imu_topic: "/camera/imu"
image_topic: "/camera/color/image_raw"

output_path: "/home/zjs/lidar_imu_camera_ws/results/d435i/vins/run1"

#===============================
# Camera model
# 相机内参固定，不在线优化
#===============================
model_type: PINHOLE
camera_name: d435i_color

image_width: 640
image_height: 480

distortion_parameters:
   k1: 0.0
   k2: 0.0
   p1: 0.0
   p2: 0.0

projection_parameters:
   fx: 604.457275390625
   fy: 604.577331542969
   cx: 328.494903564453
   cy: 240.872558593750

#===============================
# Camera -> IMU factory extrinsic
# 1: use initial value and optimize
#===============================
estimate_extrinsic: 1

extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [0.999982,   -0.00550951,  -0.00243105,
          0.00550956,  0.999985,     0.0000109132,
          0.00243095, -0.000024307,  0.999997]

extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [-0.0203096531331539,
           0.00498574459925294,
           0.0112929595634341]

#===============================
# Feature tracker
#===============================
max_cnt: 180
min_dist: 25
freq: 20
F_threshold: 1.0
show_track: 1
flow_back: 1
equalize: 1
fisheye: 0

#===============================
# Nonlinear optimization
#===============================
max_solver_time: 0.04
max_num_iterations: 8
keyframe_parallax: 10.0

#===============================
# IMU noise
# 下列为经验初值，不是本机Allan测量结果
# 若VINS不稳定，应补做Allan方差或调大噪声
#===============================
acc_n: 0.02
gyr_n: 0.005
acc_w: 0.0002
gyr_w: 0.00005

g_norm: 9.80665

#===============================
# Loop closure
# 标定阶段关闭
#===============================
loop_closure: 0
load_previous_pose_graph: 0
fast_relocalization: 0

pose_graph_save_path: "/home/zjs/lidar_imu_camera_ws/results/d435i/vins/pose_graph"

#===============================
# Time offset
# image_timestamp + td = IMU time
#===============================
estimate_td: 1
td: 0.0

#===============================
# D435i RGB is rolling shutter
# 无准确读出时间时先关闭该模型
# 采集时必须平滑，避免高速角运动
#===============================
rolling_shutter: 0
rolling_shutter_tr: 0.0

#===============================
# Visualization
#===============================
save_image: 0
visualize_imu_forward: 0
visualize_camera_size: 0.4
```

注意：

- 先用 `/camera/color/camera_info` 的实际参数替换模板中的内参；
- 外参必须是 `Color → Gyro`；
- 不要用单位矩阵替代已有出厂外参；
- `estimate_extrinsic=1`，不能设置成无初值模式；
- 相机内参在 VINS-Mono 中保持固定。

---

# 12. 创建 VINS 启动文件

创建：

```bash
nano ~/lidar_imu_camera_ws/src/d435i_vins_calib/launch/d435i_vins_calib.launch
```

写入：

```xml
<launch>
  <arg name="config_path"
       default="$(find d435i_vins_calib)/config/d435i_vins.yaml"/>

  <arg name="vins_path"
       default="$(find feature_tracker)/../"/>

  <node name="feature_tracker"
        pkg="feature_tracker"
        type="feature_tracker"
        output="screen">

    <param name="config_file"
           type="string"
           value="$(arg config_path)"/>

    <param name="vins_folder"
           type="string"
           value="$(arg vins_path)"/>
  </node>

  <node name="vins_estimator"
        pkg="vins_estimator"
        type="vins_estimator"
        output="screen">

    <param name="config_file"
           type="string"
           value="$(arg config_path)"/>

    <param name="vins_folder"
           type="string"
           value="$(arg vins_path)"/>
  </node>
</launch>
```

重新编译：

```bash
cd ~/lidar_imu_camera_ws

catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

---

# 13. 录制 RGB–IMU 联合优化数据

## 13.1 启动 D435i

```bash
roslaunch d435i_vins_calib d435i_rgb_imu.launch
```

检查：

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/imu
```

检查 frame：

```bash
rostopic echo -n 1 /camera/color/image_raw/header
rostopic echo -n 1 /camera/imu/header
```

## 13.2 录制三组独立数据

Run 1：

```bash
rosbag record \
  --lz4 \
  -O ~/lidar_imu_camera_ws/bags/d435i/vins/d435i_vins_calib_run1.bag \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/imu \
  /tf \
  /tf_static
```

按同样方法录制：

```text
d435i_vins_calib_run2.bag
d435i_vins_calib_run3.bag
```

每组建议 `2～4 分钟`。

## 13.3 场景要求

推荐：

- 书架；
- 桌椅；
- 海报；
- 带文字的纸箱；
- 门框和墙角；
- 同时包含近景和远景。

避免：

- 纯白墙；
- 玻璃和镜面；
- 暗光；
- 高频闪烁灯；
- 大面积重复纹理；
- 大量动态人员；
- 只有单一平面。

## 13.4 运动要求

开始静止约 5 秒，随后依次完成：

```text
1. 绕 X 轴缓慢来回转动；
2. 绕 Y 轴缓慢来回转动；
3. 绕 Z 轴缓慢来回转动；
4. 左右平移；
5. 上下平移；
6. 前后平移；
7. 平移与旋转组合；
8. 最后静止约 5 秒。
```

注意：

- 不能只旋转；
- 不能只平移；
- 平移外参需要明显平移激励；
- 相机必须运动平滑；
- 避免运动模糊；
- D435i RGB 为滚动快门，不要快速甩动；
- USB 线不能牵拉相机；
- 尽量让特征覆盖图像中心、边缘和角落。

检查 rosbag：

```bash
rosbag info \
  ~/lidar_imu_camera_ws/bags/d435i/vins/d435i_vins_calib_run1.bag
```

---

# 14. 离线运行 VINS 优化

建议使用 rosbag 仿真时间。

## 14.1 终端 1：ROS Master

```bash
source ~/lidar_imu_camera_ws/devel/setup.bash
roscore
```

## 14.2 终端 2：启动 VINS

```bash
source ~/lidar_imu_camera_ws/devel/setup.bash

rosparam set use_sim_time true

roslaunch d435i_vins_calib d435i_vins_calib.launch \
  2>&1 | tee \
  ~/lidar_imu_camera_ws/results/d435i/vins/run1/vins_run1.log
```

## 14.3 终端 3：RViz

```bash
source ~/lidar_imu_camera_ws/devel/setup.bash
roslaunch vins_estimator vins_rviz.launch
```

## 14.4 终端 4：播放 rosbag

```bash
rosbag play \
  --clock \
  --pause \
  ~/lidar_imu_camera_ws/bags/d435i/vins/d435i_vins_calib_run1.bag
```

按空格开始。

计算机负载较高时：

```bash
rosbag play \
  --clock \
  -r 0.5 \
  ~/lidar_imu_camera_ws/bags/d435i/vins/d435i_vins_calib_run1.bag
```

降低播放速度不会改变传感器消息内部时间戳关系。

---

# 15. 读取优化结果

当：

```yaml
estimate_extrinsic: 1
```

时，VINS 通常会在 `output_path` 中输出：

```text
extrinsic_parameter.csv
```

查看：

```bash
cat \
  ~/lidar_imu_camera_ws/results/d435i/vins/run1/extrinsic_parameter.csv
```

持续观察：

```bash
watch -n 1 \
  tail -n 5 \
  ~/lidar_imu_camera_ws/results/d435i/vins/run1/extrinsic_parameter.csv
```

运行后半段的外参应逐渐稳定。

## 15.1 输出 `td`

部分 VINS-Mono 分支没有单独保存最终 `td`。可在：

```text
VINS-Mono/vins_estimator/src/estimator.cpp
```

找到：

```cpp
if (ESTIMATE_TD)
    td = para_Td[0][0];
```

改为：

```cpp
if (ESTIMATE_TD)
{
    td = para_Td[0][0];

    ROS_INFO_THROTTLE(
        1.0,
        "[D435I_CALIB] estimated td = %.9f s",
        td
    );
}
```

重新编译：

```bash
cd ~/lidar_imu_camera_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

查看：

```bash
grep "D435I_CALIB" \
  ~/lidar_imu_camera_ws/results/d435i/vins/run1/vins_run1.log \
  | tail -n 30
```

VINS 中时间偏移定义：

```text
t_imu = t_image + td
```

---

# 16. 三组独立结果验证

每组数据都必须：

1. 重新启动 VINS；
2. 修改不同 `output_path`；
3. 播放对应 rosbag；
4. 记录收敛后的 `R_imu_cam、t_imu_cam、td`。

Run 1：

```yaml
output_path: "/home/zjs/lidar_imu_camera_ws/results/d435i/vins/run1"
```

Run 2：

```yaml
output_path: "/home/zjs/lidar_imu_camera_ws/results/d435i/vins/run2"
```

Run 3：

```yaml
output_path: "/home/zjs/lidar_imu_camera_ws/results/d435i/vins/run3"
```

工程参考，不是官方硬性指标：

```text
三次旋转外参差异：最好小于 0.5°～1°
三次平移外参差异：最好小于 5～10 mm
三次 td 差异：最好小于 2～5 ms
```

不要对三个旋转矩阵逐元素做普通算术平均。优先选择：

- 初始化正常；
- 特征持续充足；
- 轨迹无跳变；
- 外参后半段收敛；
- `td` 波动最小；
- 结果与出厂外参接近；

的一组作为最终结果。

若要做多组旋转融合，应使用四元数平均或李群上的旋转平均。

---

# 17. 判断优化是否合理

正常情况：

```text
旋转相对出厂值仅小幅修正；
平移为毫米级或较小厘米级修正；
td 逐渐稳定；
VINS 初始化成功；
轨迹连续，无频繁重置。
```

异常情况：

```text
平移突然变成十几厘米；
旋转偏离出厂值十几度；
td 持续单向增长；
三组结果完全不一致；
轨迹反复重置；
出现大量 IMU message in disorder。
```

重点排查：

```bash
rostopic hz /camera/imu
rostopic hz /camera/color/image_raw
rostopic echo /camera/imu/header
rostopic echo /camera/color/image_raw/header
```

同时确认：

```yaml
unite_imu_method: linear_interpolation
estimate_extrinsic: 1
estimate_td: 1
```

以及外参方向必须是：

```text
T_imu_cam = Color → Gyro
```

---

# 18. 固化最终配置

复制配置：

```bash
cp \
  ~/lidar_imu_camera_ws/src/d435i_vins_calib/config/d435i_vins.yaml \
  ~/lidar_imu_camera_ws/src/d435i_vins_calib/config/d435i_vins_final.yaml
```

编辑：

```bash
nano \
  ~/lidar_imu_camera_ws/src/d435i_vins_calib/config/d435i_vins_final.yaml
```

将：

```yaml
estimate_extrinsic: 1
estimate_td: 1
```

改为：

```yaml
estimate_extrinsic: 0
estimate_td: 0
```

填入最终：

```yaml
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [最终r11, 最终r12, 最终r13,
          最终r21, 最终r22, 最终r23,
          最终r31, 最终r32, 最终r33]

extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [最终tx, 最终ty, 最终tz]

td: 最终时间偏移
```

正式运行时关闭在线外参和时间偏移优化，避免在弱纹理或低激励场景中把已经确定的参数拖偏。

---

# 19. 关于三小时 Allan 方差

本流程默认不做三小时 Allan。

当前 VINS 配置中的：

```yaml
acc_n: 0.02
gyr_n: 0.005
acc_w: 0.0002
gyr_w: 0.00005
```

只是经验初值，不是本机测量值。

后续出现以下情况时，建议补做 Allan 方差：

- VINS 初始化经常失败；
- 静止时轨迹明显漂移；
- 轨迹高频抖动；
- 优化结果对噪声参数非常敏感；
- 需要论文级、可重复的噪声模型；
- 需要长时间高精度视觉惯性里程计。

Allan 方差通常需要：

```text
预热 10～15 分钟
静止固定
连续录制约 3 小时
```

它得到的是：

```text
accelerometer_noise_density
accelerometer_random_walk
gyroscope_noise_density
gyroscope_random_walk
```

这些与已经写入 EEPROM 的 Accel/Gyro Motion Intrinsic 不是同一类参数。

---

# 20. 常见问题排查

## 20.1 `/camera/color/camera_info` 没反应

`rostopic echo` 会等待发布者。先启动：

```bash
roslaunch d435i_vins_calib d435i_rgb_imu.launch
```

然后：

```bash
rostopic echo -n 1 /camera/color/camera_info
```

## 20.2 `Depth stream start failure`

当前流程不需要深度，关闭：

```bash
enable_depth:=false
```

若仍出现 USB 错误：

- 换主板 USB 3.x 接口；
- 不使用 HUB；
- 检查 `lsusb -t`；
- 固定 USB 线；
- 降低其他设备带宽。

## 20.3 `control_transfer returned error`

偶发一次可以观察；若持续高频出现并伴随流中断，应：

1. 停止所有 RealSense 程序；
2. 拔插设备；
3. 更换 USB 口或线；
4. 关闭 Depth；
5. 避免多个进程同时访问 D435i。

## 20.4 标定写入后程序核心转储

只要先出现：

```text
SUCCESS: saved calibration to camera.
Done.
```

且重新插拔后 Motion Intrinsic 已更新，则写入成功。后面的 Qt 清理异常不影响 EEPROM 结果。

## 20.5 D435i 发热

运行 RGB、Depth、IMU 或标定时发热通常正常。建议：

- 保持通风；
- 不用布完全包裹设备；
- 不让相机紧贴热源；
- 写入完成后可断电冷却；
- 若出现异味、频繁断连或无法触摸，应立即断电检查。

## 20.6 相机内参和 `rs-enumerate-devices` 不一致

优先使用与实际 ROS 图像话题对应的：

```text
/camera/color/camera_info
```

确认：

- 分辨率一致；
- 图像话题一致；
- 不混用 RGB、Depth、IR 内参；
- 不将 1280×720 内参用于 640×480 图像。

## 20.7 VINS 外参方向错误

VINS 配置使用：

```text
Color → Gyro
T_imu_cam
```

不是：

```text
Gyro → Color
T_cam_imu
```

方向错误通常会导致：

- 初始化失败；
- 轨迹姿态异常；
- 平移快速发散；
- 外参估计出现不合理大幅变化。

## 20.8 `/camera/imu` 话题存在但无数据（0 帧）

### 现象

`rostopic hz /camera/imu` 显示：

```text
subscribed to [/camera/imu]
no new messages
```

话题能订阅、发布者存在，但始终没有消息；`/camera/color/image_raw` 正常 30Hz。这是本机实际遇到并解决的问题。

### 排查链路

1. **话题不存在**：`rs_camera.launch` 中 `enable_gyro`、`enable_accel` 默认都是 `false`。不带这两个参数启动，IMU 流根本不开，话题列表里没有任何 `/camera/imu`。
2. **话题存在但 0 帧**：加上 `enable_gyro:=true enable_accel:=true` 后话题出现，但仍无数据；原始 `/camera/gyro/sample`、`/camera/accel/sample` 也是 0 帧。
3. **librealsense 层 0 帧**：用 `rs-motion` 或最小 C++ 程序直接打开 Motion Module，传感器能枚举、打开成功，但 3 秒内 0 帧。
4. **内核日志**：`dmesg` 反复出现：

   ```text
   hid-sensor-hub 0003:8086:0B3A.xxxx: No report with id 0xffffffff found
   ```

5. **IIO 设备读 0**：

   ```bash
   cat /sys/bus/iio/devices/iio:device0/in_accel_x_raw   # 连续读都是 0
   ```

6. **采样频率异常**：nodelet 打开 `/dev/iio:device0` 后（`fuser /dev/iio:device0` 可见），`in_accel_sampling_frequency` 只有 3Hz、`in_anglvel_sampling_frequency` 只有 2Hz，远低于请求的 250/200。

### 根因

两个问题叠加：

**（1）ROS 驱动依赖的 librealsense 2.50 没有 IIO 后端**

本机 `realsense2_camera 2.3.2` 由 ROS apt 提供，链接的是 `ros-noetic-librealsense2`（librealsense v2.50.0）：

```bash
# 验证 2.50 无 IIO 后端
strings /opt/ros/noetic/lib/x86_64-linux-gnu/librealsense2.so.2.50.0 \
  | grep "iio:device"        # 无输出
```

librealsense 2.58.3 及以后的 Linux 后端通过内核 `hid_sensor_hub` 驱动创建的 IIO 设备读取 D435i IMU：

```bash
strings /usr/lib/x86_64-linux-gnu/librealsense2.so.2.58.3 \
  | grep "iio:device"        # 有输出
ls /sys/bus/iio/devices/     # iio:device0 = accel_3d, iio:device1 = gyro_3d
```

2.50 没有该后端，只能尝试用 libusb 认领相机的 HID USB 接口（`if5`），但该接口被内核 `usbhid`/`hid_sensor_hub` 占用，`libusb_claim_interface(5)` 返回 `LIBUSB_ERROR_BUSY`，因此 0 帧。

**（2）曾误屏蔽 `hid_sensor_hub`**

排查时误以为 `hid_sensor_hub` 是"抢占接口的坏驱动"，将其 blacklist 并卸载。结果 Motion Module 直接从 librealsense 传感器列表消失（因为 IIO 设备没了）。**`hid_sensor_hub` 是 2.58.3 读取 IMU 的必需通路，不能屏蔽。**

### 解决步骤

```bash
# 1. 撤销对 hid_sensor_hub 的错误屏蔽（若做过）
sudo rm -f /etc/modprobe.d/realsense-hid.conf
sudo modprobe hid_sensor_hub hid_sensor_trigger hid_sensor_accel_3d hid_sensor_gyro_3d hid_sensor_custom

# 2. 将 ROS 的 librealsense 2.50 替换为 2.58.3（ABI 兼容已验证：nodelet 需要的 rs2_* 符号 2.58.3 全部提供）
sudo mv /opt/ros/noetic/lib/x86_64-linux-gnu/librealsense2.so.2.50 \
      /opt/ros/noetic/lib/x86_64-linux-gnu/librealsense2.so.2.50.orig
sudo ln -s /usr/lib/x86_64-linux-gnu/librealsense2.so.2.58.3 \
      /opt/ros/noetic/lib/x86_64-linux-gnu/librealsense2.so.2.50

# 3. 重启电脑（清理当天反复装卸内核模块造成的坏状态），然后启动相机
```

启动并验证：

```bash
roslaunch realsense2_camera rs_camera.launch \
  color_width:=640 color_height:=480 color_fps:=30 \
  enable_depth:=false \
  enable_gyro:=true enable_accel:=true \
  gyro_fps:=200 accel_fps:=250 \
  unite_imu_method:=linear_interpolation

rostopic hz /camera/imu    # 预期 ~200Hz
```

日志中确认：

```text
Running with LibRealSense v2.58.3
Motion Module was found.
gyro stream is enabled - fps: 200
accel stream is enabled - fps: 250
```

### 注意事项

- **不要 blacklist `hid_sensor_hub`**，它是 2.58.3 通过 IIO 读取 IMU 的必需驱动；
- **`accel_fps` 不要设 200**：D435i 加速度计只支持 250/63 Hz，设 200 会回退默认档位并告警 `No matching profile found for accel with fps=200`；
- 出现 `Hardware Notification: Motion Module failure` 但 `/camera/imu` 稳定 200Hz 时，属于启动瞬态通知，可忽略；
- 替换 librealsense 只影响 `realsense2_camera` 节点，可随时回退：

  ```bash
  sudo mv /opt/ros/noetic/lib/x86_64-linux-gnu/librealsense2.so.2.50.orig \
        /opt/ros/noetic/lib/x86_64-linux-gnu/librealsense2.so.2.50
  ```

---

# 21. 最终验收清单

## D435i IMU

- [x] 六面数据均显示 `Direction data collected.`
- [x] `norm(fixed data)` 接近 `9.806650`
- [x] 出现 `SUCCESS: saved calibration to camera.`
- [x] 重新插拔后设备正常识别
- [x] `rs-enumerate-devices -c` 显示新的 Motion Intrinsic

## RGB 相机

- [x] 固定为 `640×480@30Hz`
- [x] `/camera/color/image_raw` 正常
- [x] `/camera/color/camera_info` 已保存
- [x] 内参与当前分辨率一致

## RGB–IMU 外参

- [x] 已保存出厂 `Color → Gyro`
- [ ] 已录制至少三组 RGB+IMU 数据
- [ ] VINS `estimate_extrinsic=1`
- [ ] VINS `estimate_td=1`
- [ ] 三组结果一致性检查通过
- [ ] 最终配置关闭在线优化

---

# 22. 本机关键参数汇总

## 设备

```yaml
device: RealSense D435I
serial_number: "109622073033"
firmware: "5.16.0.1"
imu_type: BMI055
usb_type: "3.2"
```

## RGB 640×480 出厂内参参考

```yaml
image_width: 640
image_height: 480

fx: 604.457275390625
fy: 604.577331542969
cx: 328.494903564453
cy: 240.872558593750

k1: 0.0
k2: 0.0
p1: 0.0
p2: 0.0
```

## Color → Gyro 出厂外参

```yaml
R_imu_cam:
  - [0.999982,   -0.00550951,  -0.00243105]
  - [0.00550956,  0.999985,     0.0000109132]
  - [0.00243095, -0.000024307,  0.999997]

t_imu_cam:
  - -0.0203096531331539
  -  0.00498574459925294
  -  0.0112929595634341
```

## 标定后 Gyro Motion Intrinsic

```yaml
gyro_sensitivity:
  - [1.0, 0.0, 0.0, -0.000039]
  - [0.0, 1.0, 0.0, -0.000012]
  - [0.0, 0.0, 1.0, -0.000027]
```

## 标定后 Accel Motion Intrinsic

```yaml
accel_sensitivity:
  - [ 1.016180,  0.007751,  0.000911, -0.064920]
  - [ 0.010672,  1.009747, -0.053151,  0.254651]
  - [-0.062725,  0.048204,  1.023140,  0.065594]
```

---

# 23. 参考项目

- librealsense：`https://github.com/realsenseai/librealsense`
- RealSense ROS：`https://github.com/IntelRealSense/realsense-ros`
- VINS-Mono：`https://github.com/HKUST-Aerial-Robotics/VINS-Mono`
- Allan Variance ROS：`https://github.com/ori-drs/allan_variance_ros`

---

# 24. 最终执行命令速查

```bash
# 1. 启动固定RGB+IMU
roslaunch d435i_vins_calib d435i_rgb_imu.launch

# 2. 检查频率
rostopic hz /camera/color/image_raw
rostopic hz /camera/imu

# 3. 保存相机内参
rostopic echo -n 1 /camera/color/camera_info \
  > ~/lidar_imu_camera_ws/results/d435i/factory/color_camera_info_640x480.yaml

# 4. 保存设备完整参数
rs-enumerate-devices -c \
  > ~/lidar_imu_camera_ws/results/d435i/factory/device_info_final.txt

# 5. 录制Run 1
rosbag record \
  --lz4 \
  -O ~/lidar_imu_camera_ws/bags/d435i/vins/d435i_vins_calib_run1.bag \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/imu \
  /tf \
  /tf_static

# 6. 启动VINS
rosparam set use_sim_time true
roslaunch d435i_vins_calib d435i_vins_calib.launch

# 7. 播放数据
rosbag play \
  --clock \
  ~/lidar_imu_camera_ws/bags/d435i/vins/d435i_vins_calib_run1.bag

# 8. 查看外参
cat ~/lidar_imu_camera_ws/results/d435i/vins/run1/extrinsic_parameter.csv
```
