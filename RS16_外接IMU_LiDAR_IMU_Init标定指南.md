# RoboSense RS-LiDAR-16 与外接 IMU 联合标定指南

> **适用环境**
>
> - Ubuntu 20.04
> - ROS Noetic
> - 工作空间：`~/lidar_imu_ws`
> - 激光雷达：RoboSense RS-LiDAR-16
> - IMU：超核 HiPNUC 或其他外接 IMU
> - 标定工具：[hku-mars/LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init)
> - 雷达驱动：[RoboSense-LiDAR/rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)
标定的时候记得：rosparam set /use_sim_time false： 关掉仿真时间


本文是在已经搭建好 `~/lidar_imu_ws`、超核 IMU 驱动和 `LiDAR_IMU_Init` 的基础上，将原来的 Livox Mid-360 替换为 RoboSense RS-LiDAR-16，并完成 LiDAR–IMU 时间偏移、旋转外参和平移外参标定。

---

## 1. 标定系统结构

```text
RoboSense RS-LiDAR-16
        │ UDP
        ▼
    rslidar_sdk
        │ /rslidar_points
        │ sensor_msgs/PointCloud2
        │ x、y、z、intensity、ring、timestamp
        ▼
 LiDAR_IMU_Init
        ▲
        │ /imu/data 或其他实际 IMU 话题
        │ sensor_msgs/Imu
        │
外接九轴 IMU
```

最终标定内容包括：

- LiDAR 与 IMU 的旋转外参；
- LiDAR 与 IMU 的平移外参；
- LiDAR 与 IMU 的固定时间偏移；
- 重力方向；
- IMU 零偏初值。

---

## 2. 标定前的硬件要求

### 2.1 刚性固定

RS16 和 IMU 必须安装在同一个刚性支架上。标定过程中要求：

- 雷达和 IMU 之间不能发生相对移动；
- 不能只使用松动胶带临时粘贴；
- 支架不能明显弯曲；
- 标定完成后不能改变安装角度和位置；
- 移动时必须拿着整套支架一起运动。

### 2.2 是否必须硬件同步

运行 `LiDAR_IMU_Init` 不强制要求提前完成 PPS、PTP 或 GPS 硬件同步，它可以估计固定时间偏移。

没有硬件同步时需要满足：

- LiDAR 和 IMU 时间戳均非零；
- 时间戳持续递增且不回跳；
- 两个驱动的延迟基本稳定；
- 最好让雷达和 IMU 都使用同一台电脑的 ROS 系统时间。

若 RS16 和 IMU 均接入相同 PPS/GPS 时间源，才适合让两者都使用硬件时间。

---

## 3. 添加 RoboSense 驱动

```bash
source /opt/ros/noetic/setup.bash
cd ~/lidar_imu_ws/src

git clone --recursive https://github.com/RoboSense-LiDAR/rslidar_sdk.git
```

若之前已经克隆但缺少子模块：

```bash
cd ~/lidar_imu_ws/src/rslidar_sdk
git submodule update --init --recursive
```

安装依赖：

```bash
sudo apt update
sudo apt install -y \
  libyaml-cpp-dev \
  libpcap-dev \
  libeigen3-dev \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions
```

---

## 4. 将点类型改为 XYZIRT

`LiDAR_IMU_Init` 的 RoboSense 预处理需要：

```text
x
y
z
intensity
ring
timestamp
```

修改：

```bash
nano ~/lidar_imu_ws/src/rslidar_sdk/CMakeLists.txt
```

把：

```cmake
set(POINT_TYPE XYZI)
```

改为：

```cmake
set(POINT_TYPE XYZIRT)
```

也可以直接执行：

```bash
sed -i \
  's/set(POINT_TYPE XYZI)/set(POINT_TYPE XYZIRT)/' \
  ~/lidar_imu_ws/src/rslidar_sdk/CMakeLists.txt
```

确认：

```bash
grep -n "set(POINT_TYPE" \
  ~/lidar_imu_ws/src/rslidar_sdk/CMakeLists.txt
```

应看到：

```text
set(POINT_TYPE XYZIRT)
```

修改后必须重新编译，否则点云中仍然没有 `ring` 和逐点 `timestamp`。

---

## 5. 配置 RS-LiDAR-16 驱动

编辑：

```bash
nano ~/lidar_imu_ws/src/rslidar_sdk/config/config.yaml
```

单台 RS16 的关键配置：

```yaml
common:
  msg_source: 1
  send_packet_ros: false
  send_point_cloud_ros: true

lidar:
  - driver:
      lidar_type: RS16
      msop_port: 6699
      difop_port: 7788
      imu_port: 0

      min_distance: 0.4
      max_distance: 200.0

      # 无硬件同步且外接IMU采用ROS主机时间时，先设为false
      use_lidar_clock: false

      dense_points: false
      ts_first_point: true
      start_angle: 0
      end_angle: 360

    ros:
      ros_frame_id: rslidar
      ros_recv_packet_topic: /rslidar_packets
      ros_send_packet_topic: /rslidar_packets
      ros_send_imu_data_topic: /rslidar_imu_data
      ros_send_point_cloud_topic: /rslidar_points
```

### `use_lidar_clock` 的选择

没有硬件同步，且外接 IMU 驱动使用主机 ROS 时间时：

```yaml
use_lidar_clock: false
```

RS16 和 IMU 已确认共用 PPS/GPS 硬件时间，并且两个驱动都发布设备时间时：

```yaml
use_lidar_clock: true
```

不能只因为雷达支持 GPS 时间就直接设为 `true`。如果 IMU 仍使用电脑接收时间，两路时间基准可能不一致。

---

## 6. 编译工作空间

```bash
source /opt/ros/noetic/setup.bash
cd ~/lidar_imu_ws

rm -rf build devel

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y

catkin_make -DCMAKE_BUILD_TYPE=Release -j4
```

加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/lidar_imu_ws/devel/setup.bash
```

检查：

```bash
rospack find rslidar_sdk
rospack find lidar_imu_init
rospack find hipnuc_imu
rospack find hipnuc_lib_package
```

---

## 7. 环境加载脚本

```bash
cat > ~/lidar_imu_ws/setup_env.sh <<'SCRIPT'
#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source "$HOME/lidar_imu_ws/devel/setup.bash"
SCRIPT

chmod +x ~/lidar_imu_ws/setup_env.sh
```

每个新终端执行：

```bash
source ~/lidar_imu_ws/setup_env.sh
```

不要同时 source 多个包含同名包的 catkin 工作空间。

---

## 8. 配置外接 IMU

### 8.1 超核 HiPNUC

编辑：

```bash
nano ~/lidar_imu_ws/src/hipnuc_imu/config/hipnuc_config.yaml
```

示例：

```yaml
# hipnuc config
imu_serial: "/dev/ttyUSB0"
baud_rate: 115200
frame_id: "imu_link"
imu_topic: "/imu/data"

# hipnuc data package
frame_id_costom: "base_link_hipnuc"
imu_topic_costom: "/imu_package_hipnuc"
```

注意：

- `/imu/data`、`/IMU/data` 和 `/IMU_data` 是三个不同话题；
- ROS 话题名区分大小写；
- `costom` 是超核驱动沿用的参数拼写，不要自行改成 `custom`。

检查串口：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
sudo chmod 666 /dev/ttyUSB0
```

永久加入串口组：

```bash
sudo usermod -aG dialout "$USER"
```

启动：

```bash
source ~/lidar_imu_ws/setup_env.sh
roslaunch hipnuc_imu imu_msg.launch
```

检查实际话题：

```bash
rostopic list | grep -Ei "imu"
rostopic type /imu/data
rostopic info /imu/data
rostopic hz /imu/data
rostopic echo -n 1 /imu/data
```

### 8.2 其他 IMU

其他 IMU 必须发布：

```text
sensor_msgs/Imu
```

至少提供：

```text
header.stamp
angular_velocity.x/y/z
linear_acceleration.x/y/z
```

要求：

- 角速度单位为 `rad/s`；
- 加速度可以是 `m/s²` 或 `g`，但配置必须匹配；
- 时间戳非零并稳定递增；
- 数据频率建议不低于 100 Hz；
- 若发布厂商自定义消息，需要先转换为 `sensor_msgs/Imu`。

---

## 9. 测量 IMU 静止加速度模长

让 IMU 静止，按实际话题修改 `topic`：

```bash
python3 - <<'PY'
import math
import statistics
import rospy
from sensor_msgs.msg import Imu

topic = "/imu/data"
rospy.init_node("check_imu_acc_norm", anonymous=True)

values = []
for _ in range(200):
    msg = rospy.wait_for_message(topic, Imu, timeout=3.0)
    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    az = msg.linear_acceleration.z
    values.append(math.sqrt(ax * ax + ay * ay + az * az))

print("topic:", topic)
print("samples:", len(values))
print("mean acceleration norm:", statistics.mean(values))
print("min:", min(values))
print("max:", max(values))
PY
```

接近 `9.8`：

```yaml
mean_acc_norm: 9.81
```

接近 `1.0`：

```yaml
mean_acc_norm: 1.0
```

---

## 10. 启动并检查 RS16

连接雷达并按设备当前网段配置有线网卡。检查：

```bash
ip addr
```

启动驱动：

```bash
source ~/lidar_imu_ws/setup_env.sh
roslaunch rslidar_sdk start.launch
```

检查：

```bash
rostopic list | grep -Ei "rslidar"
rostopic type /rslidar_points
rostopic info /rslidar_points
rostopic hz /rslidar_points
```

期望类型：

```text
sensor_msgs/PointCloud2
```

---

## 11. 检查点云字段

```bash
rostopic echo -n 1 /rslidar_points/fields
```

必须至少包含：

```text
x
y
z
intensity
ring
timestamp
```

`ring` 应为 `UINT16`，`timestamp` 应为 `FLOAT64`。

如果只有：

```text
x
y
z
intensity
```

说明驱动仍然是 `XYZI`，需要确认并重新编译：

```bash
grep -n "set(POINT_TYPE" \
  ~/lidar_imu_ws/src/rslidar_sdk/CMakeLists.txt

cd ~/lidar_imu_ws
rm -rf build devel
catkin_make -DCMAKE_BUILD_TYPE=Release -j4
source devel/setup.bash
```

---

## 12. 检查 LiDAR 与 IMU 时间戳

```bash
rostopic echo -n 5 /rslidar_points/header/stamp
rostopic echo -n 5 /imu/data/header/stamp
```

要求：

- 两路时间戳均非零；
- 均持续增加；
- 不发生频繁回跳；
- 秒级数值处于同一时间基准附近；
- 不能一边是 Unix 时间，另一边是从零开始的设备启动时间。

粗略检查最新时间差：

```bash
python3 - <<'PY'
import rospy
from sensor_msgs.msg import Imu, PointCloud2

rospy.init_node("check_lidar_imu_stamp", anonymous=True)

pc = rospy.wait_for_message("/rslidar_points", PointCloud2, timeout=5.0)
imu = rospy.wait_for_message("/imu/data", Imu, timeout=5.0)

t_lidar = pc.header.stamp.to_sec()
t_imu = imu.header.stamp.to_sec()

print("LiDAR stamp:", t_lidar)
print("IMU stamp:  ", t_imu)
print("Difference IMU-LiDAR:", t_imu - t_lidar, "s")
PY
```

该结果只是粗略检查，不等于最终标定时间偏移。

---

## 13. 配置 LiDAR_IMU_Init

编辑：

```bash
nano ~/lidar_imu_ws/src/LiDAR_IMU_Init/config/robosense.yaml
```

RS16 + 外接 IMU 推荐初始配置：

```yaml
common:
  lid_topic: "/rslidar_points"
  imu_topic: "/imu/data"

preprocess:
  lidar_type: 6
  scan_line: 16
  blind: 1.0
  feature_extract_en: false

initialization:
  cut_frame_num: 3
  orig_odom_freq: 10
  mean_acc_norm: 9.81
  online_refine_time: 20.0
  data_accum_length: 500
  Rot_LI_cov: [0.00005, 0.00005, 0.00005]
  Trans_LI_cov: [0.0001, 0.0001, 0.0001]

mapping:
  filter_size_surf: 0.05
  filter_size_map: 0.15
  gyr_cov: 0.5
  acc_cov: 0.5
  b_acc_cov: 0.0001
  b_gyr_cov: 0.0001
  det_range: 100.0

publish:
  path_en: true
  scan_publish_en: true
  dense_publish_en: true
  scan_bodyframe_pub_en: true

pcd_save:
  pcd_save_en: false
  interval: -1
```

最重要的是：

```yaml
lid_topic: "/rslidar_points"
imu_topic: "/imu/data"
lidar_type: 6
scan_line: 16
```

官方模板中的 `scan_line: 128` 是通用示例，RS-LiDAR-16 必须改为 `16`。

检查：

```bash
grep -nE \
  "lid_topic|imu_topic|lidar_type|scan_line|mean_acc_norm|orig_odom_freq|cut_frame_num" \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/config/robosense.yaml
```

---

## 14. 实时标定

### 终端 1：IMU

```bash
source ~/lidar_imu_ws/setup_env.sh
sudo chmod 666 /dev/ttyUSB0
roslaunch hipnuc_imu imu_msg.launch
```

### 终端 2：RS16

```bash
source ~/lidar_imu_ws/setup_env.sh
roslaunch rslidar_sdk start.launch
```

### 终端 3：检查数据

```bash
source ~/lidar_imu_ws/setup_env.sh

rostopic type /rslidar_points
rostopic hz /rslidar_points

rostopic type /imu/data
rostopic hz /imu/data
```

### 终端 4：LI-Init

```bash
source ~/lidar_imu_ws/setup_env.sh
roslaunch lidar_imu_init robosense.launch rviz:=true
```

---

## 15. 先录包再离线标定

创建目录：

```bash
mkdir -p ~/lidar_imu_ws/bags
```

录制：

```bash
source ~/lidar_imu_ws/setup_env.sh

rosbag record \
  -O ~/lidar_imu_ws/bags/rs16_imu_calib.bag \
  /rslidar_points \
  /imu/data
```

如果实际 IMU 话题是 `/IMU/data`，就把命令中的 `/imu/data` 替换为 `/IMU/data`。

建议录制 60～90 秒。

---

## 16. 标定动作

1. 开始录制后静止 5～10 秒；
2. 绕 X 轴反复俯仰；
3. 绕 Y 轴反复侧倾；
4. 绕 Z 轴反复旋转；
5. 前后、左右、上下平移；
6. 最后做旋转和平移组合运动。

不要只做：

- 水平绕 Z 轴旋转；
- 单一直线运动；
- 小幅高频抖动；
- 始终保持雷达水平；
- 纯空旷环境运动。

建议在有墙面、墙角、桌椅、箱子或立柱的室内环境中采集。

---

## 17. 检查 rosbag

```bash
rosbag info ~/lidar_imu_ws/bags/rs16_imu_calib.bag
```

应看到：

```text
/rslidar_points    sensor_msgs/PointCloud2
/imu/data          sensor_msgs/Imu
```

两路消息数量均不能为零。

---

## 18. 离线标定

离线播放前关闭实物 `rslidar_sdk` 和 IMU 驱动，避免同一话题出现多个发布者。

### 终端 1：启动 LI-Init

```bash
source ~/lidar_imu_ws/setup_env.sh
roslaunch lidar_imu_init robosense.launch rviz:=true
```

### 终端 2：播放 rosbag

```bash
source ~/lidar_imu_ws/setup_env.sh

rosbag play \
  ~/lidar_imu_ws/bags/rs16_imu_calib.bag \
  --pause
```

按空格开始播放。

处理不过来时：

```bash
rosbag play \
  ~/lidar_imu_ws/bags/rs16_imu_calib.bag \
  -r 0.5
```

降低播放速度不会修改消息中的原始 `header.stamp`。

---

## 19. 查看结果

```bash
cat \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
```

持续查看：

```bash
watch -n 1 \
  cat ~/lidar_imu_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
```

建议完整标定三次。三次结果相差很大时，优先检查：

- 时间戳稳定性；
- IMU 单位；
- 点云逐点时间；
- `scan_line: 16`；
- 运动激励；
- 支架刚性；
- 环境几何特征。

---

## 20. 写入 FAST-LIO 时的注意事项

常见配置形式：

```yaml
mapping:
  extrinsic_est_en: false
  extrinsic_T: [tx, ty, tz]
  extrinsic_R: [
    r00, r01, r02,
    r10, r11, r12,
    r20, r21, r22
  ]
```

时间偏移参数在不同分支中可能叫：

```text
time_offset_lidar_to_imu
time_lag_imu_to_lidar
```

必须对照所用 FAST-LIO 分支的源码确认外参方向和时间偏移正负号，不要只根据参数名猜测，也不要盲目对旋转矩阵求逆。

外参已经标定后，通常设置：

```yaml
extrinsic_est_en: false
```

---

## 21. 常见问题

### 21.1 `Failed to find match for field 'ring'`

点云仍为 `XYZI` 或未重新编译。

```bash
rostopic echo -n 1 /rslidar_points/fields
```

修复：

```bash
sed -i \
  's/set(POINT_TYPE XYZI)/set(POINT_TYPE XYZIRT)/' \
  ~/lidar_imu_ws/src/rslidar_sdk/CMakeLists.txt

cd ~/lidar_imu_ws
rm -rf build devel
catkin_make -DCMAKE_BUILD_TYPE=Release -j4
source devel/setup.bash
```

### 21.2 `Failed to find match for field 'timestamp'`

点云必须包含逐点 `timestamp`，只有整帧 `header.stamp` 不够。

### 21.3 LI-Init 没有反应

```bash
rostopic list | grep -Ei "rslidar|imu"
rostopic info /rslidar_points
rostopic info /imu/data
rostopic hz /rslidar_points
rostopic hz /imu/data
```

再检查：

```bash
grep -nE "lid_topic|imu_topic" \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/config/robosense.yaml
```

### 21.4 IMU 话题存在但没有数据

```bash
rostopic info /imu/data
```

若为：

```text
Publishers: None
```

说明只有订阅者，没有 IMU 驱动实际发布。

### 21.5 地图旋转时严重重影

重点检查：

1. 点云是否有 `timestamp`；
2. `timestamp` 是否为 `FLOAT64`；
3. 两路时间是否同一基准；
4. `use_lidar_clock` 是否合适；
5. 角速度是否为 `rad/s`；
6. `scan_line` 是否为 `16`；
7. 支架是否刚性固定。

### 21.6 初始化长时间无法完成

可能原因：

- 开始时没有静止；
- 只绕 Z 轴；
- 缺少上下平移；
- 环境过于空旷；
- IMU 噪声大；
- 时间戳不稳定；
- `mean_acc_norm` 错误。

### 21.7 离线播放数据混乱

检查是否有多个发布者：

```bash
rostopic info /rslidar_points
rostopic info /imu/data
```

播放 rosbag 时通常只应有 `rosbag play` 一个发布者。

---

## 22. 一键检查

```bash
source ~/lidar_imu_ws/setup_env.sh

echo "========== Packages =========="
rospack find rslidar_sdk
rospack find lidar_imu_init

echo "========== Topic types =========="
rostopic type /rslidar_points
rostopic type /imu/data

echo "========== Publishers =========="
rostopic info /rslidar_points
rostopic info /imu/data

echo "========== Point fields =========="
rostopic echo -n 1 /rslidar_points/fields

echo "========== LI-Init config =========="
grep -nE \
  "lid_topic|imu_topic|lidar_type|scan_line|mean_acc_norm|orig_odom_freq|cut_frame_num" \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/config/robosense.yaml
```

最终应满足：

```text
/rslidar_points → sensor_msgs/PointCloud2
/imu/data       → sensor_msgs/Imu

点云字段：
x、y、z、intensity、ring、timestamp

LiDAR_IMU_Init：
lidar_type = 6
scan_line = 16
```

---

## 23. 从 Mid-360 切换到 RS16 的差异

| 项目 | Mid-360 | RS-LiDAR-16 |
|---|---|---|
| 雷达驱动 | `livox_ros_driver2` | `rslidar_sdk` |
| 点云话题 | `/livox/lidar` | `/rslidar_points` |
| 点云类型 | `livox_ros_driver2/CustomMsg` | `sensor_msgs/PointCloud2` |
| LI-Init launch | `livox_mid360.launch` | `robosense.launch` |
| `lidar_type` | `1` | `6` |
| `scan_line` | `6` | `16` |
| 点时间字段 | `offset_time` | `timestamp` |
| 点线束字段 | `line` | `ring` |
| rosbag 点云话题 | `/livox/lidar` | `/rslidar_points` |

外接 IMU 不变时，继续使用实际 IMU 话题，并重新标定 RS16 与 IMU 的外参。

---

## 24. 参考资料

1. [LiDAR_IMU_Init 官方仓库](https://github.com/hku-mars/LiDAR_IMU_Init)
2. [LiDAR_IMU_Init RoboSense 配置](https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/config/robosense.yaml)
3. [LiDAR_IMU_Init RoboSense 启动文件](https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/launch/robosense.launch)
4. [RoboSense rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)
5. [rslidar_sdk 点类型配置](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/doc/howto/05_how_to_change_point_type.md)
6. [rslidar_sdk 在线雷达配置](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/doc/howto/06_how_to_decode_online_lidar.md)
7. [超核 HiPNUC ROS1 示例](https://github.com/hipnuc/products/blob/master/examples/ROS_Melodic/README-IMU.md)
