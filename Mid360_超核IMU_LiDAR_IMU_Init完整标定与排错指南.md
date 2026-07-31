# Mid-360 与超核外接 IMU：LiDAR_IMU_Init 完整标定与排错指南

下面是一套从零开始的完整命令，工作空间统一为：

```bash
~/lidar_imu_ws
```

最终包含：

```text
lidar_imu_ws/
├── src/
│   ├── livox_ros_driver2
│   ├── hipnuc_imu
│   ├── hipnuc_lib_package
│   └── LiDAR_IMU_Init
└── third_party/
    ├── Livox-SDK2
    └── hipnuc_products
```

`livox_ros_driver2` 必须位于 catkin 工作空间的 `src` 目录；Mid-360 标定需要使用 `msg_MID360.launch` 发布 `CustomMsg`。超核 ROS1 官方示例允许把 `hipnuc_imu` 包复制到其他 ROS1 工作空间编译。([GitHub][1])

---

# 一、安装基础依赖

```bash
sudo apt update

sudo apt install -y \
  git \
  cmake \
  build-essential \
  python3-dev \
  python3-matplotlib \
  libeigen3-dev \
  libpcl-dev \
  libceres-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libsuitesparse-dev \
  libatlas-base-dev \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-eigen-conversions \
  ros-noetic-tf \
  ros-noetic-rviz
```

初始化 `rosdep`。已经初始化过时，第一条可能提示已存在，可忽略：

```bash
sudo rosdep init
rosdep update
```

加载 ROS Noetic：

```bash
source /opt/ros/noetic/setup.bash
```

---

# 二、创建工作空间

```bash
mkdir -p ~/lidar_imu_ws/src
mkdir -p ~/lidar_imu_ws/third_party
```

检查目录：

```bash
tree -L 2 ~/lidar_imu_ws 2>/dev/null || \
find ~/lidar_imu_ws -maxdepth 2 -type d
```

---

# 三、下载并安装 Livox-SDK2

```bash
cd ~/lidar_imu_ws/third_party

git clone https://github.com/Livox-SDK/Livox-SDK2.git
```

编译 SDK：

```bash
cd ~/lidar_imu_ws/third_party/Livox-SDK2

mkdir -p build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

sudo make install
sudo ldconfig
```

检查 Livox-SDK2 是否安装成功：

```bash
find /usr/local/include -maxdepth 2 -iname "*livox*" | head
find /usr/local/lib -maxdepth 1 -iname "*livox*" | head
```

检查动态库：

```bash
ldconfig -p | grep -i livox
```

---

# 四、下载 Mid-360 ROS 驱动

```bash
cd ~/lidar_imu_ws/src

git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

确认目录：

```bash
ls ~/lidar_imu_ws/src/livox_ros_driver2
```

Livox 官方要求把驱动克隆到 `[工作空间]/src/`，ROS Noetic 使用 `./build.sh ROS1` 编译。([GitHub][1])

---

# 五、下载超核 IMU ROS1 驱动

先下载超核完整资料仓库：

```bash
cd ~/lidar_imu_ws/third_party

git clone https://github.com/hipnuc/products.git hipnuc_products
```

把 ROS1 的 `hipnuc_imu` 和 `hipnuc_lib_package` 一起复制到工作空间：

```bash
cp -a \
  ~/lidar_imu_ws/third_party/hipnuc_products/examples/ROS_Melodic/serial_imu_ws/src/hipnuc_imu \
  ~/lidar_imu_ws/src/

cp -a \
  ~/lidar_imu_ws/third_party/hipnuc_products/examples/ROS_Melodic/serial_imu_ws/src/hipnuc_lib_package \
  ~/lidar_imu_ws/src/
```

检查：

```bash
ls ~/lidar_imu_ws/src/hipnuc_imu
ls ~/lidar_imu_ws/src/hipnuc_lib_package
```

在本次实测使用的超核 ROS1 驱动版本中，`hipnuc_imu/package.xml` 声明依赖 `hipnuc_lib_package`。如果只复制 `hipnuc_imu`，运行 `rosdep install` 时会出现：

```text
hipnuc_imu: Cannot locate rosdep definition for [hipnuc_lib_package]
```

因此两个包必须同时放入 `~/lidar_imu_ws/src`。

---

# 六、下载 LiDAR_IMU_Init

```bash
cd ~/lidar_imu_ws/src

git clone https://github.com/hku-mars/LiDAR_IMU_Init.git
```

检查：

```bash
ls ~/lidar_imu_ws/src/LiDAR_IMU_Init
```

---

# 七、将 LiDAR_IMU_Init 改为依赖 Driver 2

官方 `LiDAR_IMU_Init` 当前仍在 `CMakeLists.txt` 和源码中依赖旧版 `livox_ros_driver`，而 Mid-360 使用 `livox_ros_driver2`，所以需要替换消息包命名空间。([GitHub][3])

执行：

```bash
cd ~/lidar_imu_ws/src/LiDAR_IMU_Init
```

先查看旧依赖：

```bash
grep -Rnw \
  --exclude-dir=.git \
  --include='CMakeLists.txt' \
  --include='package.xml' \
  --include='*.h' \
  --include='*.hpp' \
  --include='*.cpp' \
  . \
  -e 'livox_ros_driver'
```

执行精确替换：

```bash
grep -RIl \
  --exclude-dir=.git \
  --include='CMakeLists.txt' \
  --include='package.xml' \
  --include='*.h' \
  --include='*.hpp' \
  --include='*.cpp' \
  'livox_ros_driver' . \
  | xargs -r sed -i \
  's/\<livox_ros_driver\>/livox_ros_driver2/g'
```

检查替换结果：

```bash
grep -Rnw \
  --exclude-dir=.git \
  --include='CMakeLists.txt' \
  --include='package.xml' \
  --include='*.h' \
  --include='*.hpp' \
  --include='*.cpp' \
  . \
  -e 'livox_ros_driver' \
  -e 'livox_ros_driver2'
```

输出中应该只剩：

```text
livox_ros_driver2
```

重点确认：

```bash
grep -n "livox_ros_driver" CMakeLists.txt package.xml
grep -Rnw src include -e "livox_ros_driver"
```

---

# 八、配置 LiDAR_IMU_Init 话题

你的组合是：

```text
Mid-360点云：/livox/lidar
外接超核IMU：/imu/data
```

执行：

```bash
cd ~/lidar_imu_ws/src/LiDAR_IMU_Init
```

修改 `mid360.yaml` 的 IMU 话题：

```bash
sed -i \
  's#^[[:space:]]*imu_topic:.*#  imu_topic: "/imu/data"#' \
  config/mid360.yaml
```

确认：

```bash
grep -nE "lid_topic|imu_topic|mean_acc_norm|cut_frame_num|orig_odom_freq" \
  config/mid360.yaml
```

应看到类似：

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/imu/data"

initialization:
  cut_frame_num: 5
  orig_odom_freq: 10
  mean_acc_norm: 9.805
```

官方 Mid-360 配置使用 `/livox/lidar`、`lidar_type: 1`、`scan_line: 6`，并建议 Livox 的 `cut_frame_num × orig_odom_freq` 约为 50。普通 IMU 的 `mean_acc_norm` 通常写实际静止加速度模长。([GitHub][4])

暂时不要确定 `mean_acc_norm` 是 `1.0` 还是 `9.805`，等超核 IMU 有数据后实测。

---

# 九、配置超核 IMU

打开配置：

```bash
nano ~/lidar_imu_ws/src/hipnuc_imu/config/hipnuc_config.yaml
```

也可以直接用命令修改：

```bash
sed -i \
  's#^imu_serial:.*#imu_serial: "/dev/ttyUSB0"#' \
  ~/lidar_imu_ws/src/hipnuc_imu/config/hipnuc_config.yaml

sed -i \
  's#^baud_rate:.*#baud_rate: 115200#' \
  ~/lidar_imu_ws/src/hipnuc_imu/config/hipnuc_config.yaml

sed -i \
  's#^frame_id:.*#frame_id: "imu_link"#' \
  ~/lidar_imu_ws/src/hipnuc_imu/config/hipnuc_config.yaml

sed -i \
  's#^imu_topic:.*#imu_topic: "/imu/data"#' \
  ~/lidar_imu_ws/src/hipnuc_imu/config/hipnuc_config.yaml
```

查看配置：

```bash
cat ~/lidar_imu_ws/src/hipnuc_imu/config/hipnuc_config.yaml
```

应类似：

```yaml
imu_serial: "/dev/ttyUSB0"
baud_rate: 115200
frame_id: "imu_link"
imu_topic: "/imu/data"

frame_id_costom: "base_link_hipnuc"
imu_topic_costom: "/imu_package_hipnuc"
```

当前超核仓库默认配置使用 `/dev/ttyUSB0`、`115200` 和 `/IMU_data`；这里将话题统一改成 `/imu/data`，方便接入 LI-Init。([GitHub][5])

---

# 十、准备 ROS1 的 package.xml

`livox_ros_driver2` 仓库默认保存为 `package_ROS1.xml`，先复制成标准文件：

```bash
cd ~/lidar_imu_ws/src/livox_ros_driver2

cp -f package_ROS1.xml package.xml
```

安装工作空间依赖：

```bash
source /opt/ros/noetic/setup.bash

cd ~/lidar_imu_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

---

# 十一、编译整个工作空间

使用 Livox 官方 ROS1 编译脚本：

```bash
source /opt/ros/noetic/setup.bash

cd ~/lidar_imu_ws/src/livox_ros_driver2

chmod +x build.sh

./build.sh ROS1
```

这个脚本会调用工作空间根目录下的 `catkin_make`，同时编译：

```text
livox_ros_driver2
hipnuc_lib_package
hipnuc_imu
LiDAR_IMU_Init
```

官方 `build.sh` 在 ROS1 模式下会复制 ROS1 的 `package.xml`，然后返回工作空间执行 `catkin_make`。([GitHub][6])

编译完成后加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/lidar_imu_ws/devel/setup.bash
```

检查三个 ROS 包：

```bash
rospack find livox_ros_driver2
rospack find hipnuc_lib_package
rospack find hipnuc_imu
rospack find lidar_imu_init
```

正常输出类似：

```text
/home/zjs/lidar_imu_ws/src/livox_ros_driver2
/home/zjs/lidar_imu_ws/src/hipnuc_imu
/home/zjs/lidar_imu_ws/src/LiDAR_IMU_Init
```

---

# 十二、创建环境加载脚本

```bash
cat > ~/lidar_imu_ws/setup_env.sh <<'EOF'
#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
source "$HOME/lidar_imu_ws/devel/setup.bash"

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"
EOF
```

添加权限：

```bash
chmod +x ~/lidar_imu_ws/setup_env.sh
```

以后每个新终端执行：

```bash
source ~/lidar_imu_ws/setup_env.sh
```

暂时不要把它写入 `~/.bashrc`，避免与 `~/JZJ_local` 工作空间互相覆盖。

---

# 十三、检查超核 IMU 串口

插入超核 IMU，执行：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

查看识别日志：

```bash
dmesg | grep -Ei "ttyUSB|ttyACM|cp210|ch34" | tail -30
```

超核官方 USB 转串口板通常会被识别为 `/dev/ttyUSBx`。([GitHub][7])

添加永久串口权限：

```bash
sudo usermod -aG dialout "$USER"
```

当前这次测试临时赋权：

```bash
sudo chmod 666 /dev/ttyUSB0
```

确认：

```bash
ls -l /dev/ttyUSB0
```

如果实际设备是 `/dev/ttyUSB1`，修改配置：

```bash
sed -i \
  's#^imu_serial:.*#imu_serial: "/dev/ttyUSB1"#' \
  ~/lidar_imu_ws/src/hipnuc_imu/config/hipnuc_config.yaml
```

如果 IMU 实际波特率不是 `115200`，例如 `460800`：

```bash
sed -i \
  's#^baud_rate:.*#baud_rate: 460800#' \
  ~/lidar_imu_ws/src/hipnuc_imu/config/hipnuc_config.yaml
```

超核 ROS1 官方示例支持 `115200` 和 `460800`，驱动值必须与 IMU 当前配置一致。([GitHub][7])

---

# 十四、启动超核 IMU

新终端 1：

```bash
source ~/lidar_imu_ws/setup_env.sh

roslaunch hipnuc_imu imu_msg.launch
```

新终端 2 检查：

```bash
source ~/lidar_imu_ws/setup_env.sh

rostopic info /imu/data
```

检查频率：

```bash
rostopic hz /imu/data
```

读取一帧：

```bash
rostopic echo -n 1 /imu/data
```

单独检查时间戳：

```bash
rostopic echo -n 5 /imu/data/header/stamp
```

正常应满足：

```text
消息类型：sensor_msgs/Imu
Publishers：存在超核驱动节点
频率：持续稳定输出
header.stamp：非零并持续增加
```

---

# 十五、测量超核 IMU 静止加速度模长

让 IMU 静止放平，然后执行：

```bash
source ~/lidar_imu_ws/setup_env.sh

python3 - <<'PY'
import math
import statistics

import rospy
from sensor_msgs.msg import Imu

rospy.init_node("check_hipnuc_acc_norm", anonymous=True)

values = []

for _ in range(100):
    msg = rospy.wait_for_message("/imu/data", Imu, timeout=3.0)
    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    az = msg.linear_acceleration.z
    values.append(math.sqrt(ax * ax + ay * ay + az * az))

print("samples:", len(values))
print("mean acceleration norm:", statistics.mean(values))
print("min:", min(values))
print("max:", max(values))
PY
```

如果结果接近：

```text
1.0
```

设置：

```bash
sed -i \
  's#^[[:space:]]*mean_acc_norm:.*#  mean_acc_norm: 1.0#' \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/config/mid360.yaml
```

如果结果接近：

```text
9.8
```

设置：

```bash
sed -i \
  's#^[[:space:]]*mean_acc_norm:.*#  mean_acc_norm: 9.805#' \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/config/mid360.yaml
```

超核官方示例中的静止加速度数据模长接近 `1`，因此你这个驱动版本有可能输出 `g`，不能直接假定一定是 `9.805 m/s²`。应以上面实测结果为准。([GitHub][7])

修改 YAML 后不需要重新编译。

---

# 十六、配置 Mid-360 网卡

你之前的有线网卡是：

```text
enp8s0
```

连接 Mid-360 网线后执行：

```bash
sudo ip link set enp8s0 up
sudo ip addr flush dev enp8s0
sudo ip addr add 192.168.1.5/24 dev enp8s0
```

检查：

```bash
ip addr show enp8s0
```

测试 Mid-360 默认 IP：

```bash
ping -c 4 192.168.1.12
```

当前官方 `MID360_config.json` 默认主机地址是 `192.168.1.5`，雷达地址是 `192.168.1.12`。([GitHub][8])

查看配置：

```bash
cat ~/lidar_imu_ws/src/livox_ros_driver2/config/MID360_config.json
```

检查关键 IP：

```bash
grep -nE \
  'cmd_data_ip|push_msg_ip|point_data_ip|imu_data_ip|"ip"' \
  ~/lidar_imu_ws/src/livox_ros_driver2/config/MID360_config.json
```

如果雷达实际 IP 不是 `192.168.1.12`，使用编辑器修改：

```bash
nano ~/lidar_imu_ws/src/livox_ros_driver2/config/MID360_config.json
```

---

# 十七、启动 Mid-360

保持超核 IMU 驱动运行。

新终端 3：

```bash
source ~/lidar_imu_ws/setup_env.sh

roslaunch livox_ros_driver2 msg_MID360.launch
```

这里必须使用：

```text
msg_MID360.launch
```

它发布 Livox 自定义点云消息；`rviz_MID360.launch` 发布 `PointCloud2`，不适合当前 LI-Init 的 Livox `CustomMsg` 接口。([GitHub][1])

新终端 4 检查：

```bash
source ~/lidar_imu_ws/setup_env.sh

rostopic list | grep -Ei "livox|imu"
```

应至少看到：

```text
/livox/lidar
/livox/imu
/imu/data
```

检查点云类型：

```bash
rostopic type /livox/lidar
```

应输出：

```text
livox_ros_driver2/CustomMsg
```

检查点云频率：

```bash
rostopic hz /livox/lidar
```

检查外接 IMU：

```bash
rostopic type /imu/data
rostopic hz /imu/data
```

---

# 十八、同时检查两个时间戳

```bash
rostopic echo -n 3 /livox/lidar/header/stamp
```

```bash
rostopic echo -n 3 /imu/data/header/stamp
```

要求：

```text
两者均非零
均持续递增
没有反跳
没有频繁归零
```

LI-Init 可以估计 LiDAR–IMU 的时间偏移和空间外参，不要求事先安装额外硬件同步装置，但输入时间戳必须可用且延迟相对稳定。([GitHub][9])

---

# 十九、录制标定 rosbag

新终端 5：

```bash
source ~/lidar_imu_ws/setup_env.sh

mkdir -p ~/lidar_imu_ws/bags

rosbag record \
  -O ~/lidar_imu_ws/bags/mid360_hipnuc_calib.bag \
  /livox/lidar \
  /imu/data
```

不要把 `/livox/imu` 当作标定 IMU；你现在使用的是外接超核 IMU `/imu/data`。

---

# 二十、启动 LiDAR_IMU_Init 标定

确认这两条都有稳定频率：

```bash
rostopic hz /livox/lidar
```

```bash
rostopic hz /imu/data
```

然后新终端 6：

```bash
source ~/lidar_imu_ws/setup_env.sh

roslaunch lidar_imu_init livox_mid360.launch rviz:=true
```

官方仓库提供 `livox_mid360.launch`，它会加载 `config/mid360.yaml`。([GitHub][10])

---

# 二十一、标定动作

启动后：

```text
1. 整套 Mid-360 + IMU 刚性支架静止约 5～10 秒
2. 绕 X 轴反复俯仰
3. 绕 Y 轴反复侧倾
4. 绕 Z 轴反复旋转
5. 前后、左右、上下平移
6. 最后做组合旋转和平移
```

Mid-360 和超核 IMU 必须刚性固定，运动过程中不能发生相对位移。

---

# 二十二、查看结果

标定完成后：

```bash
cat \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
```

也可以持续观察：

```bash
watch -n 1 \
  cat ~/lidar_imu_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
```

官方说明结果会写入该文件，其中包含旋转、平移以及时间偏移。([GitHub][11])

---

## 最后确认命令

```bash
source ~/lidar_imu_ws/setup_env.sh

echo "===== ROS packages ====="
rospack find livox_ros_driver2
rospack find hipnuc_lib_package
rospack find hipnuc_imu
rospack find lidar_imu_init

echo "===== Topic types ====="
rostopic type /livox/lidar
rostopic type /imu/data

echo "===== Topic publishers ====="
rostopic info /livox/lidar
rostopic info /imu/data
```

最终必须满足：

```text
/livox/lidar → livox_ros_driver2/CustomMsg
/imu/data    → sensor_msgs/Imu
两者均有 Publisher
两者均有稳定频率
```


---

# 二十三、常见配置错误与实测排查

## 23.1 `mid360.yaml contains invalid YAML`

典型报错：

```text
expected <block end>, but found '<block mapping start>'
```

通常是 `common` 下的两个话题缩进不一致，或文件中混入了 Tab。文件开头应为：

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/imu/data"
```

检查前 15 行：

```bash
nl -ba ~/lidar_imu_ws/src/LiDAR_IMU_Init/config/mid360.yaml | head -15
```

检查 Tab：

```bash
grep -nP $'\t' ~/lidar_imu_ws/src/LiDAR_IMU_Init/config/mid360.yaml
```

若有 Tab，替换成两个空格：

```bash
sed -i $'s/\t/  /g' \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/config/mid360.yaml
```

验证 YAML：

```bash
python3 - <<'PY'
import yaml

path = "/home/zjs/lidar_imu_ws/src/LiDAR_IMU_Init/config/mid360.yaml"

with open(path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)

print("YAML格式正确")
print("lid_topic:", data["common"]["lid_topic"])
print("imu_topic:", data["common"]["imu_topic"])
PY
```

修改 YAML 后不需要重新编译。

## 23.2 `Cannot load message class for livox_ros_driver2/CustomMsg`

如果：

```bash
rostopic type /livox/lidar
```

能看到：

```text
livox_ros_driver2/CustomMsg
```

但：

```bash
rostopic hz /livox/lidar
```

报：

```text
Cannot load message class for [livox_ros_driver2/CustomMsg]
```

说明当前终端没有加载工作空间生成的消息环境。执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/lidar_imu_ws/devel/setup.bash

rosmsg show livox_ros_driver2/CustomMsg
rostopic hz /livox/lidar
```

若仍失败，检查生成文件：

```bash
find ~/lidar_imu_ws/devel \
  -path "*livox_ros_driver2/msg*" \
  -type f
```

必要时重新编译：

```bash
source /opt/ros/noetic/setup.bash
cd ~/lidar_imu_ws/src/livox_ros_driver2
./build.sh ROS1
source ~/lidar_imu_ws/devel/setup.bash
```

## 23.3 ROS 话题区分大小写

以下是不同话题：

```text
/imu/data
/IMU/data
/IMU_data
```

必须以：

```bash
rostopic list | grep -Ei "imu|livox"
```

显示的实际话题为准。本次最终使用：

```text
/imu/data
```

---

# 二十四、推荐的录包与离线标定流程

## 24.1 录制原始数据

确认两路都有稳定频率：

```bash
source ~/lidar_imu_ws/setup_env.sh

rostopic hz /livox/lidar
rostopic hz /imu/data
```

开始录制：

```bash
mkdir -p ~/lidar_imu_ws/bags

rosbag record \
  -O ~/lidar_imu_ws/bags/mid360_hipnuc_calib.bag \
  /livox/lidar \
  /imu/data
```

建议录制 90～180 秒，动作顺序：

```text
1. 静止 5～10 秒
2. 绕 X 轴反复俯仰
3. 绕 Y 轴反复侧倾
4. 绕 Z 轴反复旋转
5. 前后、左右、上下平移
6. 旋转和平移组合运动
7. 初始化完成后继续运动至少 20～30 秒，用于在线细化
```

检查包：

```bash
rosbag info ~/lidar_imu_ws/bags/mid360_hipnuc_calib.bag
```

## 24.2 离线标定前关闭实物驱动

必须关闭：

```text
livox_ros_driver2
hipnuc_imu
其他向 /livox/lidar 或 /imu/data 发布的节点
```

播放期间两个话题应各自只有一个 rosbag 发布者：

```bash
rostopic info /livox/lidar
rostopic info /imu/data
```

正常应只看到类似：

```text
/play_1785...
```

如果同时出现 `/chaohe_imu` 或 Livox 实物驱动节点，时间戳会交错，导致：

```text
IMU loop back, clear IMU buffer.
```

## 24.3 `/use_sim_time` 的设置

本次离线标定推荐：

```bash
rosparam set /use_sim_time false
rosparam get /use_sim_time
```

应输出：

```text
false
```

原因：LI-Init 使用 LiDAR 和 IMU 消息中的 `header.stamp` 同步数据。本次 rosbag 中两路原始时间戳已经处于同一 Unix 时间基准，因此无需 `/clock`。

推荐播放方式：

```bash
source ~/lidar_imu_ws/setup_env.sh

rosbag play \
  ~/lidar_imu_ws/bags/mid360_hipnuc_calib.bag \
  -r 0.5
```

不要使用：

```text
--clock
-l
--loop
```

除非明确需要仿真时间且整个系统均正确使用 `/clock`。

## 24.4 启动 LI-Init

终端 1：

```bash
source ~/lidar_imu_ws/setup_env.sh

rosparam set /use_sim_time false

roslaunch lidar_imu_init livox_mid360.launch rviz:=true
```

终端 2：

```bash
source ~/lidar_imu_ws/setup_env.sh

rosbag play \
  ~/lidar_imu_ws/bags/mid360_hipnuc_calib.bag \
  -r 0.5
```

---

# 二十五、时间戳检查与 `IMU loop back` 排查

## 25.1 检查原始 bag 时间戳

下面脚本单次扫描两路消息，并检查倒退和重复时间戳：

```bash
source /opt/ros/noetic/setup.bash
source ~/lidar_imu_ws/devel/setup.bash

python3 -u - <<'PY'
import os
import rosbag

bag_path = "/home/zjs/lidar_imu_ws/bags/mid360_hipnuc_calib.bag"
topics = ["/livox/lidar", "/imu/data"]

stats = {
    topic: {
        "count": 0,
        "backward": 0,
        "duplicate": 0,
        "first_header": None,
        "last_header": None,
        "previous_header": None,
    }
    for topic in topics
}

print("开始读取:", bag_path, flush=True)
print("文件大小: %.2f GB" % (os.path.getsize(bag_path) / 1024**3), flush=True)

total_read = 0
with rosbag.Bag(bag_path, "r") as bag:
    for topic, msg, record_time in bag.read_messages(topics=topics):
        total_read += 1
        s = stats[topic]
        header_sec = msg.header.stamp.to_sec()

        if s["count"] == 0:
            s["first_header"] = header_sec

        if s["previous_header"] is not None:
            if header_sec < s["previous_header"]:
                s["backward"] += 1
            elif header_sec == s["previous_header"]:
                s["duplicate"] += 1

        s["previous_header"] = header_sec
        s["last_header"] = header_sec
        s["count"] += 1

        if total_read % 100000 == 0:
            print(
                f"已读取 {total_read} 条消息，"
                f"LiDAR={stats['/livox/lidar']['count']}，"
                f"IMU={stats['/imu/data']['count']}",
                flush=True
            )

print("\n========== 检查结果 ==========")
for topic, s in stats.items():
    print("\nTopic:", topic)
    print("  messages:", s["count"])
    print("  header first:", f"{s['first_header']:.9f}")
    print("  header last: ", f"{s['last_header']:.9f}")
    print("  backward timestamps:", s["backward"])
    print("  duplicate timestamps:", s["duplicate"])

lidar = stats["/livox/lidar"]
imu = stats["/imu/data"]
print(
    "\n首帧 IMU-LiDAR 时间差:",
    f"{imu['first_header'] - lidar['first_header']:.9f} s"
)
PY
```

本次实测原始包结果：

```text
/livox/lidar：1707 条，时间戳无倒退、无重复
/imu/data：170717 条，时间戳无倒退、无重复
首帧 IMU-LiDAR 时间差：0.044114590 s
```

这说明原始 bag 的两路时间基准正常，不需要重新打时间戳。

## 25.2 十亿秒时间偏移为什么出现

异常结果示例：

```text
Time Lag IMU to LiDAR = -1785404720 s
```

这种结果不是正常的传感器延迟，而是运行时混入了不同时间基准的数据。常见原因：

- 实物 IMU 驱动和 rosbag 同时发布 `/imu/data`；
- `/use_sim_time=true`，实物驱动用 `ros::Time::now()` 获取到了 `/clock` 相对时间；
- bag 被 `-l` 或 `--loop` 循环播放；
- 其他仿真节点发布 `/clock`；
- 同一话题存在多个发布者。

排查：

```bash
ps -ef | grep '[r]osbag play'
rosnode list | grep -Ei "livox|imu|hipnuc|chaohe|play|gazebo"
rostopic info /imu/data
rostopic info /livox/lidar
rostopic info /clock
```

## 25.3 标定完成后出现警告

标定达到 100% 后，若 rosbag 仍继续播放、循环播放或从头开始，可能出现：

```text
IMU loop back, clear IMU buffer.
TF_REPEATED_DATA ignoring data with redundant timestamp
```

只要已经出现：

```text
[Refinement] Online Refinement 100%
[Final Result] ...
Initialization and refinement result is written to ...
```

结果已经写入文件。应立即停止播放，不需要让 bag 继续运行。

---

# 二十六、实测成功结果与合理性判断

本次在统一时间环境、关闭实物驱动并单次播放 rosbag 后，获得：

```text
[Final Result] Rotation LiDAR to IMU    =  1.571001 29.974871 90.106281 deg
[Final Result] Translation LiDAR to IMU = 0.038238 0.028072 0.067594 m
[Final Result] Time Lag IMU to LiDAR    = -0.01905585 s
[Final Result] Bias of Gyroscope        = -0.000405 -0.000104 0.000768 rad/s
[Final Result] Bias of Accelerometer    = 0.007890 -0.008969 0.010759 m/s^2
[Final Result] Gravity in World Frame   = 4.387969 -0.092563 -8.768388 m/s^2
```

## 26.1 平移外参

平移模长约为：

```text
0.0827 m，即 8.27 cm
```

应与实际测量的 LiDAR 中心到 IMU 中心距离进行对照。

## 26.2 时间偏移

```text
-0.01905585 s ≈ -19.06 ms
```

该数值处于软件时间同步系统中较合理的毫秒级范围。写入其他算法前，还应确认目标程序对时间偏移正负号的定义。

## 26.3 重力与 IMU 单位

重力向量模长接近：

```text
9.805 m/s²
```

说明当前 `mean_acc_norm` 和超核 IMU 加速度尺度配置基本正确。

## 26.4 旋转外参

旋转约为：

```text
X：1.57°
Y：29.97°
Z：90.11°
```

应结合实际安装方向判断。如果 IMU 相对 Mid-360 确实存在约 30° 倾斜和约 90° 水平旋转，则结果合理；若两个坐标轴实际几乎平行，需要重新检查 IMU 坐标系定义和驱动坐标变换。

---

# 二十七、标定到 100% 后如何正确结束

看到：

```text
[Refinement] Online Refinement 100%
Initialization and refinement result is written to ...
```

即可结束，不必等待 bag 播放完。

推荐顺序：

```text
1. 在 rosbag play 终端按 Ctrl+C
2. 在 LiDAR_IMU_Init 终端按 Ctrl+C
3. RViz 若没有自动关闭，再单独关闭
```

检查结果文件：

```bash
cat ~/lidar_imu_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
```

立即备份，避免下一次标定覆盖：

```bash
cp \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/result/mid360_hipnuc_calib_01.txt
```

带时间命名备份：

```bash
cp \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt \
  ~/lidar_imu_ws/src/LiDAR_IMU_Init/result/mid360_hipnuc_$(date +%Y%m%d_%H%M%S).txt
```

---

# 二十八、最终推荐复测

同一安装状态下，建议重新录制至少两组数据并独立标定。比较：

```text
旋转角：最好相差不超过约 1°
平移：最好相差不超过约 1～2 cm
时间偏移：最好保持在相近的毫秒级范围
```

最终将稳定结果写入 FAST-LIO，并通过以下现象验证：

- 墙面是否清晰、无双层重影；
- 快速旋转时点云是否撕裂；
- 地面是否平整；
- 轨迹是否连续；
- 静止时里程计是否明显漂移。

写入 FAST-LIO 时，应从结果文件复制完整旋转矩阵，而不是直接把三个欧拉角填入 `extrinsic_R`。

[1]: https://github.com/Livox-SDK/livox_ros_driver2/tree/master "GitHub - Livox-SDK/livox_ros_driver2: Livox device driver under Ros(Compatible with ros and ros2), support Lidar HAP and Mid-360. · GitHub"
[2]: https://github.com/hipnuc/products/blob/master/examples/ROS_Melodic/serial_imu_ws/src/hipnuc_imu/CMakeLists.txt?utm_source=chatgpt.com "products/examples/ROS_Melodic/serial_imu_ws/src/hipnuc_imu/CMakeLists.txt at master · hipnuc/products · GitHub"
[3]: https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/CMakeLists.txt?utm_source=chatgpt.com "LiDAR_IMU_Init/CMakeLists.txt at main · hku-mars/LiDAR_IMU_Init · GitHub"
[4]: https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/config/mid360.yaml?utm_source=chatgpt.com "LiDAR_IMU_Init/config/mid360.yaml at main · hku-mars/LiDAR_IMU_Init · GitHub"
[5]: https://github.com/hipnuc/products/blob/master/examples/ROS_Melodic/serial_imu_ws/src/hipnuc_imu/config/hipnuc_config.yaml?utm_source=chatgpt.com "products/examples/ROS_Melodic/serial_imu_ws/src/hipnuc_imu/config/hipnuc_config.yaml at master · hipnuc/products · GitHub"
[6]: https://github.com/Livox-SDK/livox_ros_driver2/blob/master/build.sh?utm_source=chatgpt.com "livox_ros_driver2/build.sh at master · Livox-SDK/livox_ros_driver2 · GitHub"
[7]: https://github.com/hipnuc/products/blob/master/examples/ROS_Melodic/README-IMU.md?utm_source=chatgpt.com "products/examples/ROS_Melodic/README-IMU.md at master · hipnuc/products · GitHub"
[8]: https://github.com/Livox-SDK/livox_ros_driver2/blob/master/config/MID360_config.json "livox_ros_driver2/config/MID360_config.json at master · Livox-SDK/livox_ros_driver2 · GitHub"
[9]: https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/README.md?utm_source=chatgpt.com "LiDAR_IMU_Init/README.md at main · hku-mars/LiDAR_IMU_Init · GitHub"
[10]: https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/launch/livox_mid360.launch?utm_source=chatgpt.com "LiDAR_IMU_Init/launch/livox_mid360.launch at main · hku-mars/LiDAR_IMU_Init · GitHub"
[11]: https://github.com/hku-mars/LiDAR_IMU_Init/wiki?utm_source=chatgpt.com "GitHub - hku-mars/LiDAR_IMU_Init: [IROS2022] Robust Real-time LiDAR-inertial Initialization Method. · GitHub"
