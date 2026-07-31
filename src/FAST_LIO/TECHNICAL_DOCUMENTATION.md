# FAST-LIO 激光惯性里程计技术文档

项目路径：`src/location/FAST_LIO`
ROS 包名：`fast_lio`，可执行节点 `fastlio_mapping`（节点名 `laserMapping`）
上游：HKU-MARS [FAST_LIO](https://github.com/hku-mars/FAST_LIO)（FAST-LIO 2.0），论文见 `doc/Fast_LIO_2.pdf` 与 arXiv:2010.08196

## 1. 项目含义与定位

FAST-LIO 是一套紧耦合的激光雷达-惯性里程计（LiDAR-Inertial Odometry, LIO），在本工作区 `src/location/` 中承担**定位与建图前端**的角色：输入原始 LiDAR 点云和 IMU 数据，实时输出高频（可达 100 Hz 以上）的机体位姿、里程计和配准后的全局点云地图，供下游感知（如 `src/perception/SDOT`）与规划模块使用。

相比传统 LOAM 系列，本版本（FAST-LIO 2）的核心特点：

1. **无需特征提取**：直接对降采样后的原始点做 scan-to-map 配准（点到面残差），因此一套代码同时支持机械旋转式（Velodyne、Ouster）和固态（Livox Avia / Horizon / MID 系列）雷达；
2. **迭代扩展卡尔曼滤波（IEKF）**：IMU 传播与点云观测在流形上做紧耦合融合，快速运动和退化场景下仍保持鲁棒；
3. **ikd-Tree 增量地图**：动态 KD 树支持增量插入/删除与并行最近邻搜索，地图随机器人移动按 FOV 局部裁剪，计算量不随地图总规模增长；
4. 支持外 IMU、在线外参估计、ARM 平台（树莓派 4B、TX2 等）。

## 2. 项目内容（代码结构）

```
FAST_LIO/
├── CMakeLists.txt / package.xml    # 构建描述；单可执行 fastlio_mapping
├── msg/Pose6D.msg                  # IMU 预积分位姿（内部调试用）
├── config/                         # 各雷达配置
│   ├── avia.yaml / horizon.yaml / mid360.yaml   # Livox 系列
│   ├── velodyne.yaml / ouster64.yaml            # 机械旋转雷达
│   └── marsim.yaml                              # MARSIM 仿真
├── launch/
│   ├── mapping_avia/horizon/mid360.launch       # 对应雷达启动文件
│   ├── mapping_velodyne/ouster64.launch
│   ├── mapping_marsim.launch
│   └── gdb_debug_example.launch                 # gdb 调试示例
├── src/
│   ├── laserMapping.cpp            # 主程序：数据同步、IEKF 主循环、地图维护、发布
│   ├── preprocess.{h,cpp}          # 点云预处理：多雷达格式解析、盲区过滤、特征提取（可关）
│   └── IMU_Processing.hpp          # IMU 初始化、前向传播、点云运动畸变校正
├── include/
│   ├── IKFoM_toolkit/              # 流形上的迭代卡尔曼滤波工具库
│   │   ├── esekfom/esekfom.hpp     # IEKF 框架实现
│   │   └── mtk/                    # 流形类型：SO3、S2、向量及其 ⊞/⊟ 运算
│   ├── ikd-Tree/                   # 增量 KD 树（git 子模块，需单独拉取）
│   ├── use-ikfom.hpp               # 状态/输入/噪声流形定义与系统模型 f、∂f/∂x、∂f/∂w
│   ├── common_lib.h / so3_math.h / Exp_mat.h  # 类型别名与 SO(3) 数学工具
│   └── matplotlibcpp.h             # 调试用绘图
├── rviz_cfg/loam_livox.rviz        # 可视化配置
├── PCD/                            # pcd_save_en=1 时地图输出目录（scans.pcd）
├── Log/                            # 耗时分析脚本（plot.py / .m）与 pos_log.txt 输出位置
└── doc/                            # 论文 PDF 与实验图片
```

注意：`include/ikd-Tree` 是 git 子模块（`.gitmodules` 指向 `hku-mars/ikd-Tree` 的 `fast_lio` 分支），克隆后必须执行 `git submodule update --init`，否则编译时找不到 `ikd_Tree.cpp`。

## 3. 系统架构与数据流

```
LiDAR 驱动 ──→ standard_pcl_cbk / livox_pcl_cbk ──→ Preprocess（格式解析、盲区/降采样）
IMU 驱动  ──→ imu_cbk ─────────────────────────┐
                                                ▼
                                    sync_packages（按时间戳组帧）
                                                ▼
                        ImuProcess::Process
                          ├─ IMU_init（静止初始化重力与零偏）
                          ├─ IEKF 前向传播（逐 IMU 样本积分状态与协方差）
                          └─ UndistortPcl（反向传播逐点去畸变）
                                                ▼
                        体素降采样（filter_size_surf）
                                                ▼
                        IEKF 迭代更新（最多 max_iteration 次）
                          ├─ h_share_model：ikd-Tree 最近邻 → 平面拟合 → 点到面残差与雅可比
                          └─ 收敛后输出状态（位姿/速度/零偏/重力）
                                                ▼
                        map_incremental（新点增量插入 ikd-Tree）
                        lasermap_fov_segment（按 FOV 裁剪局部地图）
                                                ▼
              /Odometry、/path、/cloud_registered、/cloud_registered_body、TF
```

## 4. 算法原理与公式

### 4.1 状态定义（流形）

状态定义在 `use-ikfom.hpp`，用 IKFoM 的 `MTK_BUILD_MANIFOLD` 构造复合流形：

$$
x = [\underbrace{{}^Gp_I}_{3},\ \underbrace{{}^GR_I}_{SO(3)},\ \underbrace{{}^IR_L}_{SO(3)},\ \underbrace{{}^It_L}_{3},\ \underbrace{{}^Gv_I}_{3},\ \underbrace{b_g}_{3},\ \underbrace{b_a}_{3},\ \underbrace{{}^Gg}_{S^2}]
$$

依次为 IMU 位置、姿态、LiDAR-IMU 外参旋转与平移、速度、陀螺零偏、加速度计零偏、重力向量。总自由度 23（$S^2$ 上重力只有 2 个自由度）。输入 $u = [a_m,\ \omega_m]$（加速度计与陀螺原始读数），过程噪声 $w = [n_g,\ n_a,\ n_{b_g},\ n_{b_a}]$ 共 12 维。

姿态（SO3）与重力（S2）上的"加减法"由流形运算 $\boxplus / \boxminus$ 定义，这是 IKFoM 的核心：滤波器的所有状态更新、误差传播都在流形切空间进行，避免四元数归一化与奇异问题。

### 4.2 连续时间运动学模型（IMU 前向传播）

`get_f` 给出状态微分方程：

$$
\dot{p} = v,\qquad \dot{R} = R\,[\omega_m - b_g - n_g]_\times
$$

$$
\dot{v} = R(a_m - b_a - n_a) + {}^Gg,\qquad \dot{b}_g = n_{b_g},\qquad \dot{b}_a = n_{b_a},\qquad {}^G\dot{g} = 0
$$

外参 ${}^IR_L, {}^It_L$ 视为常量（在线估计时同样零微分）。每收到一帧 IMU，按采样间隔 $\Delta t$ 离散化传播均值与协方差：

$$
\hat{x}_{k+1} = \hat{x}_k \boxplus \big(f(\hat{x}_k, u_k)\,\Delta t\big)
$$

$$
P_{k+1} = F_{\tilde x}\, P_k\, F_{\tilde x}^T + F_w\, Q\, F_w^T
$$

其中 $F_{\tilde x} = \exp\big(\mathcal{A}\,\Delta t\big)$ 是误差状态转移矩阵，$\mathcal{A}$ 由 `df_dx`（24×23）给出，非零块包括：

$$
\frac{\partial \dot v}{\partial \delta\theta} = -R[a_m - b_a]_\times,\quad \frac{\partial \dot v}{\partial b_a} = -R,\quad \frac{\partial \dot\theta}{\partial b_g} = -I,\quad \frac{\partial \dot v}{\partial \delta g} = N(g)
$$

$N(g)$ 是 S2 流形给出的 3×2 基矩阵。`df_dw` 给出噪声映射 $F_w$（24×12）。过程噪声协方差 $Q$ 由 `acc_cov`、`gyr_cov`、`b_acc_cov`、`b_gyr_cov` 四个参数按对角阵构造。

### 4.3 反向传播运动畸变校正

一帧点云内各点采样时刻不同。IMU 传播得到每个 IMU 时刻的位姿后，以帧末时刻为基准，对每个点按其在帧内的相对时间 $\Delta t_j$ 做反向传播：

$$
{}^I p_j = T_{I_{k+1}}^{-1}\, T_{I_j}\, T_{IL}\, {}^L p_j
$$

即把帧内所有点统一变换到帧末时刻的 LiDAR 坐标系。`PointType.curvature` 字段复用为点相对帧首的时间偏移（毫秒），这就是 README 强调"点云必须带逐点时间戳"的原因：没有时间戳则无法去畸变。Velodyne/Ouster 由驱动提供 `time`/`t` 字段，Livox 必须用 `livox_lidar_msg.launch` 的 `CustomMsg`。

### 4.4 观测模型：点到面残差

去畸变并降采样后的点 ${}^L p_j$ 先按当前状态估计投影到全局系：

$$
{}^G\hat p_j = {}^G\hat R_I\,\big({}^I\hat R_L\,{}^L p_j + {}^I\hat t_L\big) + {}^G\hat p_I
$$

在 ikd-Tree 中搜索该点的 5 个最近邻（`NUM_MATCH_POINTS`），最远邻距离超过阈值（5 m 平方距离）则弃用。用近邻点拟合局部平面 $n^T x + d = 0$（`esti_plane`，要求平面内点残差小于 0.1 m），观测残差为点到平面距离：

$$
z_j = h_j(x) = n_j^T\,{}^G p_j + d_j
$$

残差有效性还加了一个权重门限：

$$
s_j = 1 - 0.9\,\frac{|z_j|}{\|{}^L p_j\|}
$$

$s_j \le 0.9$ 的点被剔除——距离传感器越远，同样的位移对应的残差可信度越低。

残差对误差状态的雅可比（`h_share_model`）：

$$
H_j = \underbrace{\begin{bmatrix} n_j^T, & -\big({}^GR_I[{}^I p'_j]_\times\big)^T C, & \cdots \end{bmatrix}}_{1\times 12\ \text{(映射到 23 维状态)}},\quad C = {}^G R_I^T n_j,\quad {}^I p'_j = {}^IR_L\,{}^L p_j + {}^It_L
$$

每个有效点贡献一行 $H$ 和一个标量观测 $-z_j$。观测噪声取固定值 `LASER_POINT_COV = 0.001`。

### 4.5 迭代卡尔曼更新（IEKF）

更新在 IKFoM 的 `esekf` 框架内进行。每轮迭代重新做一次最近邻搜索与平面拟合（因为位姿更新后对应关系变了），构造全体有效点的 $H$ 与残差向量 $z$ 后，计算卡尔曼增益的等价形式：

$$
K = \big(H^T R^{-1} H + P^{-1}\big)^{-1} H^T R^{-1}
$$

状态按新息更新并投影回流形：

$$
\hat x \leftarrow \hat x \boxplus \big(-K z - (I - KH)\, J^{-1}(\hat x \boxminus \bar x)\,(\bar x \boxminus \hat x)\big)
$$

其中 $J^{-1}$ 是流形 $\boxplus$ 运算的雅可比逆。迭代终止条件：新旧估计之差小于阈值 $\varepsilon = 0.001$（各分量）或达到 `max_iteration`（Velodyne 默认 3，Avia 默认 4）。收敛后更新协方差：

$$
P \leftarrow (I - KH)\,P\,(I - KH)^T + K R K^T
$$

$H$ 的装配采用 FAST-LIO 论文提出的动态共享方式（`dyn_share_datastruct`）：$H$ 只算非零的 12 列，$R$ 退化为标量，增益公式中矩阵求逆的维度从 23 降到实际有效维数，这是"FAST"的关键之一。

### 4.6 ikd-Tree 增量地图

地图点存放在 ikd-Tree（增量 KD 树）中，支持：

- **增量插入**（`Add_Points`）：新配准的点经 `filter_size_map`（0.5 m）体素判断后插入，同体素已有更近点时跳过，控制地图密度；
- **并行最近邻搜索**（`Nearest_Search`）：`h_share_model` 内由 OpenMP 多线程执行（核数 > 4 时 3 线程）；
- **局部地图裁剪**（`lasermap_fov_segment`）：局部地图是以雷达为中心、边长 `cube_side_length`（1000 m）的滑动立方体，机器人移动超过阈值（`MOV_THRESHOLD = 1.5 m`）时，把已离开区域的子树从地图删除，保证搜索时间恒定。

### 4.7 IMU 初始化

`IMU_init` 在前若干帧（`MAX_INI_COUNT`）假设设备静止：累积 IMU 均值，用平均加速度方向初始化重力向量 ${}^Gg$ 的指向与模长，用陀螺均值初始化 $b_g$，并设初始协方差。初始化完成前系统只做传播不做更新。因此**开机瞬间设备需保持静止约 1~2 秒**。

### 4.8 预处理

`Preprocess` 按 `lidar_type` 分派四种解析器：Livox `CustomMsg`、Velodyne（ring+time）、Ouster（ring+t）、MARSIM 仿真。通用处理：盲区过滤（`blind` 米以内剔除）、`point_filter_num` 隔点抽样、可选的 LOAM 式边缘/平面特征提取（`feature_extract_enable`，FAST-LIO2 默认关闭，直接用原始点）。

## 5. 接口定义

### 5.1 输入话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `lid_topic`（如 `/livox/lidar`、`/velodyne_points`） | `livox_ros_driver/CustomMsg` 或 `sensor_msgs/PointCloud2` | 点云，必须带逐点时间戳 |
| `imu_topic`（如 `/livox/imu`、`/imu/data`） | `sensor_msgs/Imu` | IMU，6 轴或 9 轴均可，频率需显著高于雷达帧率 |

### 5.2 输出话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | 高频里程计（frame `camera_init` → `body`） |
| `/path` | `nav_msgs/Path` | 轨迹（`path_en` 开启时） |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 全局系配准点云，`dense_publish_en` 控制稠密/稀疏 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | IMU 体系点云（`scan_bodyframe_pub_en`） |
| `/cloud_effected` | `sensor_msgs/PointCloud2` | 参与配准的有效点 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | 累计地图（每 20 帧发一次） |
| TF | `camera_init` → `body` | 与 `/Odometry` 同步广播 |

`msg/Pose6D.msg` 描述的是 IMU 预积分状态（offset_time、acc、gyr、vel、pos、rot），在 `IMU_Processing.hpp` 内部用于去畸变时的位姿序列。

## 6. 关键参数说明

| 参数 | 位置 | 默认（以 velodyne.yaml 为例） | 含义 |
|---|---|---|---|
| `lid_topic` / `imu_topic` | common | `/velodyne_points` / `/imu/data` | 输入话题 |
| `time_sync_en` / `time_offset_lidar_to_imu` | common | false / 0.0 | 软件时间同步与固定时间偏移（优先用硬件同步） |
| `lidar_type` | preprocess | 1=Avia 2=Velodyne 3=Ouster 4=MARSIM | 雷达类型 |
| `scan_line` / `scan_rate` / `timestamp_unit` | preprocess | 32 / 10 Hz / 2(µs) | 线数、帧率、点时间戳单位（0 s, 1 ms, 2 µs, 3 ns） |
| `blind` | preprocess | 2 m | 近距盲区 |
| `acc_cov` / `gyr_cov` | mapping | 0.1 | IMU 测量噪声协方差 |
| `b_acc_cov` / `b_gyr_cov` | mapping | 0.0001 | 零偏随机游走协方差 |
| `fov_degree` / `det_range` | mapping | 180 / 100 m | 局部地图 FOV 与探测半径 |
| `extrinsic_est_en` | mapping | false | 是否在线估计 LiDAR-IMU 外参 |
| `extrinsic_T` / `extrinsic_R` | mapping | [0,0,0.28] / 单位阵 | 外参：**LiDAR 在 IMU 系下的位姿**（IMU 为基准） |
| `max_iteration` | launch | 3~4 | IEKF 最大迭代次数 |
| `filter_size_surf` / `filter_size_map` | launch | 0.5 / 0.5 m | 帧点云与地图的体素降采样尺寸 |
| `cube_side_length` | launch | 1000 m | 局部地图立方体边长 |
| `point_filter_num` | launch | 2~4 | 预处理隔点抽样 |
| `path_en` / `scan_publish_en` / `dense_publish_en` / `scan_bodyframe_pub_en` | publish | — | 各输出开关 |
| `pcd_save_en` / `interval` | pcd_save | — | 退出时保存地图到 `PCD/scans.pcd`；interval=-1 全部存一个文件 |

## 7. 依赖库

**系统**：Ubuntu ≥ 16.04（实测 20.04）、ROS ≥ Melodic（本工作区 Noetic）、C++14、OpenMP、PythonLibs（仅调试用 matplotlibcpp）。

**ROS 包**：roscpp、rospy、std_msgs、sensor_msgs、geometry_msgs、nav_msgs、tf、pcl_ros、pcl_conversions、eigen_conversions、message_generation/message_runtime，以及 **livox_ros_driver**（即使只用 Velodyne 也必须安装并 source，因为 `CustomMsg` 消息类型参与编译）。

**第三方库**：

- PCL ≥ 1.8（点云 I/O、体素滤波、KD 树）
- Eigen ≥ 3.3.4
- ikd-Tree（git 子模块，仓库自带引用，`git submodule update --init` 拉取）
- IKFoM 工具库（已内置于 `include/IKFoM_toolkit`，无需另装）

## 8. 编译与运行

### 8.1 编译

```bash
# 前置：已安装并 source livox_ros_driver（ws_livox 工作区）
cd /home/zjs/JZJ_local/src/location/FAST_LIO
git submodule update --init        # 拉取 include/ikd-Tree，当前目录为空，必须执行

cd /home/zjs/JZJ_local
catkin_make
source devel/setup.bash
```

CMake 会按 CPU 核数自动启用 OpenMP（> 4 核时并行度 3）。默认 `CMAKE_BUILD_TYPE=Debug`，追求性能可改为 `Release`。

### 8.2 Livox Avia / MID-360

```bash
roslaunch fast_lio mapping_avia.launch        # 或 mapping_mid360.launch
roslaunch livox_ros_driver livox_lidar_msg.launch   # 必须用 msg 版驱动（带逐点时间戳）
```

### 8.3 Velodyne / Ouster

先改 `config/velodyne.yaml`：`lid_topic`、`imu_topic`、`timestamp_unit`（按点云 time/t 字段单位）、`scan_line`、`extrinsic_T/R`，然后：

```bash
roslaunch fast_lio mapping_velodyne.launch    # Ouster 用 mapping_ouster64.launch
# 再启动雷达驱动或播放 rosbag
rosbag play your_data.bag
```

### 8.4 MARSIM 仿真

```bash
roslaunch test_interface single_drone_avia.launch   # MARSIM 侧
roslaunch fast_lio mapping_marsim.launch
```

### 8.5 地图保存与查看

launch 或 yaml 中置 `pcd_save_en: 1`，Ctrl+C 结束节点后全部帧累加保存到 `FAST_LIO/PCD/scans.pcd`：

```bash
pcl_viewer PCD/scans.pcd    # 按 1~5 切换着色方式
```

### 8.6 结果查看

```bash
rostopic echo /Odometry
rosrun rviz rviz -d src/location/FAST_LIO/rviz_cfg/loam_livox.rviz
```

`Log/pos_log.txt` 记录每帧状态（姿态、位置、速度、零偏、重力），`Log/plot.py` 可画耗时曲线。

## 9. 注意事项

- **时间同步是硬性要求**：IMU 与 LiDAR 必须硬件同步；`time_sync_en` 的软件同步只在实在无法硬件同步时使用，精度无保证。已知固定偏移可设 `time_offset_lidar_to_imu`。
- **逐点时间戳必需**：出现 `Failed to find match for field 'time'` 警告说明点云缺时间字段，去畸变会失效。Livox 务必用 `livox_lidar_msg.launch`。
- **启动时保持静止**：IMU 初始化假设静止，初始化期间晃动会带偏重力与零偏估计。
- 外参已知时把 `extrinsic_est_en` 关掉，收敛更快更稳；外参未知可先用 LI-Init 标定。
- 子模块未拉取（`include/ikd-Tree` 为空）是编译失败的最常见原因。
- 当前 CMake 配置为 Debug 构建，实物部署建议改 Release。
