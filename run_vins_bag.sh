#!/bin/bash
# ============================================================
# 离线运行 VINS-Mono（最终配置，外参已固定）：自动打开 4 个终端窗口
#   1-roscore : ROS Master
#   2-VINS    : vins_estimator + feature_tracker（显示初始化日志）
#   3-RViz    : 可视化轨迹与特征（可在此录屏）
#   4-rosbag  : 回放 bag（默认 0.5 倍速）
#
# 用法:
#   bash run_vins_bag.sh <bag文件路径> [倍速]
#   例:
#   bash run_vins_bag.sh ~/lidar_imu_camera_ws/bags/d435i/vins/vins_calib.bag
#   bash run_vins_bag.sh ~/lidar_imu_camera_ws/bags/d435i/vins/vins_calib.bag 1.0
#
# 注意:
#   - 运行前先 Ctrl+C 停掉相机节点（避免实时话题干扰 bag 回放）
#   - 使用最终配置 d435i_final.yaml（estimate_extrinsic=0），外参已固化
# ============================================================

BAG="${1:?用法: $0 <bag文件路径> [倍速]}"
RATE="${2:-0.5}"
WS="$HOME/lidar_imu_camera_ws"
CFG="$WS/src/VINS-Mono/config/d435i/d435i_final.yaml"

[ -f "$BAG" ] || { echo "错误：找不到 bag 文件 $BAG"; exit 1; }
command -v gnome-terminal >/dev/null || { echo "错误：未找到 gnome-terminal"; exit 1; }

if pgrep -f "realsense2_camera" >/dev/null 2>&1; then
  echo "警告：realsense2_camera 仍在运行，建议先 Ctrl+C 停掉相机节点再继续。"
fi

open() { gnome-terminal --title="$1" -- bash -c "$2"; }

open "1-roscore" 'source /opt/ros/noetic/setup.bash; roscore'
sleep 3

open "2-VINS" "source /opt/ros/noetic/setup.bash; source $WS/devel/setup.bash; rosparam set use_sim_time true; roslaunch vins_estimator euroc.launch config_path:=$CFG"
sleep 3

open "3-RViz" "source /opt/ros/noetic/setup.bash; source $WS/devel/setup.bash; roslaunch vins_estimator vins_rviz.launch"
sleep 3

open "4-rosbag" "source /opt/ros/noetic/setup.bash; rosbag play --clock -r $RATE '$BAG'"

echo "================================================"
echo "已启动 4 个窗口。在 2-VINS 窗口等待:"
echo "  Initialization finish!"
echo "在 3-RViz 窗口查看/录屏轨迹。"
echo "================================================"
