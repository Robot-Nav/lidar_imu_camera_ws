#!/bin/bash
# ============================================================
# direct_visual_lidar_calibration 源码编译安装脚本（更新版）
# 目录布局:
#   ~/lidar_imu_camera_ws/third_party/{gtsam,ceres-solver,iridescence}
#   ~/lidar_imu_camera_ws/src/direct_visual_lidar_calibration
# 用法: bash ~/lidar_imu_camera_ws/setup_dvlc.sh
# 各步骤断点续跑，会提示 sudo 密码
# ============================================================
set -e

WS=$HOME/lidar_imu_camera_ws
DEPS=$WS/third_party
TOOL=$WS/src/direct_visual_lidar_calibration

echo "========== [1/6] 修复 /usr/local 残缺 GLFW（让系统 3.3.2 生效）=========="
if [ -f /usr/local/include/GLFW/glfw3.h ]; then
  if ! grep -q glfwGetMonitorWorkarea /usr/local/include/GLFW/glfw3.h 2>/dev/null; then
    echo "  检测到 /usr/local GLFW 缺 glfwGetMonitorWorkarea，移除之..."
    sudo rm -rf /usr/local/include/GLFW
    sudo rm -f /usr/local/lib/libglfw3.a
    sudo rm -rf /usr/local/lib/cmake/glfw3 /usr/local/lib/pkgconfig/glfw3.pc
    sudo ldconfig
  else
    echo "  /usr/local GLFW 完整，无需处理"
  fi
else
  echo "  /usr/local 无 GLFW"
fi

echo "========== [2/6] 初始化工具子模块 =========="
cd "$TOOL"
git submodule update --init --recursive

echo "========== [3/6] apt 安装公共依赖 =========="
sudo apt update
sudo apt install -y \
  libomp-dev libboost-all-dev libglm-dev libglfw3-dev \
  libpng-dev libjpeg-dev libfmt-dev \
  ros-noetic-cv-bridge ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions ros-noetic-camera-info-manager

echo "========== [4/6] GTSAM / Ceres（已装则跳过）=========="
if [ ! -f /usr/local/lib/libgtsam.so ] && [ ! -f /usr/local/lib/libgtsam.so.4 ]; then
  cd "$DEPS/gtsam" && mkdir -p build && cd build
  cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF \
           -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
  make -j$(nproc) && sudo make install && sudo ldconfig
else
  echo "  GTSAM 已安装，跳过"
fi

if [ ! -f /usr/local/lib/libceres.so ] && [ ! -f /usr/local/lib/libceres.a ]; then
  cd "$DEPS/ceres-solver" && mkdir -p build && cd build
  cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF
  make -j$(nproc) && sudo make install && sudo ldconfig
else
  echo "  Ceres 已安装，跳过"
fi

echo "========== [5/6] 编译安装 Iridescence =========="
if [ ! -f /usr/local/lib/libiridescence.so ]; then
  cd "$DEPS/iridescence"
  rm -rf build && mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j$(nproc)
  sudo make install
  sudo ldconfig
else
  echo "  Iridescence 已安装，跳过"
fi

echo "========== [6/6] catkin 编译工具本体（主工作空间）=========="
cd "$WS"
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release

echo "================================================"
echo "安装完成。验证:"
echo "  source $WS/devel/setup.bash"
echo "  rospack find direct_visual_lidar_calibration"
echo "================================================"
