#!/usr/bin/env bash
set -e

echo "=========================================================="
echo " [1/4] Thêm Repository ROS 2 Jazzy & Gazebo Harmonic"
echo "=========================================================="
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release build-essential git
sudo add-apt-repository universe -y

# Thêm ROS 2 Jazzy Key & Repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Thêm Gazebo Harmonic Key & Repository
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

echo "=========================================================="
echo " [2/4] Cập nhật danh sách gói apt"
echo "=========================================================="
sudo apt update

echo "=========================================================="
echo " [3/4] Cài đặt ROS 2 Jazzy, Gazebo Harmonic và Dependencies"
echo "=========================================================="
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-turtlebot3-description \
  ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-rosbag2-storage-mcap \
  libgz-sim8-dev \
  libgz-plugin2-dev \
  libgz-cmake3-dev \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-numpy \
  python3-scipy \
  python3-matplotlib \
  python3-pip

echo "=========================================================="
echo " [4/4] Cấu hình rosdep & bashrc"
echo "=========================================================="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update || true

# Cấu hình tự động source trong ~/.bashrc nếu chưa có
if ! grep -q "/opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
  echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
  echo "source /home/cuongbien/DynamuhSim/install/setup.bash" >> ~/.bashrc
fi

echo ""
echo "=========================================================="
echo " Chúc mừng! Quá trình cài đặt đã hoàn tất thành công."
echo " Bây giờ bạn có thể chạy: colcon build --symlink-install"
echo "=========================================================="
