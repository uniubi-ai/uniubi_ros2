# 安装 ROS 2 Humble

[English](ros2_install.md) | **简体中文**

本仓库面向 Ubuntu 22.04（Jammy）上的 ROS 2 Humble。机器人 aarch64 Orin 板端和
x86_64 Ubuntu 开发机均可使用对应架构的 Debian 软件包。

## 1. 配置 ROS 2 软件源

按 ROS 官方的 [ROS 2 Humble Ubuntu 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
启用 Ubuntu Universe 仓库，并安装当前版本的 ROS 2 APT source package。不要从其他
机器人复制可能已经过期的签名密钥或软件源快照。

## 2. 安装板端/开发环境

以下轻量包组合足以构建和运行本仓库，无需安装桌面 GUI：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  ros-humble-ament-cmake \
  ros-humble-rclcpp \
  ros-humble-rclpy \
  python3-rosdep \
  build-essential cmake git
```

`ros-humble-desktop` 是可选项。只有开发机需要 RViz、rqt 等 GUI 工具时才安装；板端
运动节点和媒体节点都不依赖它。

## 3. 加载并验证环境

每个新 shell 在使用 `ros2` 或 `colcon` 前都要先加载 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
ros2 pkg prefix rclcpp
ros2 pkg prefix rmw_cyclonedds_cpp
colcon --help >/dev/null
```

`ROS_DISTRO` 应输出 `humble`。如果希望交互式 Bash 自动加载：

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

脚本、systemd 服务和非交互 SSH 命令不应依赖 `.bashrc`，应显式执行
`source /opt/ros/humble/setup.bash`。

## 4. 选择 DDS 配置

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

设备存在多个网卡时，先用 `ip -br addr` 确认连接机器人网络的网卡，再设置
`CYCLONEDDS_URI`。示例见仓库 [README](../README.zh-CN.md#前置条件)。

## 5. 构建工作区

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
