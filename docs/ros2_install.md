# Install ROS 2 Humble

**English** | [简体中文](ros2_install.zh-CN.md)

This repository targets ROS 2 Humble on Ubuntu 22.04 (Jammy). The same Debian
packages are available for the robot's aarch64 Orin board and x86_64 Ubuntu
development machines.

## 1. Configure the ROS 2 package source

Follow the official [ROS 2 Humble Ubuntu installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
to enable the Ubuntu Universe repository and install the current ROS 2 APT
source package. Use the official procedure rather than copying an old signing
key or repository snapshot from another robot.

## 2. Install the board/development environment

The following lightweight package set is sufficient to build and run the
packages in this repository without desktop GUI tools:

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

`ros-humble-desktop` is optional. Install it on a development machine only when
RViz, rqt, and other GUI tools are required; it is not required by the on-board
motion or media nodes.

## 3. Source and verify

Every new shell must source the ROS environment before running `ros2` or
`colcon`:

```bash
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
ros2 pkg prefix rclcpp
ros2 pkg prefix rmw_cyclonedds_cpp
colcon --help >/dev/null
```

Expected `ROS_DISTRO` is `humble`. To source it automatically for interactive
Bash shells:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

Do not rely on `.bashrc` in scripts, services, or non-interactive SSH commands;
source `/opt/ros/humble/setup.bash` explicitly there.

## 4. Select DDS settings

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

When the machine has multiple network interfaces, inspect them with
`ip -br addr` and set `CYCLONEDDS_URI` to the interface connected to the robot.
See the repository [README](../README.md#prerequisites) for an example.

## 5. Build the workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
