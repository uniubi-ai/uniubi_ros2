# Install ROS 2 Humble

**English** | [简体中文](ros2_install.zh-CN.md)

This repository targets ROS 2 Humble on Ubuntu 22.04 (Jammy). Architecture-matched Debian
packages can be used on the **robot's Orin brain itself** and external x86_64/ARM64 Linux hosts.
The robot brain does not mean the cerebellum; an external ARM64 host is not the robot brain
merely because it shares the CPU architecture.

## 1. Configure the ROS 2 package source

Follow the official [ROS 2 Humble Ubuntu installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
to enable the Ubuntu Universe repository and install the current ROS 2 APT
source package. Use the official procedure rather than copying an old signing
key or repository snapshot from another robot.

## 2. Install the Orin brain or external-host development environment

The following packages provide the base ROS 2 development environment without desktop GUI tools.
The default media package build also needs the robot SDK and FFmpeg development libraries;
see the [media driver build guide](../src/uniubi_media_driver/README.md#build-and-run):

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-cyclonedds \
  ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  ros-humble-ament-cmake \
  ros-humble-rclcpp \
  ros-humble-rclpy \
  python3-rosdep \
  build-essential cmake git
```

`ros-humble-desktop` is optional. Install it on a development machine only when
RViz, rqt, and other GUI tools are required; it is not required by motion or media nodes
on the robot's Orin brain or external hosts.

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

The Domain must match where the process runs:

| Runtime location | `ROS_DOMAIN_ID` | RPC service |
|---|---:|---|
| Robot brain (Orin) | `1` | `cerebellumServer` |
| Remote PC/development host | `42` | `robotServer` |

For remote-PC communication, the robot must be connected to Wi-Fi and reachable from the PC. ROS 2 environment variables on the PC do not replace the robot's network connection.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
```

Set `ROS_DOMAIN_ID` from the table. Changing the Domain without changing the RPC service selects the wrong endpoint.
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
