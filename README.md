# Uniubi ROS 2

**English** | [简体中文](README.zh-CN.md)

ROS 2 integration for Uniubi robots, including a motion-control bridge, a reusable C++ ROS 2 client, direct DDS / ROS 2 protocol interfaces, and an on-board MediaBus camera driver.

The original robotServer `.msg` / `.srv` definitions come from [`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs). The ROS 2 package is named `uniubi`, and its interface type prefix is also `uniubi`. Bridge-specific `MotionStatus.msg` and `StartMotionAction.srv` definitions are maintained by `uniubi_motion_bridge`. The three motion-integration modes communicate with robotServer through ROS 2 services and DDS topics without linking `librobotMotionSdk.so`. The separate `uniubi_media_driver` links the SDK locally on the aarch64 board because MediaBus is a shared-memory interface rather than a remote robotServer topic.

## Start here

The repository provides one default application-facing entry point and two advanced options:

| Option | Intended users | Control ownership | Main interfaces | Recommendation |
|---|---|---|---|---|
| Motion bridge | Typical ROS 2 application nodes | The bridge acquires, renews, and releases control automatically | `/motion/*`, `/cmd_vel`, standard sensor topics | Recommended |
| `uniubi_motion_client` | C++ developers who need more high-level motion capabilities | The application explicitly calls `connect/startControl/releaseControl` | C++ methods and callbacks | Advanced |
| Direct DDS / ROS 2 protocol | Raw data, protocol maintenance, and cross-framework integration | Not required for read-only data; control lifecycle is application-managed | RPC, Event, raw topics, TRC | Protocol-level |

See [ROS 2 integration modes](docs/ros2_usage_modes.md) for a complete comparison and selection guide.

Direct DDS / ROS 2 protocol integration is one complete low-level approach containing RPC, Event, data topics, the control-ownership lifecycle, and TRC. It is not two separate alternatives named “Direct DDS” and “Direct RPC.” See the [Direct DDS / ROS 2 API](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.md) for the protocol contract.

> **Feature scope:** The Motion bridge focuses on common motion control and selectively exposes standard ROS 2 observation interfaces for odometry, joints, IMU, and battery data. It is not a complete ROS 2 mapping of the High Level Client or the low-level DDS / ROS 2 protocol. Camera frames are intentionally provided by the independent `uniubi_media_driver`, not by the Motion bridge.

If you only need odometry, joint, IMU, or battery data, subscribe to the standard topics published by the bridge. Read-only observation does not require motion control ownership.

## Repository layout

```text
uniubi_robot_msgs
└── uniubi                    # ROS 2 msg/srv interface package built from ros2/

uniubi_ros2
├── uniubi_motion_client      # Source-level RPC/DDS C++ wrapper, not an SDK shared library
├── uniubi_motion_bridge      # Application-facing node and bridge-specific msg/srv definitions
└── uniubi_media_driver       # On-board MediaBus JPEG camera driver
```

The bridge reuses `uniubi_motion_client` internally:

```text
Application ROS 2 node
  ├── /motion/* services
  ├── /cmd_vel
  └── standard observation topics
            ↓
uniubi_motion_bridge
            ↓
uniubi_motion_client
            ↓
uniubi/srv/System + DDS topics
            ↓
robotServer / MotionServer
```

## Prerequisites

- ROS 2 Humble is installed and sourced.
- The development machine or Orin is on the same discoverable network and DDS Domain as the robot.
- You know the target robot's `device_id`. It is the `deviceNo` in device information (the robot SN). This field routes RPC calls but cannot isolate raw DDS topics.
- Use a separate `ROS_DOMAIN_ID` for each robot. Do not place multiple robots and their bridges in the same Domain.
- Cyclone DDS is recommended.

For a board and development-machine package list, environment setup, and
verification commands, see [Install ROS 2 Humble](docs/ros2_install.md).

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

If the machine has multiple network interfaces, use `CYCLONEDDS_URI` to select the interface connected to the robot. On Orin, the robot VLAN interface is `eth0.100`:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="eth0.100"/></Interfaces></General></Domain></CycloneDDS>'
```

Interface names differ on other platforms. Run `ip -br addr`, identify the interface connected to the robot network, and replace `eth0.100` above.

## Build

```bash
mkdir -p ~/ros2_ws/src

git clone https://github.com/uniubi-ai/uniubi_robot_msgs.git ~/uniubi_robot_msgs
cp -r ~/uniubi_robot_msgs/ros2 ~/ros2_ws/src/uniubi

git clone https://github.com/uniubi-ai/uniubi_ros2.git ~/uniubi_ros2
cp -r ~/uniubi_ros2/src/uniubi_motion_client ~/ros2_ws/src/
cp -r ~/uniubi_ros2/src/uniubi_motion_bridge ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select uniubi uniubi_motion_client uniubi_motion_bridge
. install/setup.bash
```

`uniubi_media_driver` is an optional board-local package with a separate SDK dependency. See
[`src/uniubi_media_driver/README.md`](src/uniubi_media_driver/README.md) for its build and runtime setup.

## Recommended: Motion bridge

Start the bridge:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
# Use eth0.100 on Orin; replace it with the actual robot-network interface elsewhere
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="eth0.100"/></Interfaces></General></Domain></CycloneDDS>'
export ROBOT_DEVICE_ID="$(python3 -c \
  'import json; print(json.load(open("/tmp/deviceInfo"))["deviceNo"])')"

ros2 launch uniubi_motion_bridge motion_bridge.launch.py \
  device_id:="$ROBOT_DEVICE_ID"
```

The robot runtime provides `/tmp/deviceInfo`; its `deviceNo` is the target robot SN required by the bridge. On machines with multiple interfaces, also set `CYCLONEDDS_URI` as described under Prerequisites.

Starting the bridge only connects to robotServer. It does not acquire motion control immediately. The first `/motion/start_action` call acquires control and starts lease maintenance. Successful control RPCs refresh the lease directly; an extra renewal RPC is sent only while control calls are idle.

A minimal control sequence:

```bash
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: walking, params_json: '{\"lineVelocityX\":0.0,\"lineVelocityY\":0.0,\"velocity\":0.0}'}"

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}'

ros2 service call /motion/stop_action std_srvs/srv/Trigger '{}'
ros2 service call /motion/release_control std_srvs/srv/Trigger '{}'
```

`/cmd_vel` only supplies velocity parameters to the current action. It does not acquire control, start an action, or switch actions. Under ROS REP-103 and the Uniubi motion convention, `linear.y > 0` means moving left; the bridge does not invert the sign.

### Public interfaces

Services:

```text
/motion/start_action
/motion/stop_action
/motion/release_control
/motion/emergency_stop
/motion/query_capabilities
```

Topics:

```text
/motion/status
/cmd_vel
/odom
/joint_states
/imu/data
/battery_state
```

See the [Motion bridge guide](docs/motion_bridge.md) for fields, ownership lifecycle, `cmd_vel`, status updates, and action semantics.

## Other integration modes

- To call high-level motion methods directly from your own C++ node, use [`uniubi_motion_client`](docs/ros2_usage_modes.md#option-2-uniubi_motion_client-c-client).
- To subscribe to raw messages, debug QoS/type mappings, or add protocol interfaces, use the [direct DDS / ROS 2 protocol](docs/ros2_usage_modes.md#option-3-direct-dds--ros-2-protocol).
- To subscribe to complete read-only sensor observations (GPS, UWB, and Walk odometry):

```bash
ROS_DOMAIN_ID=42 \
UNIUBI_TEST_SENSOR_OBSERVED_TOPIC=/sensor/observed \
ros2 run uniubi_motion_client sensor_observed_subscriber
```

## Front cameras

For ordinary ROS 2 applications, run the independent `uniubi_media_driver` locally on the robot's
aarch64 board. It forwards the two existing MediaBus JPEG streams without re-encoding:

```text
/front_camera_0/image_raw/compressed
/front_camera_1/image_raw/compressed
```

Both topics use `sensor_msgs/msg/CompressedImage` with `format: jpeg`. They identify two front-facing
cameras; channel numbers do not claim a left/right mapping. The driver uses best-effort, volatile,
depth-1 QoS and starts a camera stream only when that topic has a subscriber.

Professional on-board perception developers should use the C++ or Python SDK MediaBus API directly
for raw NV12/NV21, audio, minimum-copy GPU processing, plane/stride access, and complete codec metadata.
See the [media driver guide](src/uniubi_media_driver/README.md).

## Documentation

| Document | Audience | Contents |
|---|---|---|
| [README](README.md) | All developers | Selection, installation, and recommended quick start |
| [Install ROS 2 Humble](docs/ros2_install.md) | First-time users and board integrators | Ubuntu 22.04 packages, environment loading, and verification |
| [ROS 2 integration modes](docs/ros2_usage_modes.md) | Architects and advanced developers | Workflows, trade-offs, and selection guidance for all three modes |
| [Motion bridge guide](docs/motion_bridge.md) | Application developers | `/motion/*`, `/cmd_vel`, status, and observation interfaces |
| [Media driver guide](src/uniubi_media_driver/README.md) | ROS 2 camera users | Two front-camera JPEG topics, parameters, platform constraints, and SDK boundary |
| [Runtime notes](docs/runtime_notes.md) | Integration and protocol developers | DDS, device matching, asynchronous semantics, and safety boundaries |
| [Direct DDS / ROS 2 API](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.md) | Protocol developers | Raw RPC, DDS topics, and field contracts |
| [ROS 2 and DDS mappings](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/ros2_dds_interop_overview.md) | Interface maintainers | `.msg/.srv` to DDS IDL mapping rules |
| [`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs) | Message maintainers | Authoritative ROS 2 msg/srv and DDS IDL definitions |

See the [documentation reading guide](docs/README.md) for additional navigation.

## Safety

For initial High Level hardware integration, validate read-only topics first, then validate ownership, action startup, and status feedback by starting `walking` with all three velocity fields explicitly set to zero. `standing` and `laying` depend on the current posture and the server state machine, so they are not a universal round-trip test. Test walking with nonzero velocity, bipedal stance, handstands, and `jump*` actions only in an open area with an accessible emergency stop and an operator present.

`stop_action` does not release control. Call `/motion/release_control` explicitly when the application finishes.

`stop_action` stops the current action and asynchronously returns the effective action to zero-speed
`walking` while retaining control. Starting `walking` with full zero parameters is the equivalent explicit
transition. `/cmd_vel` updates the current action's supported velocity parameters (including actions such as
`bipedStand` and `handstand`) but does not switch or stop the action.

## License

Uniubi-authored ROS 2 integration code, examples, and documentation in this repository are licensed under the Apache License 2.0. Vendored jsoncpp remains under its original license. See [LICENSE](LICENSE), [NOTICE](NOTICE), and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
