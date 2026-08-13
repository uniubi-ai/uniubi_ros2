# UniUbi Media Driver

ROS 2 driver for the two on-board front cameras exposed by UniUbi MediaBus.
It forwards existing JPEG frames without decoding or re-encoding them.

## Topics

| Topic | Type | MediaBus channel |
|---|---|---:|
| `/front_camera_0/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 0 |
| `/front_camera_1/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 1 |

Both messages use `format: jpeg`. Channel numbers identify the two front-facing
cameras; they do not imply left/right placement. Override `camera_names`,
`camera_channels`, and `frame_ids` together when a product has an authoritative
physical mapping.

The default `lazy_subscription: true` starts each MediaBus encoder subscription
only while the corresponding ROS 2 publisher has a subscriber. QoS is sensor
data style: best effort, volatile, and depth 1.

## Platform and permissions

The driver must run locally on the robot's aarch64 board. MediaBus uses local
shared memory and is not available through a remote or multi-device SDK setup.
The process must be allowed to access `/tmp/roudi` and the MediaBus shared-memory
resources. Configure an appropriate service account/group/ACL for production;
do not grant an entire application root privileges solely as a workaround.

## Build and run

Install `uniubi_robot_sdk` first so that CMake can find
`UniubiRobotSdkConfig.cmake`, then build the package in a ROS 2 workspace:

```bash
cmake -S ~/uniubi_robot_sdk -B /tmp/uniubi_robot_sdk_build
cmake --install /tmp/uniubi_robot_sdk_build --prefix ~/uniubi_robot_sdk_install

cd ~/ros2_ws
colcon build --packages-select uniubi_media_driver \
  --cmake-args -DCMAKE_PREFIX_PATH="$HOME/uniubi_robot_sdk_install"
. install/setup.bash
export LD_LIBRARY_PATH="$HOME/uniubi_robot_sdk_install/lib/aarch64:${LD_LIBRARY_PATH:-}"
ros2 launch uniubi_media_driver media_driver.launch.py
```

Inspect one stream:

```bash
ros2 topic echo /front_camera_0/image_raw/compressed --once --field format \
  --qos-reliability best_effort
ros2 topic hz /front_camera_0/image_raw/compressed \
  --qos-reliability best_effort
```

## When to use the SDK directly

This package is the convenient ROS 2 path for ordinary application developers,
remote visualization, and recording JPEG frames. Use the C++ or Python SDK
MediaBus API directly for on-board perception, raw NV12/NV21 frames, audio,
minimum-copy GPU pipelines, exact plane/stride handling, or full codec metadata.

> The ROS 2 media driver is intended for general development and rapid
> integration; it does not replace the complete MediaBus SDK. Use the SDK
> directly for audio, raw images, and professional on-board perception
> pipelines.
