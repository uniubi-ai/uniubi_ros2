# UniUbi Media Driver

This package also provides an [RTSP video backend](RTSP.md) for general ROS image integration. It currently uses CPU decoding. For low latency, high frame rates, or GPU perception, consider direct RTSP integration with explicitly configured hardware decoding where supported. See [intended use and performance costs](RTSP.md#intended-use-and-performance-costs). The JPEG forwarding behavior below describes the MediaBus backend.

ROS 2 driver for PCM audio capture/playback and the two on-board front cameras exposed by UniUbi MediaBus.
It forwards existing JPEG frames without decoding or re-encoding them.

See the [audio guide](AUDIO.md) for PCM topics, playback/reset, volume, and host deployment.

## Camera topics

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

For RTSP video on external hosts, select this package's [RTSP backend](RTSP.md),
which can be built without the robot SDK. The local-only restrictions below apply
to the MediaBus video backend.

Video must run locally on the robot's aarch64 board using shared memory.
Audio also supports remote x86 and ARM64 hosts; see the [audio guide](AUDIO.md).
For local deployment:
The process must be allowed to access `/tmp/roudi` and the MediaBus shared-memory
resources. Configure an appropriate service account/group/ACL for production;
do not grant an entire application root privileges solely as a workaround.

## Build and run

The default build includes MediaBus (video/PCM audio) and RTSP without backend flags. Prepare ROS 2 Humble and install the system dependencies:

```bash
sudo apt install libavformat-dev libavcodec-dev \
  libavutil-dev libswscale-dev pkg-config
```

Install `uniubi_robot_sdk` first so that CMake can find
`UniubiRobotSdkConfig.cmake`, then build the package in a ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
cmake -S ~/uniubi_robot_sdk -B /tmp/uniubi_robot_sdk_build -DBUILD_SDK_CPP_EXAMPLES=OFF
cmake --install /tmp/uniubi_robot_sdk_build --prefix ~/uniubi_robot_sdk_install

cd ~/ros2_ws
export CMAKE_PREFIX_PATH="$HOME/uniubi_robot_sdk_install:${CMAKE_PREFIX_PATH:-}"
colcon build --packages-select uniubi_media_driver
source install/setup.bash
export LD_LIBRARY_PATH="$HOME/uniubi_robot_sdk_install/lib/aarch64:${LD_LIBRARY_PATH:-}"
ros2 launch uniubi_media_driver media_driver.launch.py
```

Select the video source at launch. The command above starts local MediaBus; for remote RTSP use:

```bash
ros2 launch uniubi_media_driver media_driver.launch.py \
  video_backend:=rtsp host:=192.168.1.10
```

For an external ARM64 host, install the SDK with `-DPLATFORM=aarch64_host`; choose the matching SDK library directory: `aarch64` (brain), `x86_64`, or `aarch64_host`. See the [RTSP guide](RTSP.md) for minimal builds and old build caches.

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
MediaBus API directly for on-board perception, raw NV12/NV21 frames,
minimum-copy GPU pipelines, exact plane/stride handling, or full codec metadata.

> The ROS 2 media driver is intended for general development and rapid
> integration; it does not replace the complete MediaBus SDK. Use the SDK
> directly for raw images and professional on-board perception
> pipelines.
