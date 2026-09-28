# UniUbi 媒体驱动

本包还提供 [RTSP 视频后端](RTSP.zh-CN.md)，用于通用 ROS 图像接入。该路径目前使用 CPU 解码；对低延迟、高帧率或 GPU 感知有要求时，建议直接接入 RTSP，并按设备能力显式配置硬解。选型与开销见 [RTSP 适用场景](RTSP.zh-CN.md#适用场景与性能代价)。下文的 JPEG 直接转发说明针对 MediaBus 后端。

该 ROS 2 驱动封装 UniUbi MediaBus 的 PCM 音频采集/播放和两路板载前置摄像头。驱动直接转发
MediaBus 已编码的 JPEG 帧，不进行解码或二次编码。

PCM 话题、播放/重置、音量及 host 部署见[音频指南](AUDIO.zh-CN.md)。

## 摄像头话题

| Topic | 类型 | MediaBus 通道 |
|---|---|---:|
| `/front_camera_0/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 0 |
| `/front_camera_1/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 1 |

两路消息的 `format` 均为 `jpeg`。通道号只用于区分两路前置摄像头，不代表左右位置。
产品确定物理映射后，可同步覆盖 `camera_names`、`camera_channels` 和 `frame_ids`。

默认 `lazy_subscription: true`：只有对应 ROS 2 topic 存在订阅者时，才启动该路
MediaBus 编码帧订阅。QoS 使用传感器数据风格：best effort、volatile、depth 1。

## 平台和权限

外部主机通过 RTSP 接入视频时，请选择本包的 [RTSP 后端](RTSP.zh-CN.md)，
可不依赖机器人 SDK 构建。下述本机限制仅适用于 MediaBus 视频后端。

视频必须运行在机器人本机 aarch64 板端，使用本地共享内存。音频也支持外部 x86 和
ARM64 host，见[音频指南](AUDIO.zh-CN.md)。本机模式下，进程必须有权访问 `/tmp/roudi` 和 MediaBus 共享内存资源。生产部署
应配置合适的服务账号、用户组或 ACL，不应仅为绕过权限问题而让整个应用长期以 root 运行。

## 构建和运行

默认同时编译 MediaBus（视频/PCM 音频）和 RTSP，无需选择构建开关。先准备 ROS 2 Humble，并安装系统依赖：

```bash
sudo apt install libavformat-dev libavcodec-dev \
  libavutil-dev libswscale-dev pkg-config
```

先安装 `uniubi_robot_sdk`，确保 CMake 能找到 `UniubiRobotSdkConfig.cmake`，再在
ROS 2 工作区构建：

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

构建后，视频启动时选择来源；上述默认命令启动本机 MediaBus，远程 RTSP 使用：

```bash
ros2 launch uniubi_media_driver media_driver.launch.py \
  video_backend:=rtsp host:=192.168.1.10
```

普通 ARM64 外部主机安装 SDK 时使用 `-DPLATFORM=aarch64_host`；SDK 库目录随部署选择 `aarch64`（大脑）、`x86_64` 或 `aarch64_host`。精简构建和旧缓存处理见 [RTSP 说明](RTSP.zh-CN.md)。

检查一路图像：

```bash
ros2 topic echo /front_camera_0/image_raw/compressed --once --field format \
  --qos-reliability best_effort
ros2 topic hz /front_camera_0/image_raw/compressed \
  --qos-reliability best_effort
```

## 何时直接使用 SDK

该包面向需要 ROS 2 图像、远程显示或录制 JPEG 的普通开发者。板端感知、raw
NV12/NV21、低拷贝 GPU 流水线、精确 plane/stride 处理或完整编码元数据等
专业场景，应直接使用 C++/Python SDK 的 MediaBus API。

> ROS 2 媒体驱动面向通用开发和快速集成，并不替代完整 MediaBus SDK。原始图像
> 及专业板端感知场景建议直接集成 SDK。
