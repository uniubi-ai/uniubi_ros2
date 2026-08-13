# UniUbi 媒体驱动

该 ROS 2 驱动封装 UniUbi MediaBus 提供的两路板载前置摄像头。驱动直接转发
MediaBus 已编码的 JPEG 帧，不进行解码或二次编码。

## Topics

| Topic | 类型 | MediaBus 通道 |
|---|---|---:|
| `/front_camera_0/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 0 |
| `/front_camera_1/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 1 |

两路消息的 `format` 均为 `jpeg`。通道号只用于区分两路前置摄像头，不代表左右位置。
产品确定物理映射后，可同步覆盖 `camera_names`、`camera_channels` 和 `frame_ids`。

默认 `lazy_subscription: true`：只有对应 ROS 2 topic 存在订阅者时，才启动该路
MediaBus 编码帧订阅。QoS 使用传感器数据风格：best effort、volatile、depth 1。

## 平台和权限

驱动必须运行在机器人本机 aarch64 板端。MediaBus 使用本地共享内存，不支持远程或
多设备 SDK 模式。进程必须有权访问 `/tmp/roudi` 和 MediaBus 共享内存资源。生产部署
应配置合适的服务账号、用户组或 ACL，不应仅为绕过权限问题而让整个应用长期以 root 运行。

## 构建和运行

先安装 `uniubi_robot_sdk`，确保 CMake 能找到 `UniubiRobotSdkConfig.cmake`，再在
ROS 2 工作区构建：

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

检查一路图像：

```bash
ros2 topic echo /front_camera_0/image_raw/compressed --once --field format \
  --qos-reliability best_effort
ros2 topic hz /front_camera_0/image_raw/compressed \
  --qos-reliability best_effort
```

## 何时直接使用 SDK

该包面向需要 ROS 2 图像、远程显示或录制 JPEG 的普通开发者。板端感知、raw
NV12/NV21、音频、低拷贝 GPU 流水线、精确 plane/stride 处理或完整编码元数据等
专业场景，应直接使用 C++/Python SDK 的 MediaBus API。

> ROS 2 媒体驱动面向通用开发和快速集成，并不替代完整 MediaBus SDK。音频、原始图像
> 及专业板端感知场景建议直接集成 SDK。
