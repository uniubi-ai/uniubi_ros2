# RTSP 视频后端

RTSP 后端位于同一个 `uniubi_media_driver` ROS 2 包内，使用独立可执行程序，不依赖 UniUbi SDK。它通过 FFmpeg 的 TCP RTSP 连接独立解码各路相机，向相对话题 `<camera_name>/image_raw` 发布 BGR8 格式的 `sensor_msgs/msg/Image`。设置 `publish_compressed: true` 后，还会在 `<camera_name>/image_raw/compressed` 发布 JPEG `sensor_msgs/msg/CompressedImage`，与现有 MediaBus 相机话题一致。默认相机名和光学坐标系 ID 与 MediaBus 一致；可组合 ROS 命名空间与 `topic_prefix`。

## 适用场景与性能代价

本后端提供通用 ROS 2 图像接口，适合快速集成、功能验证、ROS 可视化和需要图像话题的录包。它是可选的接入方式，开发者也可以直接使用设备 RTSP 地址，不经过本节点。

| 场景 | 建议接入方式 |
|---|---|
| 希望快速获得标准 ROS 图像话题 | 使用本后端，根据需要开启 JPEG |
| 仅播放或转存视频流 | 直接使用 RTSP 客户端；仅转存且工具/封装格式支持时，可保存原编码流以避免解码、重编码 |
| 低延迟、高帧率、多路视频或 GPU 感知流水线 | 直接接入 RTSP，按目标平台配置解码、缓冲和后续处理 |
| 大脑本机需要 SDK 原始图像或 GPU 数据访问 | 参考 MediaBus SDK，避免不必要的 CPU 图像转换 |

当前实现采用 FFmpeg **CPU 软件解码**，将图像转换成 CPU 内存中的 BGR8，再发布 ROS 消息；没有配置硬件解码设备或 GPU 零拷贝路径。设备有硬解单元，不代表本节点会自动使用它。解码、颜色转换、内存拷贝以及 ROS 消息传输都会带来开销。按双路 1280×720、25 fps 计算，BGR8 原始图像数据量约为 **138 MB/s**，这不是输入 RTSP 的编码码率，也不表示实际网络发送量或完整内存带宽开销。

`publish_compressed: true` 会把解码后的图像再次编码为 JPEG，增加 CPU 和延迟；默认关闭。本机 MediaBus 路径直接转发已有 JPEG，不经过这套 RTSP 解码/重编码流程。默认编译两个后端只增加构建依赖和产物，未启动的后端不会产生视频处理开销。

对性能敏感的开发者，建议使用 FFmpeg、GStreamer 或目标平台提供的媒体接口直接接入 RTSP。具备硬解能力时，应确认源编码、分辨率、驱动和工具构建均受支持，并显式配置对应的硬件解码路径。GPU 感知可尽量让解码输出保留在设备内存中，避免下载到 CPU 后又上传 GPU。若最终仍需发布 CPU BGR8 ROS 图像，硬解之后的转换、拷贝和消息传输代价仍然存在，硬解不保证端到端一定更快。

应在目标设备上测量实际接收帧率、端到端延迟、CPU/GPU 占用和丢帧情况，再决定使用通用封装还是定制流水线。本后端不承诺达到源流标称帧率，也不承诺零拷贝或硬件加速。

默认同时编译 MediaBus 和 RTSP，运行时选择视频后端，不会同时启动。先准备 ROS 2 Humble 开发环境，并按[媒体驱动构建说明](README.zh-CN.md#构建和运行)安装机器人 SDK、配置 SDK 路径，再安装 FFmpeg 开发库（Ubuntu 22.04，x86_64 和 ARM64 使用相同包名）：

```bash
sudo apt install libavformat-dev libavcodec-dev \
  libavutil-dev libswscale-dev pkg-config
```

依赖准备完成后，在 `uniubi_ros2` 工作区直接构建，无需选择构建开关：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select uniubi_media_driver
source install/setup.bash
ros2 launch uniubi_media_driver media_driver.launch.py video_backend:=rtsp host:=192.168.1.10
```

上述 `host` 参数生成两路 URL：

```text
rtsp://192.168.1.10:554/live?channel=1&stream=0
rtsp://192.168.1.10:554/live?channel=2&stream=0
```

若实际端点不同，省略 `host`，在 `config/rtsp_driver.yaml` 的副本里填写 `rtsp_urls`，并指定 `rtsp_config_file:=/绝对路径/配置.yaml`。`rtsp_urls`、`camera_names`、`frame_ids` 长度须相同。URL 可能含凭据，请保护配置文件；驱动不会记录 URL 或 FFmpeg 诊断文本。`open_timeout_ms`、`read_timeout_ms` 和 `reconnect_delay_ms` 控制连接超时及重试。运行参数在启动后只读。

每路解码线程仅覆盖一帧待发布图像，独立的发布线程负责 ROS 发布和可选 JPEG 编码，因此发布慢不会让应用层帧队列增长。ROS 使用深度为 1 的 best-effort SensorDataQoS。应用层队列只保留一帧，但网络缓冲、编解码重排及订阅端队列仍可能引入延迟。时间戳取解码时 ROS 节点时钟，不代表相机采集时间。连接失败的相机独立重试；关闭节点时，中断回调会终止阻塞中的 RTSP 操作。

默认构建包含 MediaBus 视频、PCM 音频和 RTSP 视频。启动时选择 `video_backend:=mediabus`（默认）或 `video_backend:=rtsp`；PCM 音频沿用 `audio_driver.launch.py`。编译两套后端不会自动启动两套节点。

## 有线与 Wi-Fi

`host` 填写提供 RTSP 服务的设备接口地址，有线或 Wi-Fi 都可以，出接口由主机路由决定。
有线直连时应使用小脑的有线 IP；SSH 22 端口进入大脑后看到的地址不一定是 RTSP 服务地址。
可先用 `ip route get <设备有线IP>` 确认走有线网卡，再用 FFprobe 检查 554 端口的实际视频流。
节点不会修改路由或自动添加转发规则。

## 高级：精简构建与旧缓存

仅在不安装机器人 SDK、只需要 RTSP 视频时，才关闭 SDK 后端。使用独立构建和安装目录，避免混用旧产物：

```bash
colcon build --packages-select uniubi_media_driver \
  --build-base build-rtsp --install-base install-rtsp --cmake-args \
  -DUNIUBI_MEDIA_WITH_SDK=OFF
source install-rtsp/setup.bash
```

普通用户无需设置这些开关；`UNIUBI_MEDIA_WITH_SDK` 和 `UNIUBI_MEDIA_WITH_RTSP` 默认均为 `ON`。
如果旧构建目录曾缓存 `OFF`，更新代码后可执行一次
`colcon build --packages-select uniubi_media_driver --cmake-clean-cache` 恢复默认全量构建。

## 自测

参数测试通过 `colcon test --packages-select uniubi_media_driver` 执行。
`test/rtsp_integration.py` 是可选的回环网络集成测试，需安装 `python3-gi`、
`gir1.2-gst-rtsp-server-1.0`、GStreamer base/good/ugly 插件和 `python3-opencv`。
它使用合成 MJPEG/H.264 流，检查双路原图/JPEG、断流重连、单路连接卡死与及时退出；不连接机器人。

```bash
python3 src/uniubi_media_driver/test/rtsp_integration.py \
  --node "$PWD/install/uniubi_media_driver/lib/uniubi_media_driver/uniubi_rtsp_driver_node" \
  --evidence /tmp/uniubi-rtsp-integration
```
