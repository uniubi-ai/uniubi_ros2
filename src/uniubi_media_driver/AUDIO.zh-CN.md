# ROS 2 PCM 音频

同一个 `uniubi_media_driver_node` 同时支持本地视频和音频，统一使用一个 SDK 连接。音频只连接 High-level 客户端，不申请运动控制权或切换主控。

| 部署 | SDK 库 | 能力 |
|---|---|---|
| 机器人大脑 | `lib/aarch64` | 本地 PCM 采集/RawBack 播放，可与视频同时启用 |
| x86 Linux host | `lib/x86_64` | 通过机器人 IP 远程采集/播放 |
| ARM64 Linux host | `lib/aarch64_host` | 同上；构建显式指定 `PLATFORM=aarch64_host` |

## 构建

使用本次更新的 `uniubi_robot_sdk`（需提供 `createAudioRawBack` 接口），先安装 SDK，再构建 ROS 2 包。无需修改已有 `uniubi_robot_msgs`；新音频消息属于 `uniubi_media_driver`。

```bash
source /opt/ros/humble/setup.bash
# x86 主机；大脑本机也使用此命令。在普通 ARM64 host 上另加 -DPLATFORM=aarch64_host
cmake -S ~/uniubi_robot_sdk -B /tmp/sdk-install-build -DBUILD_SDK_CPP_EXAMPLES=OFF
cmake --install /tmp/sdk-install-build --prefix "$HOME/uniubi_sdk_install"
cd ~/ros2_ws
colcon build --packages-select uniubi_media_driver --cmake-args \
  -DCMAKE_PREFIX_PATH="$HOME/uniubi_sdk_install"
# 普通 ARM64 host 的 colcon 命令还需加入 -DPLATFORM=aarch64_host
source install/setup.bash
export LD_LIBRARY_PATH="$HOME/uniubi_sdk_install/lib/x86_64:${LD_LIBRARY_PATH:-}"
# 大脑改为 lib/aarch64，普通 ARM64 host 改为 lib/aarch64_host。
```

ROS 2 使用其自身的 DDS/RMW 依赖；SDK 的运行库必须完整、同版本、匹配平台。改变平台需使用新的构建目录。

## 启动

大脑本地模式，保留已有本地 SDK 配置与共享内存访问权限：

```bash
ros2 launch uniubi_media_driver audio_driver.launch.py
```

外部 x86/ARM64 host，替换设备编号、机器人地址和真实网卡名：

```bash
ros2 launch uniubi_media_driver audio_driver.launch.py \
  host:=192.168.43.23 device_id:=YOUR_DEVICE_ID network_interface:=eth0
```

默认启动通道0采集与音量20的播放输入，`enable_video=false`。如果只需要采集，可复制 `config/audio_driver.yaml`，设置 `audio_playback: false`，通过 `config_file:=/absolute/path/audio.yaml` 加载。默认的视频 launch 仍只启用视频；本地视频与音频共用一个节点时，在同一 YAML 中显式开启对应开关。远端不支持视频或布局查询，不能设置 `enable_video=true`。

## ROS 接口

| 接口 | 类型 | 用法 |
|---|---|---|
| `audio/capture` | `uniubi_media_driver/msg/AudioFrame` | 音频帧发布；best effort、volatile、深度5 |
| `audio/playback` | 同上 | PCM 播放输入；best effort、volatile、深度2 |
| `audio/reset` | `std_srvs/srv/Trigger` | 清空驱动待发送队列及 SDK 播放缓存，保留流 |
| `audio_volume` | 整数参数 0–100 | 动态设置 RawBack 流音量，不修改硬件总音量 |

话题/服务名称为相对名称，支持 ROS namespace 和 remap。`audio_capture`、`audio_playback`、`audio_channel`、`audio_frame_id`、`host`、`device_id`、`network_interface` 和 `enable_video` 需要重启节点才能修改。

消息包含 ROS 接收/发布时间、设备微秒时间戳、序号、采样率、位宽、声道数、采集通道与字节数据。设备时间戳不应直接作为 ROS 时间；不同设备/后端的时钟起点可能不同。

播放必须为 **16000 Hz / signed 16-bit little-endian / mono**，每条消息1280字节，40ms一帧。节点不进行解码或重采样。发布者应每40ms发送一帧；节点再按40ms节奏发送，内部最多待发2帧，满队列、格式错误、未就绪或发送失败时丢帧并记录日志，不重放积压数据。播放使用本机单调时钟生成SDK发送时间戳，不采用输入的设备时间戳/通道字段。

## 示例

```bash
# 只统计，不保存麦克风内容；可增加 --output capture.pcm 保存原始 PCM
ros2 run uniubi_media_driver capture_pcm.py --seconds 5
# 文件必须已经是 16kHz/s16le/mono；末帧自动补零，并追加两帧静音
ros2 run uniubi_media_driver play_pcm.py input.pcm
ros2 param set /uniubi_media_driver audio_volume 20
# 先停止播放发布者，再清空缓存；否则之后到达的新 ROS 消息会继续播放
ros2 service call /audio/reset std_srvs/srv/Trigger '{}'
```

提交成功不等于扬声器已经播放完成；需现场确认物理输出。采集订阅注册成功也不代表已收到帧，使用统计示例核验。关闭时先停采集回调和RawBack，再关闭媒体与SDK连接。
