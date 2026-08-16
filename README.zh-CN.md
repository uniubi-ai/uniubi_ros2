# Uniubi ROS 2

[English](README.md) | **简体中文**

Uniubi 机器人的 ROS 2 接入仓库，提供运动控制 bridge、可复用的 C++ ROS 2 客户端、
DDS / ROS 2 协议直连接口，以及板端 MediaBus 摄像头驱动。

robotServer 原始 `.msg` / `.srv` 定义统一来自
[`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs/blob/main/README.zh-CN.md)，其 ROS 2 package 名和
接口类型前缀是 `uniubi`。bridge 专用的 `MotionStatus.msg` 和
`StartMotionAction.srv` 由 `uniubi_motion_bridge` 自己维护。三种运动接入方式均不链接
`librobotMotionSdk.so`，而是通过 ROS 2 service 和 DDS topic 对接 robotServer。独立的
`uniubi_media_driver` 需要在 aarch64 板端链接 SDK，因为 MediaBus 是本地共享内存接口，
不是远程 robotServer topic。

## 从这里开始

仓库提供一个默认业务入口和两个按需使用的高级入口。业务开发直接选择 Motion bridge：

| 使用方式 | 适合谁 | 控制权管理 | 主要接口 | 推荐度 |
|---|---|---|---|---|
| Motion bridge | 普通 ROS 2 业务节点 | bridge 自动取权、续约和释放 | `/motion/*`、`/cmd_vel`、标准传感器 topic | 推荐 |
| `uniubi_motion_client` | 需要更多高级运控能力的 C++ 开发者 |应用显式调用 `connect/startControl/releaseControl` | C++ 方法和回调 | 高级 |
| DDS / ROS 2 协议直连 | 原始数据、协议维护和跨框架接入 | 只读数据不需要；控制流程自行管理 | RPC、Event、原始 topic、TRC | 协议级 |

三种方式的完整优缺点和选型说明见
[`docs/ros2_usage_modes.zh-CN.md`](docs/ros2_usage_modes.zh-CN.md)。

DDS / ROS 2 协议直连是一种完整的底层接入方式，同时包含 RPC、Event、数据 topic、控制权
生命周期和 TRC；不是“Direct DDS”和“Direct RPC”两种并列方案。协议契约见
[DDS / ROS 2 直连接入 API](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.zh-CN.md)。

> **功能范围：** Motion bridge 当前以常用运动控制为主，并选择性提供里程计、关节、IMU、
> 电池等标准 ROS 2 观测接口。它不是 High Level Client 或底层 DDS / ROS 2 直连接口的全量
> ROS 2 映射。摄像头帧明确由独立 `uniubi_media_driver` 提供，不放入 Motion bridge。

如果目标只是读取里程计、关节、IMU 或电池，使用 bridge 发布的标准 topic 即可，不需要申请运动控制权。

## 仓库组成

```text
uniubi_robot_msgs
└── uniubi                    # ros2/ 构建出的 ROS 2 msg/srv 接口包

uniubi_ros2
├── uniubi_motion_client      # 源码形式的 RPC/DDS C++ 封装，不是 SDK 动态库
├── uniubi_motion_bridge      # 面向业务节点的节点及 bridge 专用 msg/srv
└── uniubi_media_driver       # 板端 MediaBus JPEG 摄像头驱动
```

bridge 内部复用 `uniubi_motion_client`：

```text
业务 ROS 2 节点
  ├── /motion/* services
  ├── /cmd_vel
  └── 标准观测 topics
            ↓
uniubi_motion_bridge
            ↓
uniubi_motion_client
            ↓
uniubi/srv/System + DDS topics
            ↓
robotServer / MotionServer
```

## 前置条件

- ROS 2 Humble 环境已经安装并完成 `source`。
- 开发机或 Orin 与机器人位于同一可发现网络和 DDS Domain。
- 已确认目标机器人的 `device_id`，其值为设备信息中的 `deviceNo`（机器人 SN）。
  该字段用于 RPC 路由，不能隔离原始 DDS topic。
- 当前建议每条机器人使用独立的 `ROS_DOMAIN_ID`；不要让多条机器人及其 bridge 共享同一 Domain。
- 推荐使用 Cyclone DDS。

板端和开发机的软件包清单、环境加载与验证命令见
[安装 ROS 2 Humble](docs/ros2_install.zh-CN.md)。

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

如机器上有多个网卡，还需要通过 `CYCLONEDDS_URI` 明确指定机器人所在网卡。Orin
平台使用连接机器人网络的 VLAN 网卡 `eth0.100`：

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="eth0.100"/></Interfaces></General></Domain></CycloneDDS>'
```

其他平台的网卡名称可能不同，应先通过 `ip -br addr` 确认实际连接机器人网络的网卡，再将
上述配置中的 `eth0.100` 替换为实际名称。

## 构建

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

`uniubi_media_driver` 是可选的板端本地包，具有单独的 SDK 依赖。构建和运行方式见
[`src/uniubi_media_driver/README.zh-CN.md`](src/uniubi_media_driver/README.zh-CN.md)。

## 推荐方式：Motion bridge

启动 bridge：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
# Orin 使用 eth0.100；其他平台请替换为实际连接机器人网络的网卡名
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="eth0.100"/></Interfaces></General></Domain></CycloneDDS>'
export ROBOT_DEVICE_ID="$(python3 -c \
  'import json; print(json.load(open("/tmp/deviceInfo"))["deviceNo"])')"

ros2 launch uniubi_motion_bridge motion_bridge.launch.py \
  device_id:="$ROBOT_DEVICE_ID"
```

`/tmp/deviceInfo` 由机器人运行环境提供，`deviceNo` 就是 bridge 所需的目标机器人 SN。
如果设备有多个网卡，还必须按“前置条件”中的说明设置 `CYCLONEDDS_URI`，明确选择机器人
所在网卡。

bridge 启动后只连接 robotServer，不会立即申请运动控制权。第一次调用
`/motion/start_action` 时才自动取权并启动租约维护。成功的动作控制 RPC 会直接刷新租约，
只有控制调用空闲时才额外发送续约 RPC。

一个最小控制流程：

```bash
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: walking, params_json: '{\"lineVelocityX\":0.0,\"lineVelocityY\":0.0,\"velocity\":0.0}'}"

ros2 topic pub --rate 20 --times 40 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0}, angular: {z: 0.0}}'

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}'

ros2 service call /motion/stop_action std_srvs/srv/Trigger '{}'
ros2 service call /motion/release_control std_srvs/srv/Trigger '{}'
```

`/cmd_vel` 只把速度参数交给当前动作，不会取权、启动动作或切换动作。按 ROS
REP-103 和 UniUbi 运动接口的共同约定，`linear.y > 0` 表示向左横移，bridge
不做符号转换。

### 对外接口

Services：

```text
/motion/start_action
/motion/stop_action
/motion/release_control
/motion/emergency_stop
/motion/query_capabilities
```

Topics：

```text
/motion/status
/cmd_vel
/odom
/joint_states
/imu/data
/battery_state
```

接口字段、控制权生命周期、`cmd_vel`、状态更新和动作语义见
[`docs/motion_bridge.zh-CN.md`](docs/motion_bridge.zh-CN.md)。

## 其他使用方式

- 需要在自己的 C++ 节点中直接调用高级运控方法：使用
  [`uniubi_motion_client`](docs/ros2_usage_modes.zh-CN.md#方式二uniubi_motion_client-c-客户端)。
- 需要订阅原始消息、调试 QoS/类型映射或新增协议接口：使用
  [DDS / ROS 2 协议直连](docs/ros2_usage_modes.zh-CN.md#方式三dds--ros-2-协议直连)。
- 只读订阅完整传感器观测（GPS、UWB、Walk 里程计）：

```bash
ROS_DOMAIN_ID=42 \
UNIUBI_TEST_SENSOR_OBSERVED_TOPIC=/sensor/observed \
ros2 run uniubi_motion_client sensor_observed_subscriber
```

## 前置双摄像头

普通 ROS 2 开发者在机器人 aarch64 板端运行独立的 `uniubi_media_driver`。驱动直接转发
MediaBus 已有的两路 JPEG，不进行二次编码：

```text
/front_camera_0/image_raw/compressed
/front_camera_1/image_raw/compressed
```

两路均使用 `sensor_msgs/msg/CompressedImage`，`format: jpeg`。通道号只用于区分两路
前置摄像头，不表示左右位置。驱动使用 best-effort、volatile、depth-1 QoS，并仅在对应
topic 存在订阅者时启动该路码流。

板端专业感知开发应直接使用 C++/Python SDK 的 MediaBus API，以获得 raw NV12/NV21、
音频、低拷贝 GPU 处理、plane/stride 和完整编码元数据。详见
[媒体驱动说明](src/uniubi_media_driver/README.zh-CN.md)。

## 文档导航

| 文档 | 面向人群 | 内容 |
|---|---|---|
| [README](README.zh-CN.md) | 所有开发者 | 选型、安装和推荐方式快速开始 |
| [安装 ROS 2 Humble](docs/ros2_install.zh-CN.md) | 首次使用者与板端集成者 | Ubuntu 22.04 软件包、环境加载和验证 |
| [ROS 2 使用方式](docs/ros2_usage_modes.zh-CN.md) | 架构设计与高级开发者 | 三种方式的流程、优缺点和选择建议 |
| [Motion bridge 使用手册](docs/motion_bridge.zh-CN.md) | 普通业务开发者 | `/motion/*`、`/cmd_vel`、状态和观测接口 |
| [媒体驱动说明](src/uniubi_media_driver/README.zh-CN.md) | ROS 2 摄像头用户 | 两路前置摄像头 JPEG topic、参数、平台约束和 SDK 边界 |
| [运行注意事项](docs/runtime_notes.zh-CN.md) | 联调和协议开发者 | DDS、设备匹配、异步语义和安全边界 |
| [DDS / ROS 2 Direct API](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.zh-CN.md) | 协议开发者 | 原始 RPC、DDS topic 和字段契约 |
| [ROS 2 与 DDS 映射](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/ros2_dds_interop_overview.zh-CN.md) | 接口维护者 | `.msg/.srv` 与 DDS IDL 映射规则 |
| [`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs/blob/main/README.zh-CN.md) | 消息维护者 | ROS 2 msg/srv 和 DDS IDL 信源 |

文档阅读路径的进一步说明见 [`docs/README.zh-CN.md`](docs/README.zh-CN.md)。

## 安全

首次 High-level 实机联调建议先验证只读 topic，再用三个速度字段均显式为 0 的 `walking`
验证取权、动作启动和状态反馈。`standing` / `laying` 受当前姿态和服务端状态机约束，不能
作为通用的往返测试流程。带非零速度的 walking、双足、倒立和 `jump*` 等动作必须在空旷
场地、急停可用并有人值守的条件下测试。

`stop_action` 不等于释放控制权；业务结束后应显式调用 `/motion/release_control`。

`stop_action` 会停止当前动作，并异步将实际动作切回零速 `walking`，同时继续保留控制权。显式启动
三个参数均为 0 的 `walking` 也可以完成同样的动作切换。`/cmd_vel` 会修改当前动作支持的速度参数
（包括 `bipedStand`、`handstand` 等动作），但不会切换或停止动作。

## 许可证

本仓库中的 UniUbi 原创 ROS 2 集成代码、示例和文档使用 Apache License 2.0。vendored
jsoncpp 按其原始许可证授权。详见 [LICENSE](LICENSE)、[NOTICE](NOTICE) 和
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
