# Motion bridge 使用手册

[English](motion_bridge.md) | **简体中文**

`uniubi_motion_bridge` 面向普通 ROS 2 业务开发者。业务节点只使用公开 topic/service，不直接
调用 `uniubi/srv/System`，也不需要链接 SDK 动态库。

> **功能范围：** bridge 当前以常用运动控制为主，并选择性提供里程计、关节、IMU、电池等
> 标准 ROS 2 观测接口。它不是 High Level Client 或底层 DDS / ROS 2 直连接口的全量功能移植。
> 是否支持某项能力，以本文列出的 topic、service 和参数为准；未列出的系统、音频、媒体、
> 原始字段或新增协议能力可能尚未移植。

## 启动

先根据运行位置选择一组配置，不要只修改 Domain 而继续使用另一侧的 RPC service。

### 机器人“大脑”Orin 本机

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="eth0.100"/></Interfaces></General></Domain></CycloneDDS>'

export ROBOT_DEVICE_ID="$(python3 -c \
  'import json; print(json.load(open("/tmp/deviceInfo"))["deviceNo"])')"

ros2 run uniubi_motion_bridge uniubi_motion_bridge_node --ros-args \
  -p sensor_observed_source:=cere_motion_state \
  -p robot_service_name:=cerebellumServer \
  -p event_topic:=/robotCereServer/Event \
  -p device_id:="$ROBOT_DEVICE_ID"
```

### 远程 PC/开发主机

在远程 PC 启动 bridge 前，机器人必须已连接 Wi-Fi，并确认 PC 能通过当前网络到达机器人。
机器人未联网时，仅配置 PC 端的 Domain、网卡和 `device_id` 仍无法建立通信。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="REPLACE_WITH_ROBOT_NIC"/></Interfaces></General></Domain></CycloneDDS>'
export ROBOT_DEVICE_ID='<deviceNo>'

ros2 launch uniubi_motion_bridge motion_bridge.launch.py \
  device_id:="$ROBOT_DEVICE_ID"
```

可选的 ROS namespace 用于隔离 bridge 对外的 topic 和 service。默认空的
`namespace` 与 `frame_prefix` 保持原有根路径和 frame ID。例如两个 Host bridge 在同一个 DDS Domain 中使用不同 SN：

```bash
ros2 launch uniubi_motion_bridge motion_bridge.launch.py \
  namespace:=dog57 frame_prefix:=dog57 device_id:="$ROBOT_DEVICE_ID"
```

对外接口包括 `/dog57/cmd_vel`、`/dog57/motion/status`、
`/dog57/motion/start_action`、`/dog57/audio/*`、`/dog57/light/*`、
`/dog57/gps/observed` 和 `/dog57/uwb/observed`。里程计 frame 为
`dog57/odom`、`dog57/base_link`，IMU frame 为 `dog57/imu_link`。
`frame_prefix` 为配置的 `odom_frame_id`、`base_frame_id` 和 `imu_frame_id`
加上 `prefix/`；这些参数本身仍是原值。其他发布者的 frame ID 应与之匹配。
以 `/` 开头的自定义 topic 参数仍是绝对路径，不受 namespace 影响。
随包 YAML 使用相对默认路径和通配节点选择器，可供根节点与带 namespace 的节点加载。

bridge 直接订阅 Host 侧 ROS topic `/robot/<SN>/sensor/observed` 与
`/robot/<SN>/motion/observed`，其原生 DDS 名分别为 `rt/robot/<SN>/sensor/observed` 与
`rt/robot/<SN>/motion/observed`。共享的
`/robotServer/Event` 载体仅接收逻辑事件名 `<SN>.robotServer.control.status` 与
`<SN>.robotServer.host.event`。`device_id` 还用于 RPC 路由；发现和 RPC service
名保持不变。板内 `cere_motion_state` 来源继续使用原有 `rt/cere/motionState` 和事件路径；板内流量不属于本次 Host 侧隔离范围。
namespace 只影响 bridge 对外 ROS API。此 Host 协议要求机器人端配套更新；
旧的共享观测 topic 不作为降级路径。

`device_id` 在两种模式下都必须填写，其值是目标机器人的 `deviceNo`（机器人 SN）。
它选择该设备专属的观测订阅、过滤共享 EventBus 载体中的逻辑事件，并用于 RPC 路由。
这些隔离行为需要配套更新的机器人端；其他 DDS 参与者是否可共享 Domain 仍需单独评估。
bridge 启动后只建立连接，不会立即申请高级运动控制权。

bridge 不会把 DDS service 已发现直接视为连接就绪。SDK `connect()` 成功后，bridge 会在
5 秒总超时内用只读、无副作用的 `getMotionCapabilities` 检查双向 RPC 链路，单次 RPC
最长等待 500 ms，失败后间隔 200 ms 重试。首次收到成功响应后，才启用运动观测和状态查询，
并在 `/motion/status` 中报告 `CONNECTED`。当前实现会在开启 RPC 完成后才创建原始观测订阅，
因此首批观测帧可能丢失；这是当前实现细节，DDS 协议直连仍应使用先订阅再开推送的顺序。
如果本次就绪检查超时，bridge 不主动断开 SDK 连接；后续 service 请求会重新执行一轮有限时的就绪检查。

如果设备有多个网卡，还必须设置 `CYCLONEDDS_URI`，明确选择机器人所在网卡。

**外部主机网线直连机器人网口时**，在 `CYCLONEDDS_URI` 里加上 `<DontRoute>true</DontRoute>`：机器人 Wi-Fi 同时开启时必须加（否则 DDS 可能选中主机不可达的 Wi-Fi 地址），未开启时可加可不加：

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="eth0" priority="3" multicast="default" presence_required="false"/></Interfaces><AllowMulticast>true</AllowMulticast><DontRoute>true</DontRoute></General></Domain></CycloneDDS>'
```

`DontRoute` 只适用于与机器人同一二层/同网段（网线直连或交换机）；网络配置见[连接外设](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/how-to/connect-peripherals.zh-CN.md)。

## Services

| 名称 | 类型 | 行为 |
|---|---|---|
| `/motion/start_action` | `uniubi_motion_bridge/srv/StartMotionAction` | 必要时自动取权，然后启动指定动作 |
| `/motion/stop_action` | `std_srvs/srv/Trigger` | 发送 `stopAction`，继续持权和续约 |
| `/motion/release_control` | `std_srvs/srv/Trigger` | 先停止动作，再释放控制权；重复调用幂等 |
| `/motion/emergency_stop` | `std_srvs/srv/Trigger` | 发送高级运控急停；必须已经持权 |
| `/motion/query_capabilities` | `std_srvs/srv/Trigger` | 返回当前机型的动作和参数能力 JSON |

没有公开的 `take_control` service。取权由 `/motion/start_action` 在需要时自动完成。首次取权会先
请求将电机运控 master 切回内置小脑，等待切换稳定后再申请 High-level RPC 会话；任一步失败时
动作都不会下发，service 会返回失败。

Service 返回成功只代表服务端接受请求，不代表机械动作已经完成。

## Topics

| 名称 | 类型 | QoS/频率 | 说明 |
|---|---|---|---|
| `/motion/status` | `uniubi_motion_bridge/msg/MotionStatus` | Reliable、Transient Local，默认 10 Hz | 实际动作、速度、控制状态和最近错误 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Best Effort、Keep Last 1，默认最多 30 Hz 转发 | 当前动作参数输入 |
| `/odom` | `nav_msgs/msg/Odometry` | Best Effort、Keep Last 1 | 有效 Walk 累计里程计 |
| `/joint_states` | `sensor_msgs/msg/JointState` | Best Effort、Keep Last 1 | 关节角、速度和力矩 |
| `/imu/data` | `sensor_msgs/msg/Imu` | Sensor Data QoS | 姿态、角速度和线加速度 |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Sensor Data QoS，默认 1 Hz | 电压、电流、温度和电量 |

## 标准控制流程

```text
启动 bridge
→ connected
→ /motion/start_action
→ 自动 takeMotionControl + 自动维护租约
→ 使用 /cmd_vel 或继续 start_action
→ /motion/stop_action（可选）
→ /motion/release_control
→ connected
```

如果 `start_action` 为本次调用新取得控制权但动作启动失败，bridge 会尝试回滚释放该会话。
正常退出时 bridge 也会尽力停止动作并释放控制权；异常退出由服务端租约超时兜底。

## 启动动作

先查询当前机型能力：

```bash
ros2 service call /motion/query_capabilities std_srvs/srv/Trigger '{}'
```

再调用统一动作接口：

```bash
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: walking, params_json: '{\"lineVelocityX\":0.0,\"lineVelocityY\":0.0,\"velocity\":0.0}'}"
```

bridge 不为 standing、laying、walking 等动作分别创建 service。动作名和参数范围由服务端能力
列表决定并由服务端校验。取控后 bridge 会直接转发请求动作，不会代替调用方插入推荐的全零
`walking` 前置切换，也不会为调用方轮询 `current_action`。

首次 High-level 实机动作验证建议使用上面的全零 `walking` 请求，再通过
`/motion/status` 确认 `current_action` 和三个速度字段。不要用空 JSON 隐式依赖默认零值。
`standing` 不能从 `laying`（趴下）状态直接触发。实际可用动作及状态切换关系仍以
`/motion/query_capabilities` 返回结果为准。

## `/cmd_vel`

字段映射：

| ROS 2 字段 | 高级动作参数 |
|---|---|
| `linear.x` | `lineVelocityX` |
| `linear.y` | `lineVelocityY` |
| `angular.z` | `velocity` |

`linear.y` 和 UniUbi `lineVelocityY` 都遵循“正左负右”定义，bridge 不做
符号转换。`/motion/status.line_velocity_y` 也使用相同方向定义。

bridge 不根据 `/motion/status` 判断当前动作，也不在 bridge 内按动作范围限幅。它把三个有限数值
直接交给 `setActionParams`，鉴权、动作参数匹配和安全限制由 client/服务端处理。因此
`/cmd_vel`：

- 不申请控制权。
- 不启动或切换动作。
- 不逐条返回响应；最近失败反映在 `/motion/status`。
- 高频消息只保留最新值，并按 `cmd_vel_rate_hz` 限制 RPC 下发频率。

超过 `cmd_vel_timeout_ms` 未收到新消息时，bridge 发送一次三个速度字段均为零的参数，不调用
`stopAction`，也不释放控制权。服务端还有独立的控制帧超时保护。

## `/motion/status`

消息包含：

```text
stamp
control_state
current_action
line_velocity_x
line_velocity_y
angular_velocity
last_error_code
last_error_message
```

状态有两个更新来源：

- bridge 以 `motion_status_rate_hz`（默认 10 Hz）调用只读 `queryMotionState`，更新实际动作和速度。
- 内部 Event 在控制权被抢占等事件发生时立即更新控制状态和错误。

原始 Event JSON 不对外发布；未知事件只写 DEBUG 日志。10 Hz 是状态快照，最多存在一个查询
周期的显示延迟，不保证记录持续时间短于 100 ms 的每个中间动作。

## `stop_action` 的语义

`stop_action` 不等于“切换到 standing”，也不等于释放控制权。

对当前 capabilities 暴露的所有动作，调用 `stop_action` 都会执行动作收尾，并将实际动作切回
`walking`，同时把 walking 三轴速度清零。控制权仍会保留并续约。这个切换是异步的，当前实际动作
和速度应以 `/motion/status.current_action` 及其速度字段为准。

也可以通过显式切换到全零参数的 `walking` 来结束当前动作并进入零速 walking：

```bash
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: walking, params_json: '{\"lineVelocityX\":0.0,\"lineVelocityY\":0.0,\"velocity\":0.0}'}"
```

`/cmd_vel` 会修改当前动作所暴露的速度参数；如果当前动作支持这些字段，`walking`、`bipedStand`、
`handstand` 等动作都可以接收速度输入。它不会切换动作，也不能单独停止动作。

如果业务明确要求站立，应显式调用：

```bash
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: standing, params_json: '{}'}"
```

## 观测接口

Host 模式的 `/joint_states`、`/imu/data` 和 `/battery_state` 来自 `/robot/<SN>/motion/observed`。板内 `cere_motion_state` 模式的关节与 IMU 来自 `rt/cere/motionState`；该消息没有电池字段，因此电池约每秒通过异步 `getSystemStatus` 读取。两种模式均不要求运动控制权。

- `JointState.effort` 使用设备电机 torque。
- IMU 加速度或角速度无效时不发布；四元数无效时将 `orientation_covariance[0]` 设为 `-1`。
- `BatteryState.percentage` 把设备 0-100 电量换算为 ROS 2 的 0-1。
- `/odom` 使用设备端已累计的 position/yaw，上层不能再次积分，当前不发布 TF。
  里程计的 `position.y` 和 `twist.linear.y` 同样使用“正左负右”，bridge 原样发布。

完整原始错误码、在线状态和温度仍以 `/robot/<SN>/motion/observed` 为准，里程计生命周期字段以
`/robot/<SN>/sensor/observed` 中的 `odom` 为准。

## 主要参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `namespace`（launch） | 空 | bridge 对外接口的 ROS 命名空间 |
| `frame_prefix` | 空 | odom、base、IMU frame ID 前缀 |
| `device_id` | 空 | 目标机器人 `deviceNo` / SN；必须填写 |
| `lease_ms` | `60000` | 申请控制权时请求的租约 |
| `auto_connect` | `true` | 启动后是否自动连接所配置的 RPC service |
| `cmd_vel_timeout_ms` | `500` | ROS 2 速度输入超时 |
| `cmd_vel_rate_hz` | `30.0` | 最大参数 RPC 下发频率；上游可更高频发布，bridge 仅转发最新值 |
| `motion_status_rate_hz` | `10.0` | 实际动作状态查询频率 |
| `battery_publish_rate_hz` | `1.0` | 电池状态发布频率 |

其余 topic、frame 和电机布局参数见
`src/uniubi_motion_bridge/config/motion_bridge.yaml`。

### 大脑传感器观测来源

Domain 1 使用 `sensor_observed_source:=cere_motion_state`，客户端原生 DDS 订阅
`rt/cere/motionState`（可用 `cere_motion_topic` 配置），复用设备现有
`uniubi::dds_::BrainMotionState`。它不是普通 ROS2 消息类型，不能直接用同名 `.msg` 替代。
仅 `hasSensor != 0` 的帧进入传感器回调，`hasMotion != 0` 独立进入电机/IMU 回调；本地订阅不启用 Host Domain 的观测发布。内部毫秒时间戳转换为微秒，电机数量越界或时间戳溢出的帧被丢弃。GPS/UWB/里程计有效标志保持原值。无效定位仍会发布，不能当成定位有效。

Host 默认 `sensor_observed_source:=sensor_observed`，订阅 ROS topic `/robot/<SN>/sensor/observed`（原生 DDS 名 `rt/robot/<SN>/sensor/observed`）。
来源选择与 Domain 数字独立；只设置 `ROS_DOMAIN_ID=1` 不会自动切换来源。
原生接收器使用同一 ROS context 的 Domain 和 `CYCLONEDDS_URI` 网络配置，限机器人本地
大小脑链路使用。`device_id` 用于 RPC 寻址，内部观测消息本身不带设备 ID，不提供跨设备筛选。

构建需要 `ros-humble-cyclonedds`（含 idlc 和开发文件），并需更新 `uniubi_robot_msgs`。
复制消息仓库时保留 `ros2/` 与 `idl/` 的相邻目录结构，不再只复制 `ros2/`。
公开 IDL 原样同步自主仓，不需要新增设备话题、更新 SDK 动态库或修改设备协议。

### 后台运动状态查询

后台 `queryMotionState` 使用异步 RPC，超时固定为 **1000 ms**，不提供超时配置参数。
同时最多保留一个本地待完成请求，完成或超时后等待现有查询周期再发起下一次；
慢查询不会阻塞命令队列或看门狗。超时请求会从客户端清理，迟到响应不再更新状态。
断开、退出及相关动作状态重置时会取消旧查询。其他同步 RPC 执行期间已在期限内处理的
响应不会因稍后才消费结果而被误报超时。显式 `/motion/query_state` 仍使用原来的 5 秒等待。
