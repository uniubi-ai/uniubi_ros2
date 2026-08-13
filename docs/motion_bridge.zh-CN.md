# Motion bridge 使用手册

[English](motion_bridge.md) | **简体中文**

`uniubi_motion_bridge` 面向普通 ROS 2 业务开发者。业务节点只使用公开 topic/service，不直接
调用 `uniubi/srv/System`，也不需要链接 SDK 动态库。

> **功能范围：** bridge 当前以常用运动控制为主，并选择性提供里程计、关节、IMU、电池等
> 标准 ROS 2 观测接口。它不是 High Level Client 或底层 DDS / ROS 2 直连接口的全量功能移植。
> 是否支持某项能力，以本文列出的 topic、service 和参数为准；未列出的系统、音频、媒体、
> 原始字段或新增协议能力可能尚未移植。

## 启动

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

export ROBOT_DEVICE_ID="$(python3 -c \
  'import json; print(json.load(open("/tmp/deviceInfo"))["deviceNo"])')"

ros2 launch uniubi_motion_bridge motion_bridge.launch.py \
  device_id:="$ROBOT_DEVICE_ID"
```

`device_id` 必须填写，其值是 `/tmp/deviceInfo` 中的 `deviceNo`（机器人 SN），但它只用于
RPC 路由。`/motion/observed`、`/sensor/observed` 和
`/robotServer/Event` 等原始 topic 当前没有可供 bridge 过滤的设备身份；多条机器人共享同一
DDS Domain 时可能混入其他机器人的观测或事件。当前应为每条机器人使用独立的
`ROS_DOMAIN_ID`。bridge 启动后只建立连接，不会立即申请高级运动控制权。

bridge 不会把 DDS service 已发现直接视为连接就绪。SDK `connect()` 成功后，bridge 会在
5 秒总超时内用只读、无副作用的 `getMotionCapabilities` 检查双向 RPC 链路，单次 RPC
最长等待 500 ms，失败后间隔 200 ms 重试。首次收到成功响应后，才启用运动观测和状态查询，
并在 `/motion/status` 中报告 `CONNECTED`。如果本次就绪检查超时，bridge 不主动断开 SDK
连接；后续 service 请求会重新执行一轮有限时的就绪检查。

如果设备有多个网卡，还必须设置 `CYCLONEDDS_URI`，明确选择机器人所在网卡。

## Services

| 名称 | 类型 | 行为 |
|---|---|---|
| `/motion/start_action` | `uniubi_motion_bridge/srv/StartMotionAction` | 必要时自动取权，然后启动指定动作 |
| `/motion/stop_action` | `std_srvs/srv/Trigger` | 发送 `stopAction`，继续持权和续约 |
| `/motion/release_control` | `std_srvs/srv/Trigger` | 先停止动作，再释放控制权；重复调用幂等 |
| `/motion/emergency_stop` | `std_srvs/srv/Trigger` | 发送高级运控急停；必须已经持权 |
| `/motion/query_capabilities` | `std_srvs/srv/Trigger` | 返回当前机型的动作和参数能力 JSON |

没有公开的 `take_control` service。取权由 `/motion/start_action` 在需要时自动完成。

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
  "{action: walking, params_json: '{}'}"
```

bridge 不为 standing、laying、walking 等动作分别创建 service。动作名和参数范围由服务端能力
列表决定并由服务端校验。

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
- 内部 `/robotServer/Event` 在控制权被抢占等事件发生时立即更新控制状态和错误。

原始 Event JSON 不对外发布；未知事件只写 DEBUG 日志。10 Hz 是状态快照，最多存在一个查询
周期的显示延迟，不保证记录持续时间短于 100 ms 的每个中间动作。

## `stop_action` 的语义

`stop_action` 不等于“切换到 standing”，也不等于释放控制权。

对 walking 调用 `stop_action` 时，服务端通常保留 walking 动作并把速度清零。许多一次性动作
结束后也会回到 walking。当前实际动作应以 `/motion/status.current_action` 为准。

如果业务明确要求站立，应显式调用：

```bash
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: standing, params_json: '{}'}"
```

## 观测接口

`/joint_states`、`/imu/data` 和 `/battery_state` 来自原始 `/motion/observed`，不要求运动控制权。

- `/joint_states` 优先使用服务端电机布局；当前固件不支持布局 RPC 时可通过配置提供机型 fallback。
- `JointState.effort` 使用设备电机 torque。
- IMU 加速度或角速度无效时不发布；四元数无效时将 `orientation_covariance[0]` 设为 `-1`。
- `BatteryState.percentage` 把设备 0-100 电量换算为 ROS 2 的 0-1。
- `/odom` 使用设备端已累计的 position/yaw，上层不能再次积分，当前不发布 TF。
  里程计的 `position.y` 和 `twist.linear.y` 同样使用“正左负右”，bridge 原样发布。

完整原始错误码、在线状态和温度仍以 `/motion/observed` 为准，里程计生命周期字段以
`/sensor/observed` 中的 `odom` 为准。

## 主要参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `device_id` | 空 | 目标机器人 `deviceNo` / SN；必须填写 |
| `lease_ms` | `60000` | 申请控制权时请求的租约 |
| `auto_connect` | `true` | 启动后是否自动连接 robotServer |
| `cmd_vel_timeout_ms` | `500` | ROS 2 速度输入超时 |
| `cmd_vel_rate_hz` | `30.0` | 最大参数 RPC 下发频率；上游可更高频发布，bridge 仅转发最新值 |
| `motion_status_rate_hz` | `10.0` | 实际动作状态查询频率 |
| `battery_publish_rate_hz` | `1.0` | 电池状态发布频率 |

其余 topic、frame 和电机布局参数见
`src/uniubi_motion_bridge/config/motion_bridge.yaml`。
