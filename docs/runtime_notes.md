# 运行注意事项

本文记录使用 `uniubi_motion_client` 和 `uniubi_motion_bridge` 二次开发时容易踩坑的 ROS 2 集成行为。完整 DDS / ROS 2 协议说明统一维护在 [uniubi-docs](https://github.com/uniubi-ai/uniubi-docs)。

## DDS / ROS 2 协议直连

协议直连同时包含 RPC、Event、原始数据 topic 和 TRC。以下小节按通道说明运行边界，不代表
Direct DDS 和 Direct RPC 是两种平级接入方式。

### 原始数据 topic

原始持续数据通过 DDS/ROS 2 topic 提供：

- Motion observation topic：`/motion/observed`
- Sensor observation topic：`/sensor/observed`（GPS、UWB、Walk 里程计）
- 原始控制 topic：`/motion/trc`（不是普通只读数据流）

订阅这些观测 topic 不要求持有 High Level 控制权。`/motion/observed` 和
`/sensor/observed` 默认关闭，协议直连使用者必须先建立 reader，
再通过无需运动控制权的 RPC 调用 `setMotionObservedEnable()` 开启。Motion bridge 会自动管理
原始观测流，其业务节点只需订阅 bridge 发布的标准 ROS 2 topic。原始 topic 测试通过只说明消息类型、
DDS 发现和 QoS 链路可用。

`/sensor/observed` 使用 `BEST_EFFORT` / `KEEP_LAST depth=1` / `VOLATILE`，由
`setMotionObservedEnable(..., sensor_enable=true)` 开启。里程计从 `SensorObserved.odom`
读取，仅在 Walk 模式有效；退出 Walk 时保留当前区间末值并置 `valid=false`，再次进入 Walk 时
建立新原点并递增 `epoch`。

### RPC、Event 和控制

本仓不链接 `librobotMotionSdk.so`。协议直连通过 `uniubi/srv/System` 对接 robotServer：

- RPC service：`uniubi/srv/System`
- 异步控制事件：`/robotServer/Event`

只读查询不需要控制权。直接执行控制 RPC 时，调用方必须自行处理取权、续约、Event 和释放。
RPC 测试通过只说明请求/响应契约和路由可用，不能代表 C++ 或 Python SDK runtime 链路可用。

## Motion bridge 速度安全边界

- `start_action` 在需要时自动申请 SDK 高级运控控制权；不对外提供独立 `take_control` 服务。
- `/cmd_vel` 不申请控制权、不查询、启动或切换动作，只把三个速度字段直接交给 `setActionParams`。
- `linear.y` 和 UniUbi `lineVelocityY` 都使用“正左负右”定义，bridge 不做符号转换；
  `/motion/status.line_velocity_y` 也使用相同方向定义。
- bridge 只读取 `linear.x`、`linear.y` 和 `angular.z`，不做动作判断或速度限幅；SDK 与服务端负责校验和安全限制。
- 接收超时只发送一次三个速度字段均为零的参数，不停止动作、不释放控制权。
- 高频输入会合并为最新值，并按 `cmd_vel_rate_hz`（默认 30 Hz）限频。
- 成功的 `startAction`、`setActionParams`、`stopAction` 和 `emergencyStop` 会刷新服务端控制租约；
  client 仅在这些控制 RPC 空闲达到续约周期后发送 `renewMotionControl`。失败或超时的 RPC 不计为续约。
- `stop_action` 只停止动作并继续持权续约；`release_control` 与进程退出会先停止动作再释放控制会话。

`/odom` 仅转发 `valid=true` 的设备累计里程计，不再次积分，也暂不发布 TF。
`position.y` 和 `twist.linear.y` 均为正左负右，bridge 不做符号转换。原始生命周期字段仍以
`/sensor/observed` 中的 `odom` 为准。

## Motion bridge 状态观测

- `/motion/status` 通过 10 Hz `queryMotionState` RPC 发布实际动作、速度、控制状态和最近错误；它不参与 `/cmd_vel` 下发。
- `/motion/status` 最多有一个查询周期的显示延迟，不能当作逐控制帧反馈。
- 内部 `/robotServer/Event` 仍用于立即发现控制权抢占；bridge 将已知事件转换成结构化状态，未知原始 JSON 只写 DEBUG 日志，不再公开 `~/events`。
- `/joint_states`、`/imu/data` 和 `/battery_state` 都由 `/motion/observed` 转换，不需要 High Level 控制权。
- bridge 通过只读 `getMotorLayout` RPC 获取关节名称和 `(limbNo, jointNo)`，不依赖固定电机数组顺序。
- `/joint_states` 的 position/velocity/effort 分别来自电机 position/velocity/torque；故障码、在线状态和温度仍以原始 `/motion/observed` 为准。
- `/imu/data` 仅在 accel/gyro 有效时发布；四元数无效时按 `sensor_msgs/Imu` 约定标记 orientation 不可用。
- `/battery_state.percentage` 范围为 0-1；设备 power 原始字段范围为 0-100。未提供的容量字段用 NaN 表示。

## DDS 与设备匹配

该集成路径建议使用 Cyclone DDS：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

`MotionHighLevelClient` 和 `SystemRpcClientBase` 会将目标 `device_id` 写入每个 `System.srv`
请求；robotServer 按目标设备 SN 过滤 RPC 请求，只有匹配设备响应。但该机制只覆盖 RPC，不能
隔离 `/motion/observed`、`/sensor/observed` 和 `/robotServer/Event` 等
原始 topic，因为这些消息当前没有可供 bridge 过滤的 `device_id`。

此外，多个 bridge 使用相同的 `/cmd_vel`、`/motion/*`、`/odom` 等全局 ROS 名称时也会发生接口
冲突。因此当前多机器人部署应同时做到：每条机器人使用独立的 `ROS_DOMAIN_ID`，并避免多个
bridge 出现在同一 ROS graph。仅设置不同的 RPC `device_id` 不足以保证多机器人隔离。

字段边界如下：

- `device_id` 是 `uniubi/srv/System` 的显式字段，用于目标设备路由。
- `Header.msg` 的 `client_id` / `request_id` 来自 `Request.idl`；`System.srv` 不含该 Header 字段。

ROS 2/RMW 使用 service request header 将响应关联到对应请求。当前示例按上述服务端路由契约工作，并检查响应 `code` 和业务 payload；不会额外比较 `response.device_id`。

二次开发时，业务代码通常通过封装后的客户端方法发起调用；新增 RPC 时，优先在示例客户端封装层扩展。

## System.srv 调用封装

`uniubi/srv/System.srv` 是示例客户端内部使用的 RPC service 接口。正常二次开发应优先使用封装后的调用层：

- `MotionHighLevelClient`：高级动作、取控、续约、释放等业务流程。
- `SystemRpcClientBase`：统一处理 `service` / `method` / `params` 请求构造、超时和响应等待。

新增 ROS 2 示例或扩展现有示例时，应沿用这几个入口，保持请求 `device_id` 和超时处理一致。需要核对 `.srv` / `.msg` 字段时，以 `uniubi_robot_msgs` 仓库发布的同名接口包为准。

## HighLevel 动作是异步的

`startAction`、`stopAction` 等 RPC 返回成功，只代表机器人已接受请求，不代表真实动作已经完成。

运动测试收尾建议：

1. 发送 `stopAction`。
2. 发送 `startAction("laying")` 或等价的 laying RPC。
3. 轮询 `queryMotionState`，直到返回空对象（`{}`）或包含 `"action":"laying"`。
4. 确认机器人到达安全状态后再释放控制权。

## 音频 URL 入库是异步的

`addAudioFile` 可能只表示下载任务已入队。通过 URL 上传音频时，应轮询 `queryAudioPlayList` 并传入 `{"type":"customVoice"}`，直到上传的 `id` 出现后再播放或删除。

## 安全门控

真实机器人上应将高风险运动放在明确的人工确认之后。带速度参数的 walking、`move`、`bipedStand`、`handstand`、`waveBody` 和 `jump*` 都按高风险动作处理。急停、TRC 全零帧、音频播放 / 暂停 / 停止、音频增删和灯光设置不属于高风险运动动作，但仍需要满足接口持权和参数要求。
