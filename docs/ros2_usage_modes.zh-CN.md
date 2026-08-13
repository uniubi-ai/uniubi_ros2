# ROS 2 使用方式与选型

[English](ros2_usage_modes.md) | **简体中文**

Uniubi ROS 2 提供一个默认业务入口和两个按需使用的高级入口。它们均不依赖
`librobotMotionSdk.so`，但解决的问题不同：

- Motion bridge：常用运动控制和部分标准 ROS 2 观测接口。
- `uniubi_motion_client`：自定义 C++ 高级运控流程。
- DDS / ROS 2 协议直连：直接处理 RPC、Event、数据 topic 和 TRC。

协议直连是一种完整接入方式，不再把 Direct DDS topic 和 Direct RPC 列成两个平级方案。
其中原始数据订阅和 RPC 控制只是同一套协议中的不同通道，完整契约见
[`uniubi_robot_dds_api.zh-CN.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.zh-CN.md)。

## 选型对比

| 方式 | 主要用途 | 控制权 | 优点 | 主要代价 |
|---|---|---|---|---|
| Motion bridge | 业务控制与标准观测 | bridge 自动管理 | 最简单、ROS 2 语义清晰 | 受公开能力范围限制 |
| `uniubi_motion_client` | 自定义 C++ 高级流程 | client 续约，应用管理生命周期 | 能力完整、仍有统一封装 | 需要正确管理 executor 和收尾 |
| DDS / ROS 2 协议直连 | 原始数据、协议维护和跨框架接入 | 只读数据不需要；控制流程自行管理 | 能力和字段最完整，最接近 wire contract | 需要理解 RPC、Event、IDL、QoS 和控制权 |

默认选择规则：

- 想控制动作、发送速度并读取状态：Motion bridge。
- 单个 C++ 进程需要 bridge 尚未开放的能力：`uniubi_motion_client`。
- 需要原始字段、定位 DDS 类型/QoS、新增协议接口或跨框架接入：DDS / ROS 2 协议直连。

## 方式一：Motion bridge

Motion bridge 当前以常用运动控制为主，并选择性转换里程计、关节、IMU 和电池等观测数据。
它不是 High Level Client 或底层 DDS / ROS 2 直连接口的全量功能移植。是否支持某项能力，应以
[Motion bridge 使用手册](motion_bridge.zh-CN.md) 当前列出的 topic、service 和参数为准；系统、音频、
媒体、原始字段及后续新增的协议能力可能需要使用 `uniubi_motion_client` 或协议直连。

业务节点只与以下公开接口交互：

```text
/motion/* services
/motion/status
/cmd_vel
/odom
/joint_states
/imu/data
/battery_state
```

bridge 是唯一的高级运控客户端，内部完成连接、按需取权、续约、Event 处理和退出释放。

优点：

- 最接近常规 ROS 2 topic/service 使用方式。
- 业务节点不需要理解 controller token、租约和内部 JSON RPC。
- 多个业务节点共享一个控制入口，减少重复取权冲突。
- 关节、IMU、电池和里程计转换成社区常用消息。

缺点：

- 以运动控制为主，只能使用 bridge 当前已经公开和转换的能力。
- `/motion/status` 默认以 10 Hz 查询动作和速度，最多有一个查询周期的显示延迟。
- `/cmd_vel` 没有逐条响应，最近失败通过 `/motion/status` 反映。

详细用法见 [Motion bridge 使用手册](motion_bridge.zh-CN.md)。

## 方式二：`uniubi_motion_client` C++ 客户端

`uniubi_motion_client` 是本仓库源码编译出的 C++ 封装，不是 SDK 动态库。

典型流程：

```text
创建 ROS 2 node 与 executor
→ 在 connect 前注册状态、Event 和观测回调
→ connect()
→ queryCapabilities()
→ startControl()
→ startAction()/setActionParams()/stopAction()
→ 持续 spin；有成功控制调用时由调用本身刷新租约，空闲时 client 自动 renewMotionControl
→ releaseControl()
→ disconnect()
```

优点：

- 可以使用比 bridge 更完整的高级运控、系统、音频和原始 TRC 接口。
- 复用统一的 RPC 构造、响应匹配、Event 解析和续约逻辑。
- 适合把高级能力嵌入一个受控的 C++ 进程。

缺点：

- 应用必须正确管理 executor、回调顺序和完整退出流程。
- `connect()` 只表示连接，不能当作已经取得控制权。
- 多个进程各自创建 client 时可能竞争控制权。
- 对 Python 或普通 ROS 2 业务节点不如 bridge 直观。

示例入口：

```bash
UNIUBI_TEST_ROS_DOMAIN_ID=42 \
UNIUBI_TEST_SERVICE_NAME=robotServer \
UNIUBI_TEST_EVENT_TOPIC=/robotServer/Event \
UNIUBI_TEST_DEVICE_ID=<device-id> \
ros2 run uniubi_motion_client motion_high_level_client_example
```

示例中的真实运动默认关闭，构建或启动成功不代表实机动作已经验证。

## 方式三：DDS / ROS 2 协议直连

协议直连绕过 bridge 和 `uniubi_motion_client` 的业务封装，直接使用 robotServer 的完整通信协议：

```text
RPC 请求/响应       查询、配置、控制权和动作控制
Event               设备主动推送的状态变化
数据 topic           motion observed、sensor observed
TRC 控制 topic       高频实时控制帧
```

只读取某个原始 topic 时可以仅使用所需的数据通道；一旦进行动作或 TRC 控制，就必须同时正确
实现 RPC 控制权生命周期和 Event 处理。普通业务不应把其中一个通道当作另一套独立接入方式。

### 原始数据 topic

应用直接订阅原始数据 topic，不经过 bridge 的标准消息转换。常用入口包括：

```text
/motion/observed
/sensor/observed
```

这条路径主要用于持续数据流。`/motion/observed` 和 `/sensor/observed` 默认关闭，
需要先建立 reader，再通过 RPC 调用
`setMotionObservedEnable` 开启，但该启用调用不要求持有运动控制权。

优点：

- 只读数据订阅不需要运动控制权；里程计通过 `SensorObserved.odom` 获取。
- 保留设备错误码、有效位、原始时间戳和生命周期字段。
- 适合验证 DDS 发现、消息类型、发布频率和 QoS。

缺点：

- 使用 UniUbi 自定义消息，而不是全部转换为社区标准消息。
- 调用方需要正确设置 reliability、history depth 和 durability。
- 原始时间戳、坐标和字段语义需要按协议解释。

`/motion/trc` 是控制 topic，不属于普通只读数据流。直接发布 TRC 需要已有控制会话、正确的
控制 ID 和持续发送约束；普通业务应使用 bridge 或 client。

### RPC、Event 和控制

应用直接使用 `uniubi/srv/System` 调用 robotServer。它适合查询能力、查询状态、验证新 RPC，
以及定位问题发生在业务封装、client 还是 robotServer。

只读 RPC 不需要控制权。控制类 RPC 则要求调用方自行完成：

```text
takeMotionControl
→ 保存 controller/lease/rawActionId
→ renewMotionControl
→ 解析 /robotServer/Event
→ 控制调用
→ stopMotionAction
→ releaseMotionControl
```

优点：

- 封装最少，可精确控制 service、method、JSON payload 和请求时序。
- 新 RPC 不必先修改 client 或 bridge 就能验证。
- 适合 wire contract 调试和协议开发。

缺点：

- 需要自行处理 `device_id`、超时、返回码和 JSON schema。
- 控制类 RPC 容易遗漏续约、被抢权处理和正常释放。
- 接口接近底层协议，不适合作为普通业务代码的默认依赖。

协议直连的契约以
[`uniubi_robot_dds_api.zh-CN.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.zh-CN.md)
为准。普通业务开发者不应直接拼装控制 RPC。

## 只读使用

读取状态不等于取得运动控制权。

普通业务推荐启动 bridge 后订阅：

```text
/odom
/joint_states
/imu/data
/battery_state
/motion/status
```

需要完整原始字段时再使用协议直连中的原始 topic。bridge 标准观测和原始只读订阅都不需要调用
`/motion/start_action`。

## 不要混用多个控制入口

同一机器人可以同时存在遥控器、App 和 SDK 类控制来源。每个部署建议只运行一个
`uniubi_motion_bridge` 作为 ROS 2 高级控制入口。不要同时让多个 bridge、client 示例和协议
直连控制程序竞争控制权。

控制结束后应显式释放；服务端租约是异常退出兜底，不应代替正常收尾。
