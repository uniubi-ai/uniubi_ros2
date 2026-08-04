# Uniubi ROS 2

Uniubi 机器人的 ROS 2 接入仓库，提供运动控制 bridge、可复用的 C++ ROS 2 客户端、Direct RPC
调用和原始 DDS topic 接入示例。

robotServer 原始 `.msg` / `.srv` 定义统一来自
[`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs)，其 ROS 2 package 名和
接口类型前缀是 `uniubi`。bridge 专用的 `MotionStatus.msg` 和
`StartMotionAction.srv` 由 `uniubi_motion_bridge` 自己维护。本仓库不链接
`librobotMotionSdk.so`；所有模式均通过 ROS 2 service 和 DDS topic 对接 robotServer。

## 从这里开始

仓库提供四种使用方式。业务开发默认选择 Motion bridge：

| 使用方式 | 适合谁 | 控制权管理 | 主要接口 | 推荐度 |
|---|---|---|---|---|
| Motion bridge | 普通 ROS 2 业务节点 | bridge 自动取权、续约和释放 | `/motion/*`、`/cmd_vel`、标准传感器 topic | 推荐 |
| `uniubi_motion_client` | 需要更多高级运控能力的 C++ 开发者 |应用显式调用 `connect/startControl/releaseControl` | C++ 方法和回调 | 高级 |
| Direct RPC | 新增 RPC、验证请求响应协议 | 控制类 RPC 由应用自行管理 | `uniubi/srv/System` | 专家 |
| Direct DDS topic | 原始数据订阅、QoS 和类型调试 | 只读订阅不需要控制权 | `/motion/observed` 等原始 topic | 专家 |

四种方式的完整优缺点和选型说明见
[`docs/ros2_usage_modes.md`](docs/ros2_usage_modes.md)。

如果目标只是读取里程计、关节、IMU 或电池，使用 bridge 发布的标准 topic 即可，不需要申请运动控制权。

## 仓库组成

```text
uniubi_robot_msgs
└── uniubi                    # ros2/ 构建出的 ROS 2 msg/srv 接口包

uniubi_ros2
├── uniubi_motion_client      # 源码形式的 RPC/DDS C++ 封装，不是 SDK 动态库
└── uniubi_motion_bridge      # 面向业务节点的节点及 bridge 专用 msg/srv
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
- 已确认目标机器人的 `device_id`。该字段用于 RPC 路由，不能隔离原始 DDS topic。
- 当前建议每条机器人使用独立的 `ROS_DOMAIN_ID`；不要让多条机器人及其 bridge 共享同一 Domain。
- 推荐使用 Cyclone DDS。

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

如机器上有多个网卡，还需要通过 `CYCLONEDDS_URI` 明确指定机器人所在网卡。

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

## 推荐方式：Motion bridge

启动 bridge：

```bash
ros2 launch uniubi_motion_bridge motion_bridge.launch.py \
  device_id:=<device-id>
```

bridge 启动后只连接 robotServer，不会立即申请运动控制权。第一次调用
`/motion/start_action` 时才自动取权并启动续约。

一个最小控制流程：

```bash
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: walking, params_json: '{}'}"

ros2 topic pub --rate 50 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}'

ros2 service call /motion/stop_action std_srvs/srv/Trigger '{}'
ros2 service call /motion/release_control std_srvs/srv/Trigger '{}'
```

`/cmd_vel` 只把速度参数交给当前动作，不会取权、启动动作或切换动作。`linear.y > 0`
沿用 UniUbi 定义，表示向右横移。

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
[`docs/motion_bridge.md`](docs/motion_bridge.md)。

## 其他使用方式

- 需要在自己的 C++ 节点中直接调用高级运控方法：使用
  [`uniubi_motion_client`](docs/ros2_usage_modes.md#方式二uniubi_motion_client-c-客户端)。
- 需要新增请求/响应接口：使用
  [Direct RPC](docs/ros2_usage_modes.md#方式三direct-rpc)。
- 需要订阅原始消息或调试 QoS/类型映射：使用
  [Direct DDS topic](docs/ros2_usage_modes.md#方式四direct-dds-topic)。
- 只读订阅原始 Walk 里程计：

```bash
ROS_DOMAIN_ID=42 \
UNIUBI_TEST_ODOMETRY_TOPIC=/motion/odometry \
ros2 run uniubi_motion_client motion_odometry_subscriber
```

## 文档导航

| 文档 | 面向人群 | 内容 |
|---|---|---|
| [README](README.md) | 所有开发者 | 选型、安装和推荐方式快速开始 |
| [ROS 2 使用方式](docs/ros2_usage_modes.md) | 架构设计与高级开发者 | 四种方式的流程、优缺点和选择建议 |
| [Motion bridge 使用手册](docs/motion_bridge.md) | 普通业务开发者 | `/motion/*`、`/cmd_vel`、状态和观测接口 |
| [运行注意事项](docs/runtime_notes.md) | 联调和协议开发者 | DDS、设备匹配、异步语义和安全边界 |
| [DDS / ROS 2 Direct API](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.md) | 协议开发者 | 原始 RPC、DDS topic 和字段契约 |
| [ROS 2 与 DDS 映射](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/ros2_dds_interop_overview.md) | 接口维护者 | `.msg/.srv` 与 DDS IDL 映射规则 |
| [`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs) | 消息维护者 | ROS 2 msg/srv 和 DDS IDL 信源 |

文档阅读路径的进一步说明见 [`docs/README.md`](docs/README.md)。

## 安全

首次实机联调建议先验证只读 topic，再验证站立、趴下等低风险动作。walking、双足、倒立和
`jump*` 等动作必须在空旷场地、急停可用并有人值守的条件下测试。

`stop_action` 不等于释放控制权；业务结束后应显式调用 `/motion/release_control`。

## 许可证

本仓库中的 UniUbi 原创 ROS 2 集成代码、示例和文档使用 Apache License 2.0。vendored
jsoncpp 按其原始许可证授权。详见 [LICENSE](LICENSE)、[NOTICE](NOTICE) 和
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
