# Uniubi ROS 2

Uniubi ROS 2 集成仓库，提供基于 [`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs) 的 ROS 2 客户端和示例。

本仓库不维护 `.msg` / `.srv` 定义；消息接口统一由 [`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs) 提供。本仓库也不链接 C++ SDK 的 `librobotMotionSdk.so`，ROS 2 示例通过 `uniubi/srv/System` 直接对接 robotServer 的 DDS/RPC 通道。

## 目录结构

```
.
├── src/
│   └── uniubi_interface_test/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/uniubi_interface_test/
│       ├── src/
│       └── third_party/jsoncpp/
├── launch/
├── rviz/
└── tests/
```

## 前置条件

- ROS 2 环境已安装并完成 `source`
- 已构建并 source [`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs) 中的 `uniubi` 接口包
- 客户端与机器人处于同一 DDS Domain 和可发现网络
- 真实机器人测试前已确认目标 `device_id`

## 构建

```bash
mkdir -p ~/ros2_ws/src

# 先获取 uniubi_robot_msgs 提供的接口包
git clone https://github.com/uniubi-ai/uniubi_robot_msgs.git ~/uniubi_robot_msgs
cp -r ~/uniubi_robot_msgs/ros2 ~/ros2_ws/src/uniubi

# 再放入本仓库 ROS 2 示例包
git clone https://github.com/uniubi-ai/uniubi_ros2.git ~/uniubi_ros2
cp -r ~/uniubi_ros2/src/uniubi_interface_test ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select uniubi uniubi_interface_test
. install/setup.bash
```

## 环境配置

真实机器人联调前必须确认 DDS Domain、RMW 实现和网卡配置。运行示例使用 `UNIUBI_TEST_ROS_DOMAIN_ID` 指定 DDS Domain：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 如需指定 Cyclone DDS 网卡，可在脚本中生成或引用 CYCLONEDDS_URI
```

## 运行示例

```bash
UNIUBI_TEST_ROS_DOMAIN_ID=42 \
UNIUBI_TEST_SERVICE_NAME=robotServer \
UNIUBI_TEST_EVENT_TOPIC=/robotServer/Event \
UNIUBI_TEST_DEVICE_ID=<device-id> \
ros2 run uniubi_interface_test motion_high_level_client_test
```

环境变量：

| 变量 | 默认值 | 说明 |
|---|---|---|
| `UNIUBI_TEST_ROS_DOMAIN_ID` | `42` | DDS Domain ID |
| `UNIUBI_TEST_SERVICE_NAME` | `robotServer` | ROS 2 service 名称 |
| `UNIUBI_TEST_EVENT_TOPIC` | `/robotServer/Event` | 事件 topic |
| `UNIUBI_TEST_DEVICE_ID` | 无 | 目标设备 SN，真实环境必须显式配置 |

`motion_high_level_client_test` 的默认流程为：查询运动能力和系统状态、获取并续约控制权、尝试播放 / 暂停 / 停止音频、查询音频状态、开启再关闭运控观测上报，最后释放控制权。运动动作和原始 TRC 默认由 `kEnableMotionActionDemo=false` 关闭；示例默认不会执行站立、趴下或 walking。打开该开关后，当前测试代码执行 walking 和 TRC 调试，运行前必须确认场地、急停和人工接管条件。

## 多设备匹配

同一 DDS Domain 存在多台设备时，运行示例必须填写目标 `device_id`。`MotionHighLevelClient` 和 `SystemRpcClientBase` 会将该字段写入每个 `System.srv` 请求；robotServer 按目标设备 SN 过滤请求，只有匹配设备响应。

字段边界如下：

- `device_id` 是 `uniubi/srv/System` 的显式字段，用于目标设备路由。
- `Header.msg` 的 `client_id` / `request_id` 来自 `Request.idl`；`System.srv` 不含该 Header 字段。

ROS 2/RMW 使用 service request header 将响应关联到对应请求。当前示例按上述服务端路由契约工作，并检查响应 `code` 和业务 payload；不会额外比较 `response.device_id`。业务代码通常通过封装后的客户端方法发起调用；新增 RPC 时，优先在示例客户端封装层扩展。

## 安全策略

首次真实机器人联调建议只执行站立、趴下等低风险动作。`walking`、`move`、`bipedStand`、`handstand`、`jump*`、`damp` 等高风险运动动作应在空旷场地和人工接管条件下执行。急停、音频播放/暂停/停止、音频文件增删、灯光亮度设置、TRC 归零帧不属于高风险运动动作，但仍需满足接口持权和参数要求。

## 相关文档

- 本仓运行注意事项：[`docs/runtime_notes.md`](docs/runtime_notes.md)
- DDS / ROS 2 直连接入 API：[`uniubi-docs/docs/uniubi_robot_dds_api.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.md)
- ROS 2 与 DDS 映射：[`uniubi-docs/docs/ros2_dds_interop_overview.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/ros2_dds_interop_overview.md)
- 消息定义仓库：[`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs)

## 许可证

本仓库中的 UniUbi 原创 ROS 2 集成代码、示例和文档使用 Apache License 2.0。vendored jsoncpp 按其原始许可证授权。详见 [LICENSE](LICENSE)、[NOTICE](NOTICE) 和 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
