# DDS 网络接入配置

[English](dds_network.md)

以下 Host Domain 42 配置面向配套 DDS 更新后的固件，不改变机器人内部 Domain 1 配置。使用 `rmw_cyclonedds_cpp` 和匹配狗端类型的消息包。官方 CycloneDDS 也可在互通、RMW ABI/依赖、QoS 和实际数据验证后使用；官方最新版不等于已验证版本。不要覆盖 `/opt/ros` 系统库，也不需要为此升级整个 ROS 发行版。配套 DDS，以及必要时同一 ROS 发行版、CPU 架构的 RMW，应放在独立 overlay 中。

## 普通以太网/Wi-Fi 局域网

保存为 `dds-public.xml`，替换为客户端实际网卡。默认组播发现，不配置 peers，业务按 SN 选择狗。

```xml
<CycloneDDS><Domain Id="42"><General><Interfaces><NetworkInterface name="REPLACE_WITH_ROBOT_NIC"/></Interfaces><AllowMulticast>true</AllowMulticast><DontRoute>false</DontRoute></General><Discovery><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>
```

## 专用直连

保存为 `dds-direct.xml`，替换网卡。客户端使用 `172.29.140.2` 至 `172.29.140.254/24` 的唯一地址；`.1` 留给机器狗，`.0`、`.255` 不可用。仅 DDS 直连不需要默认网关或 DNS。使用普通以太网地址时，即使物理上是一根网线，也按普通网络配置。

```xml
<CycloneDDS><Domain Id="42"><General><Interfaces><NetworkInterface name="REPLACE_WITH_WIRED_NIC"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="172.29.140.1:17910"/></Peers></Discovery></Domain></CycloneDDS>
```

ROS 2 可用 XML 关闭组播，这与 SDK 当前生成 `AllowMulticast=true` 的 XML 不同。ROS 2 不调用 SDK `setNetworkConfig()`。直连 XML 省略 `DontRoute`，默认 `false`；它不是直连模式开关。此前 `.2`、`.3`、`.4` 测试使用 `DontRoute=true`，该简化配置仍需真实回归，不能沿用旧结论。客户端不配置狗端扩展 `DirectDiscovery`。peer 填地址，不填 CIDR，也不是访问白名单。`17910` 是狗端发现单播端口，不是所有 DDS 数据的唯一端口。

## 加载与检查

先加载 ROS、经过验证的 DDS/RMW overlay，再加载消息/应用工作空间：

```bash
source /opt/ros/humble/setup.bash
# source the validated DDS/RMW overlay, then the message/application workspace
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///absolute/path/dds-public.xml
ros2 daemon stop
ros2 topic list --no-daemon --spin-time 5
```

直连改用 `dds-direct.xml`。配置变化后重启自己的 ROS 节点。记录 DDS 源码 commit/补丁、架构、RMW 版本、库 SHA256/依赖和实际加载路径。真实节点启动后检查 `/proc/PID/maps` 中的 `libddsc`；仅设置 `LD_LIBRARY_PATH` 不代表更新完成。

检查本机 IP；直连执行 `ip route get 172.29.140.1` 确认出口和源地址。话题发现后订阅目标 SN 的观测话题，检查实际数据、时间戳、持续接收、QoS、重连和退出。发现或 SDK RPC 成功不等于 ROS 2 数据订阅验收。没有数据还需核对发布使能条件、类型和 QoS。多狗分别核对 SN。不要将多条专用直连网络桥接到共享二层网络，否则狗端相同 `.1` 地址会冲突。

跨子网或组播受限时可能需要显式可达 peers、`DontRoute=false`、双向路由、可达 advertised locator 和 UDP 放行。这些真机场景、配套 DDS 的完整观测流和长时间稳定性仍待验收。本次文档更新没有新增真机通过结果。

网络设置见[连接外设](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/how-to/connect-peripherals.zh-CN.md)。
