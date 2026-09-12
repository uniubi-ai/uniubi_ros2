# 扩展业务接口

这些接口使用 `uniubi_motion_client` 直接调用设备 RPC，不依赖 SDK 动态库。SDK、ROS 消息和设备版本须匹配；本次无需修改主仓或 `uniubi_robot_msgs`。

除取权/释放使用 `std_srvs/srv/Trigger` 外，下列 service 均使用 `uniubi_motion_bridge/srv/JsonCommand`。请求 `params_json` 是 JSON 对象（空字符串按 `{}`）；返回 `success`、`error_code`、`message`，查询结果在 `result_json`。字段遵循设备 RPC，不在 ROS 层改写。

| Service | 参数/结果 | 控制权 |
|---|---|---|
| `/motion/query_system_status` | `{}` → 完整系统状态 JSON | 无需 |
| `/motion/query_state` | `{}` → 当前动作状态 JSON，可能为 `{}` | 无需 |
| `/motion/query_motor_layout` | `{}` → 电机布局 JSON | 无需 |
| `/motion/set_action_params` | 当前动作的完整参数对象 | 需要 |
| `/audio/query_play_list` | 如 `{"type":"customVoice"}` → 文件列表 | 无需 |
| `/audio/query_play_detail` | `{}` → 播放详情 | 无需 |
| `/audio/add_file` | 设备 `addAudioFile` 参数，超时 30 秒 | 需要 |
| `/audio/delete_file` | 如 `{"id":"1"}` | 需要 |
| `/audio/start_play` | 如 `{"list":[{"id":"1"}],"volume":20,"repeat":1}` | 需要 |
| `/audio/pause_play` | `{}` | 需要 |
| `/audio/stop_play` | `{}` | 需要 |
| `/light/query_brightness` | `{}` → 亮度 JSON | 需要，与 SDK 一致 |
| `/light/set_brightness` | `{"brightness":20}`，整数 0–100 | 需要 |

这些是设备文件播放接口，与媒体驱动的 PCM `audio/playback`、`audio/reset` 独立。`add_file` 不负责上传 ROS 主机本地文件；资源地址等参数按当前设备接口契约提供。

## 控制权及动作参数

新增写接口不会隐式取权。显式调用 `/motion/acquire_control` 获取控制权（会先恢复小脑主控，与现有 `start_action` 相同），完成后用 `/motion/release_control` 释放。已有 `start_action` 的自动取权行为保持不变。释放路径可能先停止当前动作，因此不应把取权/释放当作纯灯光操作。

`set_action_params` 不启动/切换动作，参数是完整替换语义，使用设备字段名和单位。成功后清除旧的 `/cmd_vel` 待发送命令及其 watchdog 状态；service 参数按 SDK 语义持续生效，不由 `/cmd_vel` 超时自动归零。后续 `/cmd_vel` 会接管速度并恢复其 watchdog。不要同时使用两种参数控制来源。

```bash
ros2 service call /motion/query_system_status uniubi_motion_bridge/srv/JsonCommand '{}'
ros2 service call /audio/query_play_list uniubi_motion_bridge/srv/JsonCommand \
  '{params_json: "{\"type\":\"customVoice\"}"}'
# 取权会切换主控；先确认符合当前机器人使用场景。
ros2 service call /motion/acquire_control std_srvs/srv/Trigger '{}'
ros2 service call /light/set_brightness uniubi_motion_bridge/srv/JsonCommand \
  '{params_json: "{\"brightness\":20}"}'
ros2 service call /motion/release_control std_srvs/srv/Trigger '{}'
```

## GPS/UWB 观测

- `gps/observed`：`uniubi_motion_bridge/msg/GpsObserved`
- `uwb/observed`：`uniubi_motion_bridge/msg/UwbObserved`

话题为相对名称，支持 namespace/remap，QoS 为 SensorDataQoS。`stamp` 是 ROS 收到观测的时间；`device_timestamp` 原样保留 SensorObserved 时间戳，未做时钟转换。`data` 原样保存 `uniubi/GPSFrame` 或 `uniubi/UWBRawObserved`，包括信号、配对状态、位置/距离/角度及 `beacon_id`，不推测原始单位。

有效和无效观测都发布。消费者必须检查 `data.valid`；收到消息不代表 GPS 已定位或 UWB 已配对。此处没有强行映射为 NavSatFix，也未计算 UWB 笛卡尔坐标。

## 无设备集成测试

构建 `uniubi`、`uniubi_motion_client`、`uniubi_motion_bridge` 并 source 安装环境后：

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 src/uniubi_motion_bridge/test/extended_interfaces.py
```

测试启动隔离的模拟 System 服务和真实 bridge，检查控制权门控、RPC 参数、错误返回、GPS/UWB 原始字段与无效标志。该测试不会连接设备，不代表新增接口已完成设备实测。
