# 运行注意事项

本文记录使用 `uniubi_interface_test` 二次开发时容易踩坑的 ROS 2 集成行为。完整 DDS / ROS 2 协议说明统一维护在 [uniubi-docs](https://github.com/uniubi-ai/uniubi-docs)。

## Direct API 范围

本仓不链接 `librobotMotionSdk.so`。ROS 2 示例通过 `uniubi/srv/System` 和 DDS topic 直接对接 robotServer：

- RPC service：`uniubi/srv/System`
- Event topic：`/robotServer/Event`
- Motion observation topic：`/motion/observed`
- Sensor observation topic：`/sensor/observed`
- TRC topic：`/motion/trc`

ROS 2 Direct API 测试通过，只能说明 ROS 2 消息 / 服务契约和 DDS 路由可用；不能代表 C++ 或 Python SDK runtime 运行库链路可用。

## DDS 与设备匹配

该集成路径建议使用 Cyclone DDS：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

同一 DDS Domain 内存在多台机器人时，请求必须携带目标 `device_id`。推荐直接复用 `uniubi_interface_test` 里的 `MotionHighLevelClient` 或 `SystemRpcClientBase`，由示例封装统一处理请求发送和响应等待。

字段边界如下：

- `device_id` 是 `uniubi/srv/System` 的显式字段，用于目标设备路由；收到响应后仍要检查响应 `device_id` 是否匹配目标设备。
- `Header.msg` 的 `client_id` / `request_id` 来自 `Request.idl`；`System.srv` 不含该 Header 字段。

二次开发时，业务代码通常通过封装后的客户端方法发起调用；新增 RPC 时，优先在示例客户端封装层扩展。

## System.srv 调用封装

`uniubi/srv/System.srv` 是示例客户端内部使用的 RPC service 接口。正常二次开发应优先使用封装后的调用层：

- `MotionHighLevelClient`：高级动作、取控、续约、释放等业务流程。
- `SystemRpcClientBase`：统一处理 `service` / `method` / `params` 请求构造、超时和响应检查。

新增 ROS 2 示例或扩展现有示例时，应沿用这几个入口，保持 `device_id`、超时和目标设备响应检查一致。需要核对 `.srv` / `.msg` 字段时，以 `uniubi_robot_msgs` 发布的接口包为准。

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

真实机器人上应将高风险运动放在明确的人工确认之后。带速度参数的 walking、`move`、`bipedStand`、`handstand`、`waveBody`、`peakLoadStand` 和 `jump*` 都按高风险动作处理。急停、TRC 全零帧、音频播放 / 暂停 / 停止、音频增删和灯光设置不属于高风险运动动作，但仍需要满足接口持权和参数要求。
