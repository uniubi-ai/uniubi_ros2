# 文档阅读路径

[English](README.md) | **简体中文**

本文说明不同开发者应该阅读哪些文档，避免把业务用法、底层协议和接口维护规范混在一起。

## 普通 ROS 2 业务开发者

按以下顺序阅读：

1. 仓库根目录 [README](../README.zh-CN.md)：完成安装和首次启动。
2. [Motion bridge 使用手册](motion_bridge.zh-CN.md)：使用 `/motion/*`、`/cmd_vel` 和标准观测 topic。
3. [运行注意事项](runtime_notes.zh-CN.md)：实机测试前确认动作异步语义和安全边界。

普通业务节点不需要直接调用 `uniubi/srv/System`，也不需要理解 DDS IDL 映射。

## 需要自定义控制流程的 C++ 开发者

先阅读 [ROS 2 使用方式](ros2_usage_modes.zh-CN.md)，确认确实需要绕过 bridge；然后查看
`src/uniubi_motion_client` 的头文件和示例。该客户端是仓库源码组件，不依赖
`librobotMotionSdk.so`，但调用方需要自行管理连接、控制权和退出流程。

## DDS / ROS 2 协议直连开发者

阅读：

1. [ROS 2 使用方式](ros2_usage_modes.zh-CN.md) 中的协议直连边界。
2. [`uniubi_robot_dds_api.zh-CN.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.zh-CN.md)：RPC、Event、原始 topic、控制权和 QoS 的完整契约。
3. [运行注意事项](runtime_notes.zh-CN.md)：本仓接入时的 DDS Domain、设备匹配和运行约束。
4. [`ros2_dds_interop_overview.zh-CN.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/ros2_dds_interop_overview.zh-CN.md)：需要维护类型时再阅读 IDL 映射规范。
5. [`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs/blob/main/README.zh-CN.md)：消息定义信源。

这是一种完整的协议级接入方式，不再拆分成 Direct DDS 和 Direct RPC 两种方案。只需要原始
只读数据的开发者可以只阅读相应 topic 小节；新增 RPC、执行控制或移植到其他 DDS 框架时，
需要同时理解请求响应、Event、控制权和数据通道。
