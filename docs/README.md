# 文档阅读路径

本文说明不同开发者应该阅读哪些文档，避免把业务用法、底层协议和接口维护规范混在一起。

## 普通 ROS 2 业务开发者

按以下顺序阅读：

1. 仓库根目录 [README](../README.md)：完成安装和首次启动。
2. [Motion bridge 使用手册](motion_bridge.md)：使用 `/motion/*`、`/cmd_vel` 和标准观测 topic。
3. [运行注意事项](runtime_notes.md)：实机测试前确认动作异步语义和安全边界。

普通业务节点不需要直接调用 `uniubi/srv/System`，也不需要理解 DDS IDL 映射。

## 需要自定义控制流程的 C++ 开发者

先阅读 [ROS 2 使用方式](ros2_usage_modes.md)，确认确实需要绕过 bridge；然后查看
`src/uniubi_motion_client` 的头文件和示例。该客户端是仓库源码组件，不依赖
`librobotMotionSdk.so`，但调用方需要自行管理连接、控制权和退出流程。

## 需要原始数据的 ROS 2 开发者

阅读：

1. [ROS 2 使用方式](ros2_usage_modes.md) 中的 Direct DDS topic 边界。
2. [`uniubi_robot_dds_api.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.md)：原始 topic 和 QoS。
3. [`ros2_dds_interop_overview.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/ros2_dds_interop_overview.md)：IDL 映射规范。
4. [`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs)：消息定义信源。

这组文档主要面向需要 bridge 未转换字段、原始数据接入或排查 DDS 类型兼容问题的开发者。

## Direct RPC 和协议维护者

阅读：

1. [ROS 2 使用方式](ros2_usage_modes.md) 中的 Direct RPC 边界。
2. [运行注意事项](runtime_notes.md) 中的 DDS Domain、设备匹配和 RPC 封装约束。
3. [`uniubi_robot_dds_api.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.md)：wire API。

这组文档只面向新增 RPC、核对 JSON 请求响应或定位 robotServer 协议问题的开发者。
