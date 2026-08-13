# Documentation Guide

**English** | [简体中文](README.zh-CN.md)

This page directs each type of developer to the relevant documentation so that application usage, low-level protocols, and interface-maintenance rules remain clearly separated.

## Typical ROS 2 application developers

Read these documents in order:

1. The repository [README](../README.md) for installation and the first launch.
2. The [Motion bridge guide](motion_bridge.md) for `/motion/*`, `/cmd_vel`, and standard observation topics.
3. [Runtime notes](runtime_notes.md) for asynchronous action semantics and safety boundaries before hardware testing.

Typical application nodes do not need to call `uniubi/srv/System` directly or understand DDS IDL mappings.

## C++ developers with custom control flows

Read [ROS 2 integration modes](ros2_usage_modes.md) first and confirm that bypassing the bridge is necessary. Then inspect the headers and examples under `src/uniubi_motion_client`. This client is a source component of the repository and does not depend on `librobotMotionSdk.so`, but the application must manage connection, control ownership, and shutdown correctly.

## Direct DDS / ROS 2 protocol developers

Read:

1. The direct-protocol boundary in [ROS 2 integration modes](ros2_usage_modes.md).
2. [`uniubi_robot_dds_api.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.md) for the complete RPC, Event, raw-topic, control-ownership, and QoS contract.
3. [Runtime notes](runtime_notes.md) for DDS Domain, device matching, and runtime constraints in this repository.
4. [`ros2_dds_interop_overview.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/ros2_dds_interop_overview.md) when maintaining IDL mappings.
5. [`uniubi_robot_msgs`](https://github.com/uniubi-ai/uniubi_robot_msgs) as the source of truth for message definitions.

Direct protocol integration is one complete approach, not two separate “Direct DDS” and “Direct RPC” modes. Developers who only need raw read-only data can focus on the relevant topic sections. Adding RPC calls, performing control, or porting to another DDS framework requires understanding requests and responses, Event handling, control ownership, and data channels together.
