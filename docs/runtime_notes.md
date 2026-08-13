# Runtime Notes

**English** | [简体中文](runtime_notes.zh-CN.md)

This document records ROS 2 integration behaviors that commonly cause mistakes when extending `uniubi_motion_client` or `uniubi_motion_bridge`. The complete DDS / ROS 2 protocol is maintained in [`uniubi-docs`](https://github.com/uniubi-ai/uniubi-docs).

## Direct DDS / ROS 2 protocol

Direct protocol integration includes RPC, Event, raw data topics, and TRC. The sections below describe each channel's runtime boundary; they do not define “Direct DDS” and “Direct RPC” as separate peer-level integration modes.

### Raw data topics

Continuous raw data is available through DDS/ROS 2 topics:

- Motion observation: `/motion/observed`
- Sensor observation: `/sensor/observed` (GPS, UWB, and Walk odometry)
- Raw control: `/motion/trc` (not an ordinary read-only stream)

Subscribing to observation topics does not require High Level control ownership. `/motion/observed` and `/sensor/observed` are disabled by default. Direct-protocol clients must create the reader first and then call the control-free `setMotionObservedEnable()` RPC. The Motion bridge manages raw observation streams automatically; its application nodes only subscribe to the standard ROS 2 topics published by the bridge. A successful raw-topic test proves only that the message type, DDS discovery, and QoS path work.

`/sensor/observed` uses `BEST_EFFORT` / `KEEP_LAST depth=1` / `VOLATILE` and is enabled with `setMotionObservedEnable(..., sensor_enable=true)`. Odometry comes from `SensorObserved.odom` and is valid only in Walk mode. When Walk ends, the final value for that interval is retained and `valid=false`. Entering Walk again establishes a new origin and increments `epoch`.

### RPC, Event, and control

The direct-protocol and motion packages in this repository do not link
`librobotMotionSdk.so`. They communicate with robotServer through
`uniubi/srv/System`:

- RPC service: `uniubi/srv/System`
- Asynchronous control events: `/robotServer/Event`

Read-only queries do not require control ownership. For control RPCs, the caller must manage acquisition, renewal, Event handling, and release. A successful RPC test proves only the request/response contract and routing; it does not validate the C++ or Python SDK runtime path.

## Motion bridge velocity safety boundary

- `start_action` acquires SDK High Level motion control when needed; no independent public `take_control` service exists.
- `/cmd_vel` does not acquire control, query, start, or switch actions. It passes three velocity fields directly to `setActionParams`.
- `linear.y` and Uniubi `lineVelocityY` both use positive-left/negative-right. The bridge does not invert the sign, and `/motion/status.line_velocity_y` uses the same convention.
- The bridge reads only `linear.x`, `linear.y`, and `angular.z`; it does not infer actions or clamp values by action. The SDK and server validate parameters and enforce safety limits.
- An input timeout sends one set of zero values for all three velocity fields. It does not stop the action or release control.
- High-rate input is coalesced to the latest value and limited by `cmd_vel_rate_hz` (30 Hz by default).
- Successful `startAction`, `setActionParams`, `stopAction`, and `emergencyStop` calls refresh the server-side control lease. The client sends `renewMotionControl` only when control RPCs have been idle for a renewal interval. Failed or timed-out RPCs do not count as renewal.
- `stop_action` stops the action while retaining and renewing control. `release_control` and process shutdown stop the action before releasing the control session.

`/odom` forwards only device-accumulated odometry with `valid=true`. It does not integrate again and currently publishes no TF. `position.y` and `twist.linear.y` remain positive-left/negative-right without bridge conversion. The raw lifecycle fields remain available in `/sensor/observed.odom`.

## Motion bridge status observations

- `/motion/status` publishes actual action, velocity, control state, and the latest error through a 10 Hz `queryMotionState` RPC. It is not involved in `/cmd_vel` delivery.
- `/motion/status` can lag by up to one query period and is not per-control-frame feedback.
- Internal `/robotServer/Event` processing immediately detects control preemption. Known events become structured state; unknown raw JSON is written only to DEBUG logs and is not exposed through `~/events`.
- `/joint_states`, `/imu/data`, and `/battery_state` are converted from `/motion/observed` and do not require High Level control ownership.
- The bridge obtains joint names and `(limbNo, jointNo)` through the read-only `getMotorLayout` RPC instead of relying on a fixed motor array order.
- `/joint_states` position/velocity/effort come from motor position/velocity/torque. Fault codes, online state, and temperature remain available only in raw `/motion/observed`.
- `/imu/data` is published only when acceleration and angular velocity are valid. Invalid quaternions mark orientation as unavailable according to `sensor_msgs/Imu` conventions.
- `/battery_state.percentage` ranges from 0 to 1; the raw device power field ranges from 0 to 100. Unavailable capacity fields are represented as NaN.

## DDS and device matching

Cyclone DDS is recommended:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

`MotionHighLevelClient` and `SystemRpcClientBase` write the target `device_id` into every `System.srv` request. robotServer filters RPC requests by target device SN, so only the matching device responds. This applies only to RPC and cannot isolate raw topics such as `/motion/observed`, `/sensor/observed`, and `/robotServer/Event`, because those messages currently contain no `device_id` that the bridge can filter.

Multiple bridges also collide if they publish the same global ROS names such as `/cmd_vel`, `/motion/*`, and `/odom`. A multi-robot deployment must therefore assign a separate `ROS_DOMAIN_ID` to each robot and prevent multiple bridges from appearing in one ROS graph. Different RPC `device_id` values alone do not provide multi-robot isolation.

Field boundaries:

- `device_id` is an explicit `uniubi/srv/System` field used for target-device routing.
- `Header.msg` fields `client_id` / `request_id` originate in `Request.idl`; `System.srv` does not contain this Header.

ROS 2/RMW uses the service request header to associate responses with requests. The current examples follow the server routing contract above and check the response `code` and business payload; they do not perform an additional `response.device_id` comparison.

Application code normally calls the wrapped client methods. When adding an RPC, extend the example client's wrapper layer first.

## `System.srv` call wrappers

`uniubi/srv/System.srv` is the RPC service interface used internally by the example client. Normal application development should prefer the wrappers:

- `MotionHighLevelClient`: high-level actions, control acquisition, renewal, and release.
- `SystemRpcClientBase`: shared `service` / `method` / `params` request construction, timeout handling, and response waiting.

New or extended ROS 2 examples should use these entry points so that request `device_id` and timeout handling remain consistent. For `.srv` / `.msg` fields, use the matching interface package published by `uniubi_robot_msgs` as the source of truth.

## High Level actions are asynchronous

A successful `startAction` or `stopAction` RPC means that the robot accepted the request, not that the physical motion has finished.

Recommended shutdown sequence for motion tests:

1. Send `stopAction`.
2. Send `startAction("laying")` or an equivalent laying RPC.
3. Poll `queryMotionState` until it returns an empty object (`{}`) or contains `"action":"laying"`.
4. Confirm that the robot has reached a safe state before releasing control.

## Adding an audio URL is asynchronous

`addAudioFile` may only mean that the download job was queued. When adding audio by URL, poll `queryAudioPlayList` with `{"type":"customVoice"}` until the uploaded `id` appears before playing or deleting it.

## Safety gate

Require explicit operator confirmation before high-risk movement on real hardware. Treat walking with velocity parameters, `move`, `bipedStand`, `handstand`, `waveBody`, and `jump*` as high-risk actions. Emergency stop, all-zero TRC frames, audio play/pause/stop, audio add/remove, and light settings are not high-risk motion actions, but their interface ownership and parameter requirements still apply.
