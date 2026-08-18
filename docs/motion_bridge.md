# Motion Bridge Guide

**English** | [简体中文](motion_bridge.zh-CN.md)

`uniubi_motion_bridge` is intended for typical ROS 2 application developers. Application nodes use public topics and services only; they neither call `uniubi/srv/System` directly nor link an SDK shared library.

> **Feature scope:** The bridge focuses on common motion control and selectively exposes standard ROS 2 observation interfaces for odometry, joints, IMU, and battery data. It is not a complete port of the High Level Client or low-level DDS / ROS 2 protocol. Support is defined by the topics, services, and parameters listed here. System, audio, media, raw fields, and newly added protocol capabilities not listed here may not yet be available.

## Launch

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

export ROBOT_DEVICE_ID="$(python3 -c \
  'import json; print(json.load(open("/tmp/deviceInfo"))["deviceNo"])')"

ros2 launch uniubi_motion_bridge motion_bridge.launch.py \
  device_id:="$ROBOT_DEVICE_ID"
```

`device_id` is required and must be the `deviceNo` (robot SN) from `/tmp/deviceInfo`, but it is used only for RPC routing. Raw topics such as `/motion/observed`, `/sensor/observed`, and `/robotServer/Event` currently contain no device identity that the bridge can filter. If multiple robots share one DDS Domain, observations or events from another robot may be mixed in. Assign a separate `ROS_DOMAIN_ID` to each robot. On startup, the bridge establishes a connection only and does not immediately acquire High Level motion control.

Discovering the DDS service is not treated as connection readiness. After SDK `connect()` succeeds, the bridge checks the bidirectional RPC path with the read-only, side-effect-free `getMotionCapabilities` call for up to five seconds. Each RPC waits at most 500 ms, with 200 ms between retries. Only after the first successful response does it enable motion observations and state queries and report `CONNECTED` on `/motion/status`. In the current implementation, the enable RPC completes before the raw observation subscriptions are created, so the first observation frames may be lost. This is a known implementation detail; direct DDS clients should still use the protocol's reader-first order. If the readiness check times out, the bridge does not actively disconnect the SDK; a later service request runs another bounded readiness check.

On machines with multiple network interfaces, set `CYCLONEDDS_URI` to select the interface connected to the robot.

## Services

| Name | Type | Behavior |
|---|---|---|
| `/motion/start_action` | `uniubi_motion_bridge/srv/StartMotionAction` | Acquires control if necessary, then starts the requested action |
| `/motion/stop_action` | `std_srvs/srv/Trigger` | Sends `stopAction` while retaining and renewing control |
| `/motion/release_control` | `std_srvs/srv/Trigger` | Stops the action, then releases control; repeated calls are idempotent |
| `/motion/emergency_stop` | `std_srvs/srv/Trigger` | Sends a High Level emergency stop; control must already be held |
| `/motion/query_capabilities` | `std_srvs/srv/Trigger` | Returns the current robot model's action and parameter capabilities as JSON |

There is no public `take_control` service. `/motion/start_action` acquires control automatically when needed.
Before acquiring the High-level RPC session for the first time, the bridge restores the built-in cerebellum
as motor-control master and waits for the asynchronous switch to settle. The action is not sent if either
step fails.

A successful service response means that the server accepted the request, not that the physical motion has finished.

## Topics

| Name | Type | QoS / rate | Description |
|---|---|---|---|
| `/motion/status` | `uniubi_motion_bridge/msg/MotionStatus` | Reliable, Transient Local, 10 Hz by default | Actual action, velocities, control state, and latest error |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Best Effort, Keep Last 1; forwarded at up to 30 Hz by default | Parameters for the current action |
| `/odom` | `nav_msgs/msg/Odometry` | Best Effort, Keep Last 1 | Valid accumulated Walk odometry |
| `/joint_states` | `sensor_msgs/msg/JointState` | Best Effort, Keep Last 1 | Joint position, velocity, and effort |
| `/imu/data` | `sensor_msgs/msg/Imu` | Sensor Data QoS | Orientation, angular velocity, and linear acceleration |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Sensor Data QoS, 1 Hz by default | Voltage, current, temperature, and charge |

## Standard control flow

```text
start bridge
→ connected
→ /motion/start_action
→ automatic takeMotionControl + automatic lease maintenance
→ use /cmd_vel or call start_action again
→ /motion/stop_action (optional)
→ /motion/release_control
→ connected
```

If `start_action` acquired control for the current call but failed to start the action, the bridge attempts to roll back and release that session. During normal shutdown, it also makes a best effort to stop the action and release control. Server lease expiry protects against abnormal termination.

## Start an action

Query the current robot's capabilities first:

```bash
ros2 service call /motion/query_capabilities std_srvs/srv/Trigger '{}'
```

Then call the unified action interface:

```bash
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: walking, params_json: '{\"lineVelocityX\":0.0,\"lineVelocityY\":0.0,\"velocity\":0.0}'}"
```

The bridge does not define separate services for standing, laying, walking, and other actions. The server capability list defines available action names and parameter ranges, and the server performs validation. After control acquisition, the bridge forwards the requested action directly; it does not automatically insert the recommended zero-velocity `walking` transition or poll `current_action` for the caller.

For the first High Level hardware action check, use the all-zero `walking` request above, then confirm `current_action` and all three velocity fields on `/motion/status`. Do not rely on an empty JSON object to imply zero defaults.
`standing` cannot be triggered directly from the `laying` state. Treat `/motion/query_capabilities` as authoritative for available actions and transitions.

## `/cmd_vel`

Field mappings:

| ROS 2 field | High Level action parameter |
|---|---|
| `linear.x` | `lineVelocityX` |
| `linear.y` | `lineVelocityY` |
| `angular.z` | `velocity` |

`linear.y` and Uniubi `lineVelocityY` both use positive-left/negative-right. The bridge does not invert the sign, and `/motion/status.line_velocity_y` follows the same convention.

The bridge does not infer the current action from `/motion/status` or clamp values to action-specific ranges. It passes the three finite values directly to `setActionParams`; the client/server handles authorization, action-parameter matching, and safety limits. Therefore `/cmd_vel`:

- Does not acquire control.
- Does not start or switch actions.
- Does not return a response for each message; the latest failure appears in `/motion/status`.
- Retains only the latest high-rate message and limits RPC delivery using `cmd_vel_rate_hz`.

If no new message arrives within `cmd_vel_timeout_ms`, the bridge sends one parameter set with all three velocity fields at zero. It does not call `stopAction` or release control. The server independently protects against control-frame timeouts.

## `/motion/status`

The message contains:

```text
stamp
control_state
current_action
line_velocity_x
line_velocity_y
angular_velocity
last_error_code
last_error_message
```

State has two update sources:

- The bridge calls the read-only `queryMotionState` at `motion_status_rate_hz` (10 Hz by default) to update actual action and velocity.
- Internal `/robotServer/Event` handling updates control state and errors immediately for events such as control preemption.

Raw Event JSON is not published; unknown events are written to DEBUG logs only. The 10 Hz state is a snapshot that can lag by one query period and does not guarantee that every intermediate action shorter than 100 ms is recorded.

## `stop_action` semantics

`stop_action` does not mean “switch to standing,” and it does not release control.

For every action exposed by the current capabilities, `stop_action` runs the action's stop/finalization path and returns the effective action to `walking` with all three walking velocities set to zero. Control is retained and renewed. The transition is asynchronous; use `/motion/status.current_action` and the velocity fields as the source of truth.

Starting `walking` explicitly with full zero parameters is the other supported way to end the current action and enter the zero-speed walking state:

```bash
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: walking, params_json: '{\"lineVelocityX\":0.0,\"lineVelocityY\":0.0,\"velocity\":0.0}'}"
```

`/cmd_vel` updates the mapped velocity parameters of the current action when that action exposes them
(for example, `walking`, `bipedStand`, or `handstand`); it does not switch actions. It cannot stop an
action by itself.

To explicitly request standing:

```bash
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: standing, params_json: '{}'}"
```

## Observation interfaces

`/joint_states`, `/imu/data`, and `/battery_state` come from raw `/motion/observed` and do not require motion control ownership.

- `/joint_states` prefers the server motor layout. If the current firmware lacks the layout RPC, a robot-model fallback can be configured.
- `JointState.effort` uses device motor torque.
- `/imu/data` is not published when acceleration or angular velocity is invalid. If the quaternion is invalid, `orientation_covariance[0]` is set to `-1`.
- `BatteryState.percentage` converts the device's 0–100 charge value to the ROS 2 range 0–1.
- `/odom` uses the device's accumulated position and yaw. Consumers must not integrate it again. TF is not currently published. `position.y` and `twist.linear.y` remain positive-left/negative-right without conversion.

Use raw `/motion/observed` for complete fault codes, online state, and temperature. Use `/sensor/observed.odom` for odometry lifecycle fields.

## Main parameters

| Parameter | Default | Description |
|---|---|---|
| `device_id` | empty | Target robot `deviceNo` / SN; required |
| `lease_ms` | `60000` | Requested lease when acquiring control |
| `auto_connect` | `true` | Whether to connect to robotServer automatically at startup |
| `cmd_vel_timeout_ms` | `500` | ROS 2 velocity-input timeout |
| `cmd_vel_rate_hz` | `30.0` | Maximum parameter-RPC rate; upstream publishers may run faster, but only the latest value is forwarded |
| `motion_status_rate_hz` | `10.0` | Actual motion-state query rate |
| `battery_publish_rate_hz` | `1.0` | Battery-state publication rate |

See `src/uniubi_motion_bridge/config/motion_bridge.yaml` for the remaining topic, frame, and motor-layout parameters.
