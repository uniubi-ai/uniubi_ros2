# ROS 2 Integration Modes

**English** | [简体中文](ros2_usage_modes.zh-CN.md)

Uniubi ROS 2 provides one default application-facing motion entry point and two advanced motion options. These three motion-integration modes do not depend on `librobotMotionSdk.so`, but they address different needs:

- Motion bridge: common motion control and selected standard ROS 2 observation interfaces.
- `uniubi_motion_client`: custom C++ High Level motion-control flows.
- Direct DDS / ROS 2 protocol: direct RPC, Event, data-topic, and TRC integration.

Direct protocol integration is one complete approach, not two peer-level alternatives named Direct DDS topics and Direct RPC. Raw data subscriptions and RPC control are channels of the same protocol. See [`uniubi_robot_dds_api.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.md) for the complete contract.

Camera media is intentionally outside these three motion modes. Ordinary ROS 2 camera users run the
independent board-local [`uniubi_media_driver`](../src/uniubi_media_driver/README.md). Professional
on-board perception developers use the SDK MediaBus API directly.

## Comparison

| Mode | Primary purpose | Control ownership | Advantages | Main cost |
|---|---|---|---|---|
| Motion bridge | Application control and standard observations | Bridge-managed | Simplest option with clear ROS 2 semantics | Limited to currently exposed capabilities |
| `uniubi_motion_client` | Custom C++ High Level flows | Client renews; application manages lifecycle | More complete capabilities with a shared wrapper | Executor and shutdown must be managed correctly |
| Direct DDS / ROS 2 protocol | Raw data, protocol maintenance, and cross-framework integration | Not needed for read-only data; application-managed for control | Most complete fields and closest to the wire contract | Requires understanding RPC, Event, IDL, QoS, and ownership |

Default selection:

- To control actions, send velocity, and read state: use the Motion bridge.
- If one C++ process needs capabilities not exposed by the bridge: use `uniubi_motion_client`.
- For raw fields, DDS type/QoS debugging, new protocol interfaces, or cross-framework integration: use the direct DDS / ROS 2 protocol.

## Option 1: Motion bridge

The Motion bridge focuses on common motion control and selectively converts odometry, joint, IMU, and battery observations. It is not a complete port of the High Level Client or low-level DDS / ROS 2 protocol. Treat the topics, services, and parameters in the [Motion bridge guide](motion_bridge.md) as the current feature boundary. Camera frames use the independent `uniubi_media_driver`; other system, audio, raw fields, and newly added protocol capabilities may require `uniubi_motion_client` or direct protocol access.

Application nodes interact only with these public interfaces:

```text
/motion/* services
/motion/status
/cmd_vel
/odom
/joint_states
/imu/data
/battery_state
```

The bridge is the sole High Level motion client. It manages connection, on-demand control acquisition, renewal, Event handling, and release on shutdown.

Advantages:

- Closest to conventional ROS 2 topic/service usage.
- Application nodes do not need controller tokens, leases, or internal JSON RPC details.
- Multiple application nodes share one control entry point, reducing ownership conflicts.
- Joint, IMU, battery, and odometry data use common community messages.

Trade-offs:

- Motion-focused and limited to capabilities currently exposed and converted by the bridge.
- `/motion/status` queries action and velocity at 10 Hz by default and may lag by one query period.
- `/cmd_vel` has no per-message response; the latest failure appears in `/motion/status`.

See the [Motion bridge guide](motion_bridge.md) for details.

## Option 2: `uniubi_motion_client` C++ client

`uniubi_motion_client` is a C++ wrapper compiled from this repository's source, not an SDK shared library.

Typical flow:

```text
create ROS 2 node and executor
→ register state, Event, and observation callbacks before connect
→ connect()
→ queryCapabilities()
→ startControl()
→ startAction()/setActionParams()/stopAction()
→ keep spinning; successful control calls refresh the lease, while an idle client sends renewMotionControl
→ releaseControl()
→ disconnect()
```

Advantages:

- Exposes more complete High Level motion, system, audio, and raw TRC interfaces than the bridge.
- Reuses common RPC construction, response matching, Event parsing, and lease-renewal logic.
- Suitable for embedding advanced capabilities in one controlled C++ process.

Trade-offs:

- The application must manage its executor, callback registration order, and complete shutdown flow correctly.
- `connect()` means connected, not control ownership acquired.
- Separate client processes may compete for control.
- Less natural than the bridge for Python or typical ROS 2 application nodes.

Example:

```bash
UNIUBI_TEST_ROS_DOMAIN_ID=42 \
UNIUBI_TEST_SERVICE_NAME=robotServer \
UNIUBI_TEST_EVENT_TOPIC=/robotServer/Event \
UNIUBI_TEST_DEVICE_ID=<device-id> \
ros2 run uniubi_motion_client motion_high_level_client_example
```

Real movement is disabled by default in the example. A successful build or launch does not mean that hardware motion has been validated.

## Option 3: Direct DDS / ROS 2 protocol

This mode bypasses the application wrappers in the bridge and `uniubi_motion_client` and uses the complete robotServer communication protocol directly:

```text
RPC requests/responses   queries, configuration, control ownership, and action control
Event                    device-initiated state changes
data topics              motion observed, sensor observed
TRC control topic        high-rate real-time control frames
```

A read-only application may use only the required raw data channel. Once it performs action or TRC control, it must also implement the RPC control-ownership lifecycle and Event handling correctly. Typical applications must not treat one channel as an independent alternative to the protocol as a whole.

### Raw data topics

Applications subscribe to raw data without the bridge's standard-message conversion. Common topics include:

```text
/motion/observed
/sensor/observed
```

These topics provide continuous data. Both are disabled by default. Create the reader first and then enable them with the `setMotionObservedEnable` RPC, which does not require motion control ownership.

Advantages:

- Read-only subscriptions require no control ownership; odometry is available through `SensorObserved.odom`.
- Device error codes, validity flags, raw timestamps, and lifecycle fields remain intact.
- Suitable for validating DDS discovery, message types, publication rates, and QoS.

Trade-offs:

- Uses Uniubi custom messages rather than converting everything to community-standard messages.
- Callers must configure reliability, history depth, and durability correctly.
- Raw timestamps, coordinates, and field semantics must be interpreted according to the protocol.

`/motion/trc` is a control topic, not an ordinary read-only stream. Publishing TRC directly requires an existing control session, a correct control ID, and continuous transmission. Typical applications should use the bridge or client instead.

### RPC, Event, and control

Applications call robotServer directly through `uniubi/srv/System`. This is suitable for querying capabilities or state, validating a new RPC, and determining whether a problem is in the application wrapper, client, or robotServer.

Read-only RPCs require no control ownership. For control RPCs, the caller must implement:

```text
takeMotionControl
→ retain controller/lease/rawActionId
→ renewMotionControl
→ parse /robotServer/Event
→ make control calls
→ stopMotionAction
→ releaseMotionControl
```

Advantages:

- Minimal wrapping and precise control over service, method, JSON payload, and request timing.
- New RPCs can be validated before modifying the client or bridge.
- Suitable for wire-contract debugging and protocol development.

Trade-offs:

- Callers must manage `device_id`, timeouts, return codes, and JSON schemas.
- Control RPC implementations can easily omit renewal, preemption handling, or normal release.
- The interface is close to the low-level protocol and should not be a typical application's default dependency.

The direct protocol contract is defined in [`uniubi_robot_dds_api.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.md). Typical application developers should not construct control RPCs directly.

## Read-only use

Reading state is not the same as acquiring motion control.

Typical applications should launch the bridge and subscribe to:

```text
/odom
/joint_states
/imu/data
/battery_state
/motion/status
```

Use raw direct-protocol topics only when complete raw fields are required. Neither standard bridge observations nor raw read-only subscriptions require `/motion/start_action`.

## Do not mix control entry points

A robot may have a remote controller, mobile app, and SDK-based controllers at the same time. Run only one `uniubi_motion_bridge` as the ROS 2 High Level control entry point for each deployment. Do not let multiple bridges, client examples, and direct-protocol control programs compete for ownership.

Release control explicitly when finished. Server lease expiry is a fallback for abnormal termination, not a substitute for normal cleanup.
