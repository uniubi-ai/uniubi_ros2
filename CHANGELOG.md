# Changelog

## Unreleased

- Align firmware version compatibility with the C++ and Python SDKs: firmware `Cyvet-V1.00.003` and newer uses `main`, `Cyvet-V1.00.002` uses the matching release tag, and earlier firmware uses `Cyvet-V1.00.001`. Pin the updated C++ runtime for optional MediaBus builds.

- Keep robotServer wire interfaces in the `uniubi` package and define bridge-only
  `MotionStatus` and `StartMotionAction` interfaces in `uniubi_motion_bridge`.
- Initialize repository structure.
- Add Walk odometry subscription, controlled reset API, read-only ROS 2 example, and usage documentation.
- Add the `uniubi_motion_client` package and the first control-ownership-only motion bridge.
- Add guarded `/cmd_vel`, command watchdog, stop/emergency services, command status, and `/odom` conversion.
- Add one generic `start_action` service instead of one ROS service per preset action.
- Add dynamic motor-layout based `/joint_states`, `/imu/data`, and `/battery_state` bridge topics.
- Make `start_action` acquire control internally, keep explicit release, and decouple `/cmd_vel` from action lifecycle.
