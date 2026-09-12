# Extended ROS 2 interfaces

These services use the native `uniubi_motion_client` RPC implementation, without SDK shared libraries. No device firmware or `uniubi_robot_msgs` changes are required for this extension; deployed protocol versions must match.

All services below use `uniubi_motion_bridge/srv/JsonCommand`: `params_json` must be a JSON object (empty means `{}`). Responses contain `success`, `error_code`, `message`, and query data in `result_json`. Fields follow the device RPC contract.

| Service | Input/result | Control required |
|---|---|---|
| `/motion/query_system_status` | `{}` → full system status | No |
| `/motion/query_state` | `{}` → motion state, possibly `{}` | No |
| `/motion/query_motor_layout` | `{}` → motor layout | No |
| `/motion/set_action_params` | Complete parameters for the current action | Yes |
| `/audio/query_play_list` | e.g. `{"type":"customVoice"}` → file list | No |
| `/audio/query_play_detail` | `{}` → playback detail | No |
| `/audio/add_file` | Device `addAudioFile` parameters; 30 s timeout | Yes |
| `/audio/delete_file` | e.g. `{"id":"1"}` | Yes |
| `/audio/start_play` | e.g. `{"list":[{"id":"1"}],"volume":20,"repeat":1}` | Yes |
| `/audio/pause_play` | `{}` | Yes |
| `/audio/stop_play` | `{}` | Yes |
| `/light/query_brightness` | `{}` → brightness; matches SDK control requirement | Yes |
| `/light/set_brightness` | `{"brightness":20}`, integer 0–100 | Yes |

File playback is separate from the media driver's PCM `audio/playback` and `audio/reset`. `add_file` does not upload files from the ROS host; supply resources according to the device API contract.

## Ownership and action semantics

New controlled services never acquire control implicitly. Call `/motion/acquire_control` (`std_srvs/srv/Trigger`) explicitly, then `/motion/release_control` when finished. Acquisition restores the cerebellum as master; release may stop the current action. These are motion-control lifecycle operations even when requested for a light or file operation. Existing `start_action` retains automatic acquisition.

`set_action_params` updates the current action without starting or switching it. Parameters use the device's complete-replacement semantics, names and units. On success it clears older pending `/cmd_vel` commands and their watchdog state. Service commands persist like SDK commands; they do not expire on the `/cmd_vel` timeout. A subsequent `/cmd_vel` takes over velocity and restores its watchdog. Do not mix command sources.

```bash
ros2 service call /motion/query_system_status uniubi_motion_bridge/srv/JsonCommand '{}'
ros2 service call /audio/query_play_list uniubi_motion_bridge/srv/JsonCommand \
  '{params_json: "{\"type\":\"customVoice\"}"}'
# Acquisition switches the motion master; use only when appropriate for the robot.
ros2 service call /motion/acquire_control std_srvs/srv/Trigger '{}'
ros2 service call /light/set_brightness uniubi_motion_bridge/srv/JsonCommand \
  '{params_json: "{\"brightness\":20}"}'
ros2 service call /motion/release_control std_srvs/srv/Trigger '{}'
```

## GPS and UWB

Relative, remappable topics `gps/observed` and `uwb/observed` use `GpsObserved` and `UwbObserved` from `uniubi_motion_bridge`, with SensorDataQoS. Each contains ROS receipt `stamp`, unchanged `device_timestamp`, and original `uniubi/GPSFrame` or `uniubi/UWBRawObserved` in `data`, including `beacon_id`.

Invalid observations are published too: consumers must inspect `data.valid`. Receipt does not prove GPS fix or UWB pairing. No device-clock conversion, raw-unit interpretation, NavSatFix conversion or UWB Cartesian reconstruction is performed.

## Integration test

After building and sourcing `uniubi`, `uniubi_motion_client`, and `uniubi_motion_bridge`:

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 src/uniubi_motion_bridge/test/extended_interfaces.py
```

The test starts a fake System server and the real bridge. It checks ownership gates, RPC mapping, JSON validation, device rejection and GPS/UWB forwarding. It does not connect to a robot or establish hardware validation of these new interfaces.

## Three-platform hardware validation (2026-09-12)

With `8b6014f`, the local brain (Domain 1 / cerebellumServer), x86 host and ARM64 host (Domain 42 / robotServer) passed status queries, light query/set/restore, URL audio-file addition/list/play/pause/resume/stop/delete, and zero-speed walking → parameter update → laying → release. Final queries remained laying and test nodes exited normally. Release behavior was unchanged.

Poll for walking before updating action parameters, and for laying before releasing control. Request acceptance and a fixed sleep do not prove state arrival.

URL addition requires `id`, `name`, and `url`; the WAV test also used `wav:true`. Acceptance is asynchronous: poll the file list before playing. The local `file` path request was rejected in this test and is not validated.

Both external hosts received GPS/UWB messages with `valid=0`, not valid positioning. The brain Domain 1 test received none: `/sensor/observed` had a subscriber but no discovered publisher. The local brain GPS/UWB path is therefore not validated. Native SDK local observations use a different data path.

Background motion-state polling still logged timeouts with its 100 ms RPC deadline; explicit `/motion/query_state` calls and final state checks succeeded. Background polling stability requires separate work. These results establish interface/state behavior; physical output of the new file-playback tests still needs on-site confirmation.
