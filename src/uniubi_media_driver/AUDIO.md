# ROS 2 PCM audio

`uniubi_media_driver_node` shares one SDK connection for local video and audio. Audio connects a High-level client without acquiring motion control or switching the motion owner.

| Deployment | Runtime directory | Support |
|---|---|---|
| Robot brain | `lib/aarch64` | Local PCM capture/RawBack playback, alongside video |
| x86 Linux host | `lib/x86_64` | Remote capture/playback through the robot IP |
| ARM64 Linux host | `lib/aarch64_host` | Remote audio; explicitly build with `PLATFORM=aarch64_host` |

## Build

Use the updated C++ SDK with `createAudioRawBack`. The new audio message belongs to `uniubi_media_driver`; no changes to `uniubi_robot_msgs` are needed.

```bash
source /opt/ros/humble/setup.bash
# x86 or native brain. Add -DPLATFORM=aarch64_host for a generic ARM64 host.
cmake -S ~/uniubi_robot_sdk -B /tmp/sdk-install-build -DBUILD_SDK_CPP_EXAMPLES=OFF
cmake --install /tmp/sdk-install-build --prefix "$HOME/uniubi_sdk_install"
cd ~/ros2_ws
colcon build --packages-select uniubi_media_driver --cmake-args \
  -DCMAKE_PREFIX_PATH="$HOME/uniubi_sdk_install"
# Also add -DPLATFORM=aarch64_host to colcon's CMake arguments on generic ARM64 hosts.
source install/setup.bash
export LD_LIBRARY_PATH="$HOME/uniubi_sdk_install/lib/x86_64:${LD_LIBRARY_PATH:-}"
# Use lib/aarch64 on the brain, lib/aarch64_host on generic ARM64 hosts.
```

ROS 2 uses its own DDS/RMW dependencies. Deliver a complete matching SDK runtime set. Use new build directories when changing platforms.

## Run

Local brain deployment retains the existing SDK configuration and shared-memory permissions:

```bash
ros2 launch uniubi_media_driver audio_driver.launch.py
```

On an external x86/ARM64 host, substitute the robot IP, device ID and actual network interface:

```bash
ros2 launch uniubi_media_driver audio_driver.launch.py \
  host:=192.168.43.23 device_id:=YOUR_DEVICE_ID network_interface:=eth0
```

The audio configuration enables capture channel 0 and playback at volume 20, with video disabled. For capture only, copy `config/audio_driver.yaml`, set `audio_playback: false`, and pass `config_file:=/absolute/path/audio.yaml`. The existing video launch remains video-only. Enable both media types in one YAML to run them together locally. Remote mode does not support video or layout queries; `enable_video=true` is rejected.

## Interfaces

| Interface | Type | Behavior |
|---|---|---|
| `audio/capture` | `uniubi_media_driver/msg/AudioFrame` | Published frames; best effort, volatile, depth 5 |
| `audio/playback` | Same | Playback input; best effort, volatile, depth 2 |
| `audio/reset` | `std_srvs/srv/Trigger` | Clear driver and SDK playback buffers, retaining the stream |
| `audio_volume` | Integer parameter 0–100 | Dynamic RawBack stream volume; does not change hardware master volume |

Names are relative and support namespaces/remapping. Restart to change `audio_capture`, `audio_playback`, `audio_channel`, `audio_frame_id`, `host`, `device_id`, `network_interface` or `enable_video`.

Audio messages carry ROS receipt/publication time, device timestamp in microseconds, sequence, sample rate, bit width, channel count, capture channel and bytes. Device timestamps are not ROS time; different devices/backends may have different clock origins.

Playback accepts only **16000 Hz / signed 16-bit little-endian / mono**, exactly 1280 bytes per message (40 ms). There is no decoding or resampling. Publishers must pace at one frame per 40 ms. The node sends at 40 ms intervals and holds at most two pending frames; invalid format, full queue, disconnection and write failures cause logged drops rather than replaying a backlog. SDK playback timestamps come from the sender's monotonic clock; input device timestamps and capture-channel fields are not used for playback.

## Examples

```bash
# Statistics only; add --output capture.pcm to save microphone PCM.
ros2 run uniubi_media_driver capture_pcm.py --seconds 5
# Input must already be 16kHz/s16le/mono. Pads the final frame and adds two silence frames.
ros2 run uniubi_media_driver play_pcm.py input.pcm
ros2 param set /uniubi_media_driver audio_volume 20
# Stop the publisher first: subsequent ROS messages will continue playback after reset.
ros2 service call /audio/reset std_srvs/srv/Trigger '{}'
```

Submission is not playback-completion confirmation; verify physical output on site. Successful capture subscription also does not prove frames arrived: use the statistics example. Shutdown stops capture callbacks and RawBack before closing media and SDK connections.
