# RTSP video backend

This backend lives in the same `uniubi_media_driver` ROS 2 package as the MediaBus driver, but has its own executable and does not use the UniUbi SDK. It receives RTSP over TCP with FFmpeg, decodes each camera independently, and publishes BGR8 `sensor_msgs/msg/Image` on relative topics `<camera_name>/image_raw`. Set `publish_compressed: true` to also publish JPEG `sensor_msgs/msg/CompressedImage` on `<camera_name>/image_raw/compressed`, the existing MediaBus camera topic. The default camera names and optical frame IDs match MediaBus. A ROS namespace and `topic_prefix` can be used together.

## Intended use and performance costs

This backend provides standard ROS 2 image topics for quick integration, functional validation, ROS visualization, and recording workflows that need image messages. It is an optional integration path: developers can also consume the device RTSP URLs directly without this node.

| Use case | Suggested path |
|---|---|
| Quickly obtain standard ROS image topics | Use this backend; enable JPEG only when needed |
| Only view or archive video | Use an RTSP client directly; when the tool and container support it, archive the encoded stream without decoding or re-encoding |
| Low latency, high frame rates, many streams, or GPU perception | Consume RTSP directly and configure decoding, buffering, and processing for the target platform |
| SDK raw images or GPU data access on the robot's Orin brain | Consider the MediaBus SDK to avoid unnecessary CPU image conversion |

The current implementation uses FFmpeg **CPU software decoding**, converts frames into BGR8 in CPU memory, and publishes ROS messages. It does not configure a hardware decoder device or a GPU zero-copy path. Having a hardware decoder does not make this node use it automatically. Decoding, color conversion, memory copies, and ROS message transport all have costs. Two 1280x720 BGR8 streams at 25 fps contain approximately **138 MB/s** of raw image data; this is neither the encoded RTSP bitrate nor a measurement of actual network traffic or total memory bandwidth consumption.

Enabling `publish_compressed` re-encodes decoded images as JPEG, adding CPU work and latency; it is disabled by default. The MediaBus backend on the robot's Orin brain forwards existing JPEG frames without this RTSP decode/re-encode pipeline. Building both backends adds dependencies and artifacts; an unstarted backend performs no video processing.

For performance-sensitive applications, consider direct RTSP integration through FFmpeg, GStreamer, or the platform media interfaces. Where hardware decoding is available, verify support for the source codec, resolution, driver, and tool build, and explicitly configure the matching hardware decoding path. GPU perception pipelines should retain decoded frames in device memory where possible instead of downloading to the CPU and uploading again. If the application still needs CPU BGR8 ROS images, conversion, copying, and message transport costs remain after hardware decoding. Hardware decoding alone does not guarantee lower end-to-end latency.

Measure delivered frame rate, end-to-end latency, CPU/GPU utilization, and dropped frames on the target device before choosing this general-purpose wrapper or a custom pipeline. This backend does not guarantee the source's nominal frame rate, zero-copy transport, or hardware acceleration.

The default build includes both MediaBus and RTSP; choose the video backend at launch. Building both does not start both nodes. Prepare ROS 2 Humble and install/configure the robot SDK as described in the [media driver build guide](README.md#build-and-run), then install the FFmpeg development libraries (Ubuntu 22.04; the package names are the same on x86_64 and ARM64):

```bash
sudo apt install libavformat-dev libavcodec-dev \
  libavutil-dev libswscale-dev pkg-config
```

Once the dependencies are ready, build from the `uniubi_ros2` workspace without backend build flags:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select uniubi_media_driver
source install/setup.bash
ros2 launch uniubi_media_driver media_driver.launch.py video_backend:=rtsp host:=192.168.1.10
```

The launch `host` argument makes these two URLs:

```text
rtsp://192.168.1.10:554/live?channel=1&stream=0
rtsp://192.168.1.10:554/live?channel=2&stream=0
```

For other endpoints, omit `host` and put `rtsp_urls` in a copy of `config/rtsp_driver.yaml`, then pass `rtsp_config_file:=/absolute/path/to/file.yaml`. Match the lengths of `rtsp_urls`, `camera_names`, and `frame_ids`. RTSP URLs may contain credentials; keep configuration files private. The driver does not log URLs or FFmpeg diagnostic strings. `open_timeout_ms`, `read_timeout_ms`, and `reconnect_delay_ms` control endpoint failure and retry. All runtime parameters are fixed after startup.

Each decoder overwrites a single pending frame while a separate thread publishes, so a slow ROS publisher or optional JPEG encoder does not create an application frame queue. ROS SensorDataQoS is best-effort with depth 1. The application queue is bounded to one frame; network buffers, codec reordering, and downstream subscriber queues can still introduce delay. Timestamps use the ROS node clock when each frame is decoded, not a camera capture timestamp. The driver retries failed cameras separately and stops blocked RTSP operations through an interrupt callback on shutdown.

The default build includes MediaBus video, PCM audio and RTSP video. Select `video_backend:=mediabus` (default) or `video_backend:=rtsp` at launch; PCM audio continues to use `audio_driver.launch.py`. Compiling both backends does not start both nodes.

## Ethernet and Wi-Fi

Set `host` to the device interface that exposes RTSP. Ethernet and Wi-Fi are both supported;
the host routing table selects the network interface. For a direct Ethernet connection, use
the cerebellum's wired IP. An address observed after SSH login to the brain on port 22 is not
necessarily an RTSP endpoint. Check `ip route get <DEVICE_WIRED_IP>` and probe the actual
stream on port 554 before launching. This node does not modify routes or create forwarding rules.

## Advanced: minimal builds and old caches

Only disable the SDK backend when you need RTSP video without installing the robot SDK. Use separate build/install directories to avoid mixing old artifacts:

```bash
colcon build --packages-select uniubi_media_driver \
  --build-base build-rtsp --install-base install-rtsp --cmake-args \
  -DUNIUBI_MEDIA_WITH_SDK=OFF
source install-rtsp/setup.bash
```

Normal users do not need these flags: `UNIUBI_MEDIA_WITH_SDK` and `UNIUBI_MEDIA_WITH_RTSP` both default to `ON`.
If an existing build directory cached `OFF`, run
`colcon build --packages-select uniubi_media_driver --cmake-clean-cache` once to restore the full default build.

## Self-test

Run parameter tests with `colcon test --packages-select uniubi_media_driver`.
The optional loopback integration test `test/rtsp_integration.py` requires `python3-gi`,
`gir1.2-gst-rtsp-server-1.0`, GStreamer base/good/ugly plugins and `python3-opencv`.
It validates dual MJPEG/H.264 raw/JPEG topics, reconnect, a stalled peer and bounded shutdown
using synthetic streams; it never connects to a robot.

```bash
python3 src/uniubi_media_driver/test/rtsp_integration.py \
  --node "$PWD/install/uniubi_media_driver/lib/uniubi_media_driver/uniubi_rtsp_driver_node" \
  --evidence /tmp/uniubi-rtsp-integration
```
