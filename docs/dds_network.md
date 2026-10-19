# DDS network configuration

[简体中文](dds_network.zh-CN.md)

These Host Domain 42 profiles target firmware with the matching DDS update. They do not change the on-board Domain 1 profile. Use `rmw_cyclonedds_cpp` with matching robot message definitions. An official CycloneDDS version is also an option after interoperability, RMW ABI/dependency, QoS, and actual data tests; the newest release is not automatically validated. Do not overwrite `/opt/ros` libraries or upgrade the entire ROS distribution just for this update. Package matching DDS and, when necessary, the RMW for the same ROS distribution and CPU architecture in an independent overlay.

## Ordinary Ethernet/Wi-Fi LAN

Save as `dds-public.xml`, replacing the interface with the client interface. Multicast discovery uses no peers; select the robot by SN.

```xml
<CycloneDDS><Domain Id="42"><General><Interfaces><NetworkInterface name="REPLACE_WITH_ROBOT_NIC"/></Interfaces><AllowMulticast>true</AllowMulticast><DontRoute>false</DontRoute></General><Discovery><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>
```

## Dedicated direct access

Save as `dds-direct.xml`, replacing the interface. Assign a unique client address in `172.29.140.2`–`172.29.140.254/24`; `.1` belongs to the robot and `.0`/`.255` cannot be client addresses. DDS-only access needs no default gateway or DNS. A physical cable using ordinary Ethernet addresses follows the LAN profile instead.

```xml
<CycloneDDS><Domain Id="42"><General><Interfaces><NetworkInterface name="REPLACE_WITH_WIRED_NIC"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="172.29.140.1:17910"/></Peers></Discovery></Domain></CycloneDDS>
```

ROS 2 can disable multicast through XML; this differs from SDK-generated XML, which currently uses `AllowMulticast=true`. ROS 2 does not call SDK `setNetworkConfig()`. The direct profile omits `DontRoute`, whose default is `false`; it is not a direct-mode switch. Earlier `.2`/`.3`/`.4` checks used `DontRoute=true`, so this simplified profile needs new real regression tests. Do not copy the robot-only `DirectDiscovery` extension into client XML. The peer is an address, not a CIDR range or access whitelist. `17910` is the robot discovery unicast port, not the only DDS data port.

## Load and verify

```bash
source /opt/ros/humble/setup.bash
# source the validated DDS/RMW overlay, then the message/application workspace
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///absolute/path/dds-public.xml
ros2 daemon stop
ros2 topic list --no-daemon --spin-time 5
```

Use `dds-direct.xml` for dedicated direct access and restart your ROS nodes after changing configuration. Record DDS source commit/patches, architecture, RMW version, library SHA256/dependencies, and actual loaded paths. After launching the real node, inspect `libddsc` in `/proc/PID/maps`; setting `LD_LIBRARY_PATH` alone is insufficient.

Verify client IP and `ip route get 172.29.140.1` for direct access. Check discovery, then subscribe to the target SN observation topics and verify actual data, timestamps, sustained delivery, QoS, reconnect, and exit. Discovery or successful SDK RPC does not establish ROS 2 data subscription acceptance. Missing data also requires checking publisher enablement, types, and QoS. Keep multi-robot SN checks separate. Do not bridge multiple dedicated links into one shared L2 network with conflicting robot `.1` addresses.

Cross-subnet or multicast-restricted networks may need explicit reachable peers, `DontRoute=false`, bidirectional routing, reachable advertised locators and UDP rules. These real-robot scenarios, full matching-DDS observation streams, and long-duration stability remain pending. This documentation update does not report new device tests.

See [Connect Peripherals](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/how-to/connect-peripherals.md) for network setup.
