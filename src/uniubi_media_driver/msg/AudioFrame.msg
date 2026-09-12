# header.stamp is ROS receipt/publication time, not the device clock.
std_msgs/Header header
# SDK timestamp in microseconds; clock origin is device/backend specific.
uint64 device_timestamp_us
uint64 sequence
uint32 sample_rate
# Sample bit width; playback accepts only 16-bit signed little-endian PCM.
uint32 sample_format
uint32 channel_count
uint32 channel
# Playback requires 1280 bytes (40 ms at 16000 Hz, mono, s16le).
uint8[<=65536] data
