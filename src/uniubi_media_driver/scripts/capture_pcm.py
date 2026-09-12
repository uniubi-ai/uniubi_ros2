#!/usr/bin/env python3
"""Inspect captured PCM metadata and optionally save raw audio."""
import argparse
import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from uniubi_media_driver.msg import AudioFrame

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/audio/capture')
    parser.add_argument('--seconds', type=float, default=5)
    parser.add_argument('--output', help='Optional raw s16le PCM output file')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0: parser.error('--seconds must be positive')
    rclpy.init()
    node = rclpy.create_node('uniubi_capture_pcm')
    output = open(args.output, 'wb') if args.output else None
    counts = {'frames': 0, 'bytes': 0, 'invalid': 0}
    def receive(frame):
        if (frame.sample_rate, frame.sample_format, frame.channel_count) != (16000, 16, 1):
            counts['invalid'] += 1
            return
        counts['frames'] += 1
        counts['bytes'] += len(frame.data)
        if output: output.write(bytes(frame.data))
    subscription = node.create_subscription(AudioFrame, args.topic, receive, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if output: output.close()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    print(counts)
    return 0 if counts['frames'] and not counts['invalid'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
