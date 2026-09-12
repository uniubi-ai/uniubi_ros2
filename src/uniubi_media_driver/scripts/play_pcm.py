#!/usr/bin/env python3
"""Publish a 16 kHz / s16le / mono file at 40 ms per frame."""
import argparse
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from uniubi_media_driver.msg import AudioFrame

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcm')
    parser.add_argument('--topic', default='/audio/playback')
    args = parser.parse_args()
    with open(args.pcm, 'rb') as source:
        pcm = source.read()
    if not pcm or len(pcm) % 2:
        parser.error('PCM must contain a nonempty even number of bytes')
    rclpy.init()
    node = rclpy.create_node('uniubi_play_pcm')
    publisher = node.create_publisher(AudioFrame, args.topic, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 5
        while rclpy.ok() and publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError('No playback subscriber; enable audio_playback on the driver')
            rclpy.spin_once(node, timeout_sec=0.05)
        chunks = [pcm[i:i+1280].ljust(1280, b'\0') for i in range(0, len(pcm), 1280)]
        chunks += [bytes(1280), bytes(1280)]
        submitted = 0
        for sequence, data in enumerate(chunks):
            if not rclpy.ok(): break
            message = AudioFrame()
            message.header.stamp = node.get_clock().now().to_msg()
            message.sample_rate, message.sample_format, message.channel_count = 16000, 16, 1
            message.sequence = sequence
            message.data = data
            publisher.publish(message)
            submitted += 1
            # Never catch up by sending a burst after scheduler delays.
            time.sleep(0.04)
        time.sleep(0.3)
        print(f'Submitted {submitted} frames; submission is not playback completion confirmation')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
