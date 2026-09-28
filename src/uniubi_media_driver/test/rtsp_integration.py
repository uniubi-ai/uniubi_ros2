#!/usr/bin/env python3
"""Opt-in RTSP integration test; see RTSP.md for dependencies and invocation.

Runs only loopback synthetic sources; never contacts a robot. Requires a built
RTSP node plus rclpy, GStreamer RTSP server GI, JPEG and x264 plugins, and cv2.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time


def serve(port):
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstRtspServer', '1.0')
    from gi.repository import Gst, GstRtspServer, GLib
    Gst.init(None)
    for plugin in ['videotestsrc', 'jpegenc', 'rtpjpegpay', 'x264enc', 'rtph264pay']:
        if not Gst.ElementFactory.find(plugin):
            raise RuntimeError('Missing GStreamer plugin: ' + plugin)
    server = GstRtspServer.RTSPServer()
    server.set_address('127.0.0.1')
    server.set_service(str(port))
    for name, encoder in [('camera0', 'jpegenc ! rtpjpegpay pt=26'),
                          ('camera1', 'x264enc tune=zerolatency key-int-max=15 ! rtph264pay pt=96')]:
        factory = GstRtspServer.RTSPMediaFactory()
        factory.set_launch('( videotestsrc is-live=true pattern=ball ! '
                           'video/x-raw,width=320,height=240,framerate=15/1 ! '
                           'videoconvert ! video/x-raw,format=I420 ! ' + encoder + ' name=pay0 )')
        factory.set_shared(True)
        server.get_mount_points().add_factory('/' + name, factory)
    if not server.attach(None):
        raise RuntimeError('RTSP server bind failed')
    GLib.MainLoop().run()


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def stop(proc, sig=signal.SIGTERM, timeout=4):
    if proc is None or proc.poll() is not None:
        return 0.0
    start = time.monotonic()
    proc.send_signal(sig)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        raise AssertionError('Process did not stop before timeout')
    return time.monotonic() - start


def run(args):
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CompressedImage
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    os.environ.setdefault('ROS_DOMAIN_ID', '187')
    rclpy.init()
    observer = rclpy.create_node('rtsp_integration_observer')
    counts = [0, 0, 0, 0]
    stamps = [0, 0, 0, 0]
    subscriptions = []
    errors = []

    def receive(msg, index):
        try:
            stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            assert stamp > 0 and stamp >= stamps[index], 'invalid/nonmonotonic timestamp'
            assert msg.header.frame_id == 'front_camera_' + str(index % 2) + '_optical_frame'
            if index < 2:
                assert (msg.width, msg.height, msg.encoding, msg.step) == (320, 240, 'bgr8', 960)
                assert len(msg.data) == 320 * 240 * 3
            else:
                assert 'jpeg' in msg.format
                frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                assert frame is not None and frame.shape == (240, 320, 3)
            stamps[index] = stamp
            counts[index] += 1
        except Exception as exc:
            errors.append(str(exc) or repr(exc))

    for i in range(4):
        topic = '/rtsp_test/front_camera_' + str(i % 2) + '/image_raw'
        if i >= 2:
            topic += '/compressed'
        subscriptions.append(observer.create_subscription(
            Image if i < 2 else CompressedImage, topic,
            lambda msg, index=i: receive(msg, index), qos_profile_sensor_data))

    def until(predicate, seconds, reason):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
            assert not errors, errors
            if predicate():
                return
        raise AssertionError(reason + ': counts=' + repr(counts))

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
        assert not errors, errors

    port = free_port()
    result = {}
    with tempfile.TemporaryDirectory(prefix='uniubi-rtsp-test-') as temp:
        temp = Path(temp)
        def start_server():
            proc = subprocess.Popen([sys.executable, __file__, '--serve', str(port)],
                                    stdout=open(temp/'server.log', 'a'), stderr=subprocess.STDOUT)
            time.sleep(0.8)
            assert proc.poll() is None, (temp/'server.log').read_text()
            return proc

        def start_node(urls, label):
            config = temp/(label + '.json')
            config.write_text(json.dumps({'/**': {'ros__parameters': {
                'rtsp_urls': urls,
                'camera_names': ['front_camera_0', 'front_camera_1'],
                'frame_ids': ['front_camera_0_optical_frame', 'front_camera_1_optical_frame'],
                'topic_prefix': 'rtsp_test',
                'open_timeout_ms': 1000, 'read_timeout_ms': 1000,
                'reconnect_delay_ms': 300, 'publish_compressed': True,
            }}}))
            return subprocess.Popen([args.node, '--ros-args', '--params-file', str(config)],
                                    stdout=open(temp/(label + '.log'), 'w'), stderr=subprocess.STDOUT)

        server = node = None
        blackhole = None
        halted = threading.Event()
        accepted = []
        try:
            urls = ['rtsp://127.0.0.1:' + str(port) + '/camera' + str(i) for i in range(2)]
            server = start_server()
            node = start_node(urls, 'dual')
            until(lambda: min(counts) >= 10, 25, 'dual MJPEG/H264 raw/JPEG frames missing')
            result['dual_stream_frames'] = counts.copy()
            stop(server)
            server = None
            spin(3)
            before = counts.copy()
            spin(2)
            assert counts == before, 'stale images republished after disconnect'
            server = start_server()
            until(lambda: all(a >= b + 10 for a, b in zip(counts, before)), 25, 'reconnect failed')
            result['recovered_frames'] = [a-b for a,b in zip(counts,before)]
            result['normal_shutdown_seconds'] = stop(node, signal.SIGINT)
            assert node.returncode == 0, 'normal shutdown exit status'
            node = None
            spin(1)

            blackhole = socket.socket()
            blackhole.bind(('127.0.0.1', 0))
            blackhole.listen()
            blackhole.settimeout(0.2)
            def stall():
                while not halted.is_set():
                    try:
                        conn, _ = blackhole.accept()
                        accepted.append(conn)
                    except socket.timeout:
                        pass
                    except OSError:
                        break
            thread = threading.Thread(target=stall, daemon=True)
            thread.start()
            urls[0] = 'rtsp://127.0.0.1:' + str(blackhole.getsockname()[1]) + '/stall'
            before = counts.copy()
            node = start_node(urls, 'stalled')
            until(lambda: counts[1] >= before[1] + 15 and counts[3] >= before[3] + 15,
                  20, 'stalled stream blocked healthy stream')
            assert counts[0] == before[0] and counts[2] == before[2]
            result['healthy_frames_while_other_stalled'] = counts[1] - before[1]
            assert len(accepted) >= 2, 'stalled RTSP connection did not time out/retry'
            result['stalled_open_retries'] = len(accepted)
            result['stalled_shutdown_seconds'] = stop(node, signal.SIGINT)
            assert node.returncode == 0, 'stalled shutdown exit status'
            node = None
            result['result'] = 'PASS'
        finally:
            halted.set()
            if blackhole:
                blackhole.close()
            for conn in accepted:
                conn.close()
            stop(node, signal.SIGINT)
            stop(server)
            if args.evidence:
                import shutil
                destination = Path(args.evidence)
                destination.mkdir(parents=True, exist_ok=True)
                for item in temp.glob('*.log'):
                    shutil.copy2(item, destination/item.name)
            observer.destroy_node()
            rclpy.shutdown()
    print(json.dumps(result, indent=2))
    if args.evidence:
        (Path(args.evidence)/'result.json').write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node', help='Absolute path to the built RTSP node executable')
    parser.add_argument('--evidence', help='Directory for test logs and result.json')
    parser.add_argument('--serve', type=int, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.serve:
        serve(args.serve)
    elif args.node:
        run(args)
    else:
        parser.error('--node is required')
