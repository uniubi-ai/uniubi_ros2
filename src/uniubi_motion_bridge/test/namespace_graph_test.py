#!/usr/bin/env python3
"""Verify root compatibility, two bridge namespaces, YAML, and explicit absolute topics."""
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rcl_interfaces.srv import GetParameters


def main():
    default_config = Path(os.environ['BRIDGE_CONFIG_FILE'])
    with tempfile.TemporaryDirectory(prefix='motion-namespace-') as directory:
        custom = Path(directory) / 'dog58.yaml'
        custom.write_text(default_config.read_text().replace('frame_prefix: ""', 'frame_prefix: yaml58').replace('cmd_vel_topic: cmd_vel', 'cmd_vel_topic: /custom/dog58_cmd_vel').replace('auto_connect: true', 'auto_connect: false'))
        no_connect = Path(directory) / 'default.yaml'
        no_connect.write_text(default_config.read_text().replace('auto_connect: true', 'auto_connect: false'))
        launches = [
            ('dog57', no_connect, 'dog57'),
            ('dog58', custom, ''),
            ('', no_connect, ''),
        ]
        processes = []
        try:
            for namespace, config, prefix in launches:
                args = ['ros2', 'launch', 'uniubi_motion_bridge', 'motion_bridge.launch.py',
                        'config_file:=' + str(config), 'device_id:=' + (namespace or 'root')]
                if namespace:
                    args.append('namespace:=' + namespace)
                if prefix:
                    args.append('frame_prefix:=' + prefix)
                processes.append(subprocess.Popen(args, stdout=subprocess.DEVNULL,
                                                  stderr=subprocess.PIPE, start_new_session=True))
            rclpy.init()
            node = rclpy.create_node('namespace_graph_test')
            wanted_services = {'/motion/start_action', '/dog57/motion/start_action',
                               '/dog58/motion/start_action', '/dog57/audio/query_play_list',
                               '/dog58/light/set_brightness', '/dog57/motion/query_state',
                               '/dog58/motion/acquire_control'}
            wanted_topics = {'/motion/status', '/cmd_vel', '/gps/observed', '/uwb/observed',
                             '/dog57/motion/status', '/dog57/cmd_vel', '/dog57/odom',
                             '/dog57/joint_states', '/dog57/imu/data', '/dog57/battery_state',
                             '/dog57/gps/observed', '/dog57/uwb/observed',
                             '/dog58/motion/status', '/dog58/odom', '/custom/dog58_cmd_vel'}
            end = time.monotonic() + 20
            while time.monotonic() < end:
                services = {n for n, _ in node.get_service_names_and_types()}
                topics = {n for n, _ in node.get_topic_names_and_types()}
                if wanted_services <= services and wanted_topics <= topics:
                    break
                if any(p.poll() is not None for p in processes):
                    raise AssertionError('Bridge launch exited early')
                time.sleep(.1)
            else:
                raise AssertionError(f'missing services={wanted_services-services}; topics={wanted_topics-topics}')
            assert '/dog58/cmd_vel' not in topics
            assert '/dog57/audio/query_play_list' in services
            assert '/dog58/audio/query_play_list' in services
            assert '/audio/query_play_list' in services

            def params(namespace):
                base = ('/' + namespace if namespace else '') + '/uniubi_motion_bridge'
                client = node.create_client(GetParameters, base + '/get_parameters')
                assert client.wait_for_service(timeout_sec=5)
                request = GetParameters.Request()
                request.names = ['device_id', 'frame_prefix', 'cmd_vel_topic', 'odom_frame_id']
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done() and future.result()
                return [v.string_value for v in future.result().values]
            assert params('dog57') == ['dog57', 'dog57', 'cmd_vel', 'odom']
            assert params('dog58') == ['dog58', 'yaml58', '/custom/dog58_cmd_vel', 'odom']
            assert params('') == ['root', '', 'cmd_vel', 'odom']
            node.destroy_node()
            rclpy.shutdown()
            print('PASS: three bridges, root compatibility, two namespaces, YAML and absolute override', flush=True)
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in processes:
                try:
                    process.communicate(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.communicate()


if __name__ == '__main__':
    main()
