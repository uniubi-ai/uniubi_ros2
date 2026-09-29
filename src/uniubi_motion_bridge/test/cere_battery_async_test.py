#!/usr/bin/env python3
"""Board-mode battery uses bounded async RPC and does not block other services."""
import json
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from uniubi.srv import System
from uniubi_motion_bridge.srv import JsonCommand


def main():
    rclpy.init()
    node = rclpy.create_node('cere_battery_async_test')
    group = ReentrantCallbackGroup()
    lock = threading.Lock()
    status_delay = {'seconds': 1.5}
    starts = []
    batteries = []
    methods = []

    def rpc(request, response):
        with lock:
            methods.append(request.method)
            delay = status_delay['seconds']
        params = {}
        if request.method == 'getSystemStatus':
            with lock:
                starts.append(time.monotonic())
            time.sleep(delay)
            params = {'battery': {'online': True, 'voltage': 49.5,
                                  'current': -2.0, 'temperature': 31.0, 'power': 75.0}}
        elif request.method == 'queryMotionState':
            params = {'action': '', 'lineVelocityX': 0.0,
                      'lineVelocityY': 0.0, 'velocity': 0.0}
        response.code = 0
        response.payload = json.dumps({'result': True, 'params': params})
        return response

    node.create_service(System, '/cere_battery_rpc', rpc, callback_group=group)
    node.create_subscription(BatteryState, '/battery_state',
                             lambda msg: batteries.append(msg), qos_profile_sensor_data, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    process = subprocess.Popen([
        os.environ['BRIDGE_NODE_EXECUTABLE'], '--ros-args',
        '-p', 'robot_service_name:=/cere_battery_rpc',
        '-p', 'device_id:=offline',
        '-p', 'sensor_observed_source:=cere_motion_state',
        '-p', 'publish_joint_states:=false',
    ], start_new_session=True)

    def wait(predicate, seconds=8):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            with lock:
                if predicate():
                    return
            time.sleep(.01)
        raise AssertionError('timed out waiting for condition')

    try:
        query = node.create_client(JsonCommand, '/audio/query_play_list', callback_group=group)
        assert query.wait_for_service(timeout_sec=8)
        wait(lambda: len(starts) >= 1)
        before = time.monotonic()
        future = query.call_async(JsonCommand.Request())
        wait(lambda: future.done(), 1)
        assert future.result().success and time.monotonic() - before < .35
        with lock:
            status_delay['seconds'] = .01
        wait(lambda: len(starts) >= 2, 5)
        wait(lambda: len(batteries) >= 1, 5)
        with lock:
            assert not any(m == 'setMotionObservedEnable' for m in methods), methods
            battery = batteries[-1]
        assert battery.present and battery.voltage == 49.5
        assert battery.percentage == .75 and battery.current == -2.0
        print('PASS: timed-out battery RPC did not block service; valid battery recovered; no Host observation RPC', flush=True)
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        executor.shutdown()
        thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
