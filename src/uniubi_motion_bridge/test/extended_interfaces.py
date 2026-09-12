#!/usr/bin/env python3
"""Integration test against an in-process fake robot. Run in an isolated ROS domain."""
import json
import os
import signal
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from uniubi.srv import System
from uniubi.msg import SensorObserved
from uniubi_motion_bridge.srv import JsonCommand
from uniubi_motion_bridge.msg import GpsObserved, UwbObserved
from std_srvs.srv import Trigger

def main():
    rclpy.init()
    node = rclpy.create_node('extended_interface_test')
    calls = []
    reject = set()
    def rpc(req, res):
        params = json.loads(req.payload)['params']
        calls.append((req.method, params))
        out = {'method': req.method}
        if req.method == 'takeMotionControl':
            out = {'controller': 'test-token', 'rawActionId': 1, 'leaseTimeout': 60000}
        elif req.method == 'queryMotionState':
            out = {}
        res.code = 0
        res.payload = json.dumps({'result': req.method not in reject, 'params': out})
        return res
    server = node.create_service(System, '/test_robot', rpc)
    sensor_pub = node.create_publisher(SensorObserved, '/sensor/observed', qos_profile_sensor_data)
    received = {}
    subs = [node.create_subscription(GpsObserved, '/gps/observed', lambda x: received.update(gps=x), qos_profile_sensor_data),
            node.create_subscription(UwbObserved, '/uwb/observed', lambda x: received.update(uwb=x), qos_profile_sensor_data)]
    process = subprocess.Popen(['ros2', 'run', 'uniubi_motion_bridge', 'uniubi_motion_bridge_node',
        '--ros-args', '-p', 'robot_service_name:=/test_robot', '-p', 'device_id:=fake'], start_new_session=True)
    def call(name, text='{}', trigger=False):
        typ = Trigger if trigger else JsonCommand
        client = node.create_client(typ, name)
        assert client.wait_for_service(timeout_sec=10), name
        req = typ.Request()
        if not trigger: req.params_json = text
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=20)
        assert future.done(), name
        result = future.result()
        node.destroy_client(client)
        return result
    try:
        assert not call('/audio/add_file', '{"url":"http://fake/audio.wav"}').success
        assert not any(m == 'takeMotionControl' for m, _ in calls)
        assert not call('/motion/query_state', '[]').success
        for endpoint, method in [('query_system_status','getSystemStatus'),('query_state','queryMotionState'),('query_motor_layout','getMotorLayout')]:
            result = call('/motion/'+endpoint)
            assert result.success, endpoint
            assert json.loads(result.result_json) == ({} if method == 'queryMotionState' else {'method':method})
        assert call('/audio/query_play_list', '{"type":"customVoice"}').success
        assert call('/audio/query_play_detail').success
        assert call('/motion/acquire_control', trigger=True).success
        for endpoint, data in [('/audio/add_file', {'url':'http://fake/audio.wav'}),
            ('/audio/start_play', {'list':[{'id':'1'}]}),('/audio/pause_play',{}),('/audio/stop_play',{}),
            ('/audio/delete_file',{'id':'1'}),('/light/set_brightness',{'brightness':35}),
            ('/light/query_brightness',{}),('/motion/set_action_params',{'height':0.2})]:
            assert call(endpoint,json.dumps(data)).success, endpoint
        assert not call('/light/set_brightness','{"brightness":101}').success
        assert not call('/light/set_brightness','{"brightness":3.5}').success
        reject.add('addAudioFile')
        assert not call('/audio/add_file','{}').success
        msg = SensorObserved(); msg.timestamp = 123456
        msg.gps.valid = 0; msg.gps.rssi = -50
        msg.uwb.valid = 0; msg.uwb.beacon_id = 987; msg.uwb.distance = 123
        deadline = time.monotonic()+5
        while len(received)<2 and time.monotonic()<deadline:
            sensor_pub.publish(msg);rclpy.spin_once(node,timeout_sec=.1)
        assert received['gps'].device_timestamp == 123456
        assert received['gps'].data.valid == 0 and received['gps'].data.rssi == -50
        assert received['uwb'].data.beacon_id == 987 and received['uwb'].data.valid == 0
        assert received['uwb'].data.distance == 123
        assert ('setCameraLightBrightness',{'brightness':35}) in calls
        assert ('setMotionActionParams', {'params':{'height':0.2}}) in calls
        for method in ('addAudioFile','deleteAudioFile','startPlayList','stopPlayList',
                       'getAudioPlayList','getAudioPlayDetail','getCameraLightBrightness'):
            assert any(m == method for m, _ in calls), method
        assert ('stopPlayList', {'pause':True}) in calls
        assert call('/motion/release_control', trigger=True).success
        print('PASS: queries, RPC commands, control gate, JSON validation, rejection, GPS/UWB validity and beacon ID', flush=True)
    finally:
        os.killpg(process.pid, signal.SIGINT)
        end=time.monotonic()+10
        while process.poll() is None and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        if process.poll() is None:os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)
        node.destroy_node();rclpy.shutdown()
if __name__ == '__main__': main()
