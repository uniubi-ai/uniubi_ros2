#!/usr/bin/env python3
"""Isolated fake RPC regression for the bridge background query deadline."""
import json,os,signal,subprocess,threading,time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from uniubi.srv import System
from uniubi_motion_bridge.srv import JsonCommand
from uniubi_motion_bridge.msg import MotionStatus

def main():
 rclpy.init();node=rclpy.create_node('async_status_test');lock=threading.Lock()
 mode={'delay':.6,'action':'initial'};starts=[];statuses=[]
 def rpc(req,res):
  if req.device_id != "fake":
   res.code=1;res.payload="{}";return res
  data={}
  if req.method=='queryMotionState':
   with lock:delay=mode['delay'];action=mode['action'];starts.append((time.monotonic(),action))
   time.sleep(delay);data={'action':action,'lineVelocityX':0.,'lineVelocityY':0.,'velocity':0.}
  if req.method=='getAudioPlayDetail':time.sleep(1.2)
  res.code=0;res.payload=json.dumps({'result':True,'params':data});return res
 group=ReentrantCallbackGroup();server=node.create_service(System,'/async_test_robot',rpc,callback_group=group)
 def status(m):
  with lock:statuses.append((time.monotonic(),m.current_action,m.last_error_message))
 sub=node.create_subscription(MotionStatus,'/motion/status',status,100,callback_group=group)
 executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node);thread=threading.Thread(target=executor.spin);thread.start()
 command=[os.environ['BRIDGE_NODE_EXECUTABLE']] if os.environ.get('BRIDGE_NODE_EXECUTABLE') else ['ros2','run','uniubi_motion_bridge','uniubi_motion_bridge_node']
 process=subprocess.Popen(command+['--ros-args','-p','robot_service_name:=/async_test_robot','-p','device_id:=fake'],start_new_session=True)
 def wait(predicate,seconds=8):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   with lock:
    if predicate():return
   time.sleep(.01)
  raise AssertionError('timed out waiting for fake test condition')
 try:
  client=node.create_client(JsonCommand,'/audio/query_play_list',callback_group=group);assert client.wait_for_service(timeout_sec=10)
  wait(lambda:len(starts)>=1 and time.monotonic()-starts[-1][0]<.2)
  before=time.monotonic();future=client.call_async(JsonCommand.Request())
  wait(lambda:future.done(),2);latency=time.monotonic()-before
  assert future.result().success and latency<.35,latency
  wait(lambda:any(s[1]=='initial' for s in statuses))
  slow=node.create_client(JsonCommand,'/audio/query_play_detail',callback_group=group)
  assert slow.wait_for_service(timeout_sec=5)
  wait(lambda:time.monotonic()-starts[-1][0]<.15)
  blocked=slow.call_async(JsonCommand.Request());wait(lambda:blocked.done(),3)
  assert blocked.result().success
  time.sleep(.15)
  with lock:
   assert not any('timed out' in s[2] for s in statuses)
   mode.update(delay=1.5,action='late')
  wait(lambda:any(action=='late' for _,action in starts))
  with lock:late_start=next(t for t,a in starts if a=='late')
  wait(lambda:any('1000 ms' in error for _,_,error in statuses),3)
  with lock:
   timeout_at=next(t for t,_,e in statuses if '1000 ms' in e)
   assert .9<=timeout_at-late_start<1.3,timeout_at-late_start
   mode.update(delay=.01,action='fresh')
  wait(lambda:any(a=='fresh' for _,a,_ in statuses))
  time.sleep(.7)
  with lock:
   assert not any(a=='late' for _,a,_ in statuses),statuses
   assert statuses[-1][1]=='fresh' and not statuses[-1][2],statuses[-1]
   initial_starts=[t for t,a in starts if a=='initial']
   assert all(b-a>=.6 for a,b in zip(initial_starts,initial_starts[1:])),initial_starts
  print(f'PASS: 600 ms response accepted; concurrent service {latency:.3f}s; fixed 1s timeout; late response ignored; recovery',flush=True)
 finally:
  os.killpg(process.pid,signal.SIGINT)
  try:process.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
  executor.shutdown();thread.join();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
