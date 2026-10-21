#!/usr/bin/env python3
"""DDS retained-sample probe on an isolated synthetic topic; no Gazebo/Nav2."""
import argparse,json,os,sys,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
os.environ['ROS_DOMAIN_ID']='97';os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
os.environ['FASTDDS_BUILTIN_TRANSPORTS']='UDPv4'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from amcl_observer import observer_qos
rclpy.init();node=Node('isolated_amcl_qos_probe');results=[]
def spin(seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
try:
    for repeat in range(3):
        topic='/dynamuhsim_qos_probe_'+str(os.getpid())+'_'+str(repeat)
        pub=node.create_publisher(PoseWithCovarianceStamped,topic,observer_qos('transient_local'))
        message=PoseWithCovarianceStamped();message.header.frame_id='map';message.header.stamp.sec=repeat+1
        message.pose.pose.position.x=-27.;message.pose.pose.orientation.w=1.
        pub.publish(message);spin(.5)
        received={mode:[] for mode in ['sensor_data','transient_local']};subs=[]
        for mode in received:
            subs.append(node.create_subscription(PoseWithCovarianceStamped,topic,
                lambda msg,mode=mode:received[mode].append(msg.header.stamp.sec),observer_qos(mode)))
        spin(2.)
        results.append(dict(repeat=repeat,publish_count=1,published_before_subscriptions=True,received=received))
        assert received['sensor_data']==[],received
        assert received['transient_local']==[repeat+1],received
        for sub in subs:node.destroy_subscription(sub)
        node.destroy_publisher(pub)
    output=dict(status='PASS',domain=97,scope='Synthetic DDS retained-sample mechanism, not historical failure reproduction',results=results)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output))
finally:node.destroy_node();rclpy.shutdown()
