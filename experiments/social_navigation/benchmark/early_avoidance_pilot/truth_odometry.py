#!/usr/bin/env python3
"""Simulation-only oracle robot odometry. Never publishes people or robot commands."""
import math,threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
class TruthOdometry(Node):
    def __init__(self):
        super().__init__('simulation_truth_odometry')
        self.publisher=self.create_publisher(Odometry,'/odom',20);self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self);self.last=None;self.lock=threading.Lock()
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='odom';transform.transform.translation.x=-28.;transform.transform.rotation.w=1.;self.static.sendTransform(transform)
        self.gz=GzNode();assert self.gz.subscribe(Pose_V,'/world/social_warehouse/pose/info',self.receive)
    def receive(self,msg):
        # Deliberately discard all other entities, including all humans.
        robot=next((p for p in msg.pose if p.name=='mecanum_robot'),None)
        if robot is None:return
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nsec
        q=robot.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));x,y=robot.position.x,robot.position.y
        with self.lock:
            if self.last and stamp<=self.last[0]:return
            vx=vy=wz=0.
            if self.last:
                dt=(stamp-self.last[0])/1e9;wx=(x-self.last[1])/dt;wy=(y-self.last[2])/dt
                vx=math.cos(yaw)*wx+math.sin(yaw)*wy;vy=-math.sin(yaw)*wx+math.cos(yaw)*wy
                wz=math.atan2(math.sin(yaw-self.last[3]),math.cos(yaw-self.last[3]))/dt
            self.last=(stamp,x,y,yaw)
        message=Odometry();message.header.stamp.sec=msg.header.stamp.sec;message.header.stamp.nanosec=msg.header.stamp.nsec;message.header.frame_id='odom';message.child_frame_id='base_footprint'
        message.pose.pose.position.x=x+28.;message.pose.pose.position.y=y;message.pose.pose.orientation.z=math.sin(yaw/2);message.pose.pose.orientation.w=math.cos(yaw/2)
        message.twist.twist.linear.x=vx;message.twist.twist.linear.y=vy;message.twist.twist.angular.z=wz
        transform=TransformStamped();transform.header=message.header;transform.child_frame_id='base_footprint';transform.transform.translation.x=x+28.;transform.transform.translation.y=y;transform.transform.rotation=message.pose.pose.orientation
        self.publisher.publish(message);self.tf.sendTransform(transform)
if __name__=='__main__':
    rclpy.init();node=TruthOdometry()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
