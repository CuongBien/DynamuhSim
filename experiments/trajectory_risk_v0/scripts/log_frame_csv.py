#!/usr/bin/env python3
import argparse,csv,math,pathlib
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

def yaw(q): return math.atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z))
def smin(xs):
    ys=[v for v in xs if math.isfinite(v) and v>0.0]
    return min(ys) if ys else float('nan')
class Logger(Node):
    def __init__(self,out):
        super().__init__('long_traverse_csv_logger'); pathlib.Path(out).parent.mkdir(parents=True,exist_ok=True)
        self.f=open(out,'w',newline=''); self.w=csv.writer(self.f)
        self.w.writerow(['stamp_ns','robot_x','robot_y','robot_yaw','cmd_v','cmd_w','scan_min','front_min','left_min','right_min','hard_negative'])
        self.odom=None; self.cmd=None
        self.create_subscription(Odometry,'/odom',self.on_odom,20)
        self.create_subscription(TwistStamped,'/cmd_vel',self.on_cmd,20)
        self.create_subscription(LaserScan,'/scan',self.on_scan,20)
    def on_odom(self,m): self.odom=m
    def on_cmd(self,m): self.cmd=m
    def sec(self,m,lo,hi):
        vals=[]
        for i,r in enumerate(m.ranges):
            d=math.degrees(m.angle_min+i*m.angle_increment)
            if lo<=d<=hi and math.isfinite(r) and r>0.0: vals.append(r)
        return min(vals) if vals else float('nan')
    def on_scan(self,s):
        if self.odom is None or self.cmd is None: return
        p=self.odom.pose.pose.position; q=self.odom.pose.pose.orientation
        v=self.cmd.twist.linear.x; w=self.cmd.twist.angular.z
        # handles scans expressed either around [-180,180] or [0,360]
        front_vals=[]
        for i,r in enumerate(s.ranges):
            d=math.degrees(s.angle_min+i*s.angle_increment)
            dn=((d+180.0)%360.0)-180.0
            if abs(dn)<=30.0 and math.isfinite(r) and r>0.0: front_vals.append(r)
        front=min(front_vals) if front_vals else float('nan')
        left=self.sec(s,30,90); right=self.sec(s,-90,-30)
        hard=int(math.isfinite(front) and front<0.70 and abs(v)>0.03)
        ns=int(s.header.stamp.sec)*1000000000+int(s.header.stamp.nanosec)
        self.w.writerow([ns,p.x,p.y,yaw(q),v,w,smin(s.ranges),front,left,right,hard]); self.f.flush()
    def destroy_node(self):
        try: self.f.close()
        finally: super().destroy_node()
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',required=True); a=ap.parse_args()
    rclpy.init(); n=Logger(a.output)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
