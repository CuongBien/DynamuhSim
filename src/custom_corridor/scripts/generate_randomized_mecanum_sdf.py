#!/usr/bin/env python3
"""
Randomized Mecanum Robot SDF Generator for Gazebo Harmonic.
Generates an episode-specific SDF with narrow randomization as specified:
- Chassis dimensions: 0.24 x 0.20 m +/- 20%
- Wheelbase & wheel separation scaled accordingly
- Max velocity: vx in [0.55, 0.65] m/s, vy in [0.35, 0.45] m/s, w in [1.60, 2.00] rad/s
- D435i Camera: height in [0.075, 0.085] m, subtle pitch/yaw jitter, 424x240 @ 10Hz, 87 deg FOV, clip [0.28, 5.0] m
- 2D LiDAR: height in [0.115, 0.125] m, 5 Hz
- Circumscribed robot radius for RVO2: hypot(L/2, W/2) approx 0.16 m
Logs all sampled parameters for manifest.
"""

import math
import numpy as np
from typing import Dict, Tuple


def generate_randomized_mecanum_sdf(seed: int = None, randomize: bool = True) -> Tuple[str, Dict]:
    rng = np.random.RandomState(seed) if seed is not None else np.random.RandomState()
    
    if randomize:
        # Chassis: 0.24 x 0.20 m +/- 20%
        length = float(0.24 * rng.uniform(0.90, 1.10)) # +/- 10% narrow randomization
        width = float(0.20 * rng.uniform(0.90, 1.10))
        height = float(0.08 * rng.uniform(0.95, 1.05))
        
        # Max velocity within narrow range
        vx_max = float(rng.uniform(0.55, 0.65))
        vy_max = float(rng.uniform(0.35, 0.45))
        w_max = float(rng.uniform(1.60, 2.00))
        
        # Camera placement: height [0.075, 0.085], subtle tilt
        cam_x = float(length / 2.0)
        cam_z = float(rng.uniform(0.076, 0.084))
        cam_pitch = float(math.radians(rng.uniform(-2.0, 2.0)))
        cam_yaw = float(math.radians(rng.uniform(-1.0, 1.0)))
        
        # LiDAR placement
        lidar_z = float(rng.uniform(0.115, 0.125))
    else:
        length = 0.24
        width = 0.20
        height = 0.08
        vx_max = 0.60
        vy_max = 0.40
        w_max = 1.80
        cam_x = 0.12
        cam_z = 0.08
        cam_pitch = 0.0
        cam_yaw = 0.0
        lidar_z = 0.12

    wheel_radius = 0.04
    wheelbase = length * (0.16 / 0.24)
    wheel_separation = width
    rvo2_radius = math.hypot(length / 2.0, width / 2.0)
    
    manifest_params = {
        "chassis_length_m": round(length, 4),
        "chassis_width_m": round(width, 4),
        "chassis_height_m": round(height, 4),
        "wheelbase_m": round(wheelbase, 4),
        "wheel_separation_m": round(wheel_separation, 4),
        "wheel_radius_m": wheel_radius,
        "rvo2_neighbor_radius_m": round(rvo2_radius, 4),
        "vx_max_mps": round(vx_max, 3),
        "vy_max_mps": round(vy_max, 3),
        "w_max_radps": round(w_max, 3),
        "camera_x_m": round(cam_x, 4),
        "camera_z_m": round(cam_z, 4),
        "camera_pitch_deg": round(math.degrees(cam_pitch), 3),
        "camera_yaw_deg": round(math.degrees(cam_yaw), 3),
        "lidar_z_m": round(lidar_z, 4),
        "depth_camera_resolution": [424, 240],
        "depth_camera_fps": 10,
        "depth_camera_hfov_deg": 87.0,
        "depth_camera_min_range_m": 0.28,
        "depth_camera_max_range_m": 5.0,
        "lidar_fps": 5
    }

    fl_x, fl_y = wheelbase / 2.0, wheel_separation / 2.0
    fr_x, fr_y = wheelbase / 2.0, -wheel_separation / 2.0
    rl_x, rl_y = -wheelbase / 2.0, wheel_separation / 2.0
    rr_x, rr_y = -wheelbase / 2.0, -wheel_separation / 2.0

    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="mecanum_robot">
    <pose>0.0 0.0 {wheel_radius} 0.0 0.0 0.0</pose>

    <!-- Base footprint (ground projection) -->
    <link name="base_footprint"/>

    <!-- Main Chassis Link -->
    <link name="base_link">
      <inertial>
        <pose>0.0 0.0 {height/2.0:.4f} 0 0 0</pose>
        <mass>3.5</mass>
        <inertia>
          <ixx>0.01598</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.02473</iyy><iyz>0.0</iyz><izz>0.03700</izz>
        </inertia>
      </inertial>

      <collision name="chassis_collision">
        <pose>0.0 0.0 {height/2.0:.4f} 0 0 0</pose>
        <geometry>
          <box>
            <size>{length:.4f} {width:.4f} {height:.4f}</size>
          </box>
        </geometry>
      </collision>

      <visual name="chassis_visual">
        <pose>0.0 0.0 {height/2.0:.4f} 0 0 0</pose>
        <geometry>
          <box>
            <size>{length:.4f} {width:.4f} {height:.4f}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.15 0.25 0.45 1.0</ambient>
          <diffuse>0.20 0.35 0.65 1.0</diffuse>
          <specular>0.4 0.4 0.5 1.0</specular>
        </material>
      </visual>

      <visual name="top_plate_visual">
        <pose>0.0 0.0 {height + 0.005:.4f} 0 0 0</pose>
        <geometry>
          <box>
            <size>{length - 0.02:.4f} {width - 0.02:.4f} 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1.0</ambient>
          <diffuse>0.25 0.25 0.25 1.0</diffuse>
        </material>
      </visual>
    </link>

    <!-- IMU Link & Sensor -->
    <link name="imu_link">
      <pose relative_to="base_link">0.0 0.0 0.06 0 0 0</pose>
      <inertial>
        <mass>0.02</mass>
        <inertia>
          <ixx>1e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-5</iyy><iyz>0</iyz><izz>1e-5</izz>
        </inertia>
      </inertial>
      <sensor name="imu" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <topic>imu</topic>
        <gz_frame_id>imu_link</gz_frame_id>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></x>
            <y><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></y>
            <z><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></x>
            <y><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></y>
            <z><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>
    </link>

    <!-- 2D LiDAR (5 Hz) -->
    <link name="base_scan">
      <pose relative_to="base_link">0.0 0.0 {lidar_z:.4f} 0 0 0</pose>
      <inertial>
        <mass>0.12</mass>
        <inertia>
          <ixx>4e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>4e-5</iyy><iyz>0</iyz><izz>4e-5</izz>
        </inertia>
      </inertial>
      <collision name="lidar_collision">
        <geometry>
          <cylinder>
            <radius>0.035</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="lidar_visual">
        <geometry>
          <cylinder>
            <radius>0.035</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1.0</ambient>
          <diffuse>0.1 0.1 0.1 1.0</diffuse>
        </material>
      </visual>
      <sensor name="lidar" type="gpu_lidar">
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>5</update_rate>
        <topic>scan</topic>
        <gz_frame_id>base_scan</gz_frame_id>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>0.0</min_angle>
              <max_angle>6.283185307</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.12</min>
            <max>3.5</max>
            <resolution>0.015</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </lidar>
      </sensor>
    </link>

    <!-- D435i Front Depth Camera Link & Sensor -->
    <link name="camera_link">
      <pose relative_to="base_link">{cam_x:.4f} 0.0 {cam_z:.4f} 0 {cam_pitch:.4f} {cam_yaw:.4f}</pose>
      <inertial>
        <mass>0.06</mass>
        <inertia>
          <ixx>1e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-5</iyy><iyz>0</iyz><izz>1e-5</izz>
        </inertia>
      </inertial>
      <visual name="camera_visual">
        <geometry>
          <box>
            <size>0.025 0.09 0.025</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1.0</ambient>
          <diffuse>0.3 0.3 0.3 1.0</diffuse>
        </material>
      </visual>
      <sensor name="depth_camera" type="depth_camera">
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>10</update_rate>
        <topic>camera/depth_image</topic>
        <gz_frame_id>camera_link</gz_frame_id>
        <camera>
          <camera_info_topic>camera/camera_info</camera_info_topic>
          <horizontal_fov>1.518436</horizontal_fov>
          <image>
            <width>424</width>
            <height>240</height>
            <format>R_FLOAT32</format>
          </image>
          <clip>
            <near>0.28</near>
            <far>5.00</far>
          </clip>
        </camera>
      </sensor>
    </link>

    <!-- Wheels (FL, FR, RL, RR) -->
    <!-- FL Wheel -->
    <link name="front_left_wheel">
      <pose relative_to="base_link">{fl_x:.4f} {fl_y:.4f} 0.0 -1.5707963 0 0</pose>
      <inertial>
        <mass>0.25</mass>
        <inertia>
          <ixx>0.00012</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00012</iyy><iyz>0</iyz><izz>0.00020</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere><radius>{wheel_radius}</radius></sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu><mu2>0.0</mu2>
              <fdir1 gz:expressed_in="base_link">1 -1 0</fdir1>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>{wheel_radius}</radius><length>0.03</length></cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1.0</ambient><diffuse>0.2 0.2 0.2 1.0</diffuse>
        </material>
      </visual>
    </link>

    <!-- FR Wheel -->
    <link name="front_right_wheel">
      <pose relative_to="base_link">{fr_x:.4f} {fr_y:.4f} 0.0 -1.5707963 0 0</pose>
      <inertial>
        <mass>0.25</mass>
        <inertia>
          <ixx>0.00012</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00012</iyy><iyz>0</iyz><izz>0.00020</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere><radius>{wheel_radius}</radius></sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu><mu2>0.0</mu2>
              <fdir1 gz:expressed_in="base_link">1 1 0</fdir1>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>{wheel_radius}</radius><length>0.03</length></cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1.0</ambient><diffuse>0.2 0.2 0.2 1.0</diffuse>
        </material>
      </visual>
    </link>

    <!-- RL Wheel -->
    <link name="rear_left_wheel">
      <pose relative_to="base_link">{rl_x:.4f} {rl_y:.4f} 0.0 -1.5707963 0 0</pose>
      <inertial>
        <mass>0.25</mass>
        <inertia>
          <ixx>0.00012</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00012</iyy><iyz>0</iyz><izz>0.00020</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere><radius>{wheel_radius}</radius></sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu><mu2>0.0</mu2>
              <fdir1 gz:expressed_in="base_link">1 1 0</fdir1>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>{wheel_radius}</radius><length>0.03</length></cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1.0</ambient><diffuse>0.2 0.2 0.2 1.0</diffuse>
        </material>
      </visual>
    </link>

    <!-- RR Wheel -->
    <link name="rear_right_wheel">
      <pose relative_to="base_link">{rr_x:.4f} {rr_y:.4f} 0.0 -1.5707963 0 0</pose>
      <inertial>
        <mass>0.25</mass>
        <inertia>
          <ixx>0.00012</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00012</iyy><iyz>0</iyz><izz>0.00020</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere><radius>{wheel_radius}</radius></sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu><mu2>0.0</mu2>
              <fdir1 gz:expressed_in="base_link">1 -1 0</fdir1>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>{wheel_radius}</radius><length>0.03</length></cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1.0</ambient><diffuse>0.2 0.2 0.2 1.0</diffuse>
        </material>
      </visual>
    </link>

    <!-- Fixed Joints -->
    <joint name="base_footprint_joint" type="fixed">
      <parent>base_footprint</parent>
      <child>base_link</child>
    </joint>

    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
    </joint>

    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>base_scan</child>
    </joint>

    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>

    <!-- Revolute Wheel Joints -->
    <joint name="front_left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>front_left_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit>
      </axis>
    </joint>

    <joint name="front_right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>front_right_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit>
      </axis>
    </joint>

    <joint name="rear_left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>rear_left_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit>
      </axis>
    </joint>

    <joint name="rear_right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>rear_right_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit>
      </axis>
    </joint>

    <!-- Native Mecanum Drive Plugin -->
    <plugin filename="gz-sim-mecanum-drive-system" name="gz::sim::systems::MecanumDrive">
      <front_left_joint>front_left_wheel_joint</front_left_joint>
      <front_right_joint>front_right_wheel_joint</front_right_joint>
      <back_left_joint>rear_left_wheel_joint</back_left_joint>
      <back_right_joint>rear_right_wheel_joint</back_right_joint>
      <wheel_separation>{wheel_separation:.4f}</wheel_separation>
      <wheelbase>{wheelbase:.4f}</wheelbase>
      <wheel_radius>{wheel_radius:.4f}</wheel_radius>
      <min_acceleration>-1.5</min_acceleration>
      <max_acceleration>1.5</max_acceleration>
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
      <odom_publish_frequency>30</odom_publish_frequency>
      <tf_topic>/tf</tf_topic>
    </plugin>

    <!-- Joint State Publisher Plugin -->
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <topic>joint_states</topic>
      <joint_name>front_left_wheel_joint</joint_name>
      <joint_name>front_right_wheel_joint</joint_name>
      <joint_name>rear_left_wheel_joint</joint_name>
      <joint_name>rear_right_wheel_joint</joint_name>
    </plugin>

  </model>
</sdf>
"""
    return sdf_content, manifest_params


if __name__ == "__main__":
    sdf_str, params = generate_randomized_mecanum_sdf(seed=42)
    print("Sample Generated Parameters:")
    for k, v in params.items():
        print(f"  {k}: {v}")
    print(f"SDF Length: {len(sdf_str)} chars")
