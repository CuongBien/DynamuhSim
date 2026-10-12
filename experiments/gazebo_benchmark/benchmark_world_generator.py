"""
Generates patched SDF worlds and robot models for the Gazebo Harmonic Benchmark Matrix.
Supports:
- Varying N humans (0, 2, 5, 10, 24)
- Human representations (actor + cylinder, cylinder-only, actor-only)
- Sensor variations (no sensors, lidar-only, depth-only, lidar+depth)
- Depth camera resolutions & update rates (320x240@15, 320x240@30, 640x480@15, 640x480@30, 848x480@30)
- Corridor, Hospital, School worlds
"""

import os
import re
import copy
import xml.etree.ElementTree as ET


class BenchmarkWorldGenerator:
    def __init__(self, root_dir="/home/cuongbien/DynamuhSim"):
        self.root_dir = root_dir
        self.output_dir = os.path.join(root_dir, "experiments/gazebo_benchmark/generated_configs")
        os.makedirs(self.output_dir, exist_ok=True)

        self.corridor_090_path = os.path.join(root_dir, "src/custom_corridor/worlds/corridor_090.sdf")
        self.burger_model_path = os.path.join(root_dir, "src/custom_corridor/models/turtlebot3_burger/model.sdf")

    def generate_world_with_humans(
        self,
        num_humans: int = 5,
        representation: str = "actor_and_cylinder",  # "actor_and_cylinder", "cylinder_only", "actor_only"
        corridor_type: str = "090",
        seed: int = 42,
        real_time_factor: float = 1.0,
        x_start: float = 3.5,
        x_span: float = 7.0,
        out_filename: str = None,
        scenario_file: str = None,
        scenario_humans: list = None,
        **kwargs
    ) -> str:

        """
        Generates an SDF world with N humans and specified representation.
        """
        corridor_path = os.path.join(self.root_dir, f"src/custom_corridor/worlds/corridor_{corridor_type}.sdf")
        with open(corridor_path, "r") as f:
            content = f.read()

        # Update human_count in plugin
        content = re.sub(
            r"<human_count>\d+</human_count>",
            f"<human_count>{num_humans}</human_count>",
            content
        )
        content = re.sub(
            r"<robot_name>.*?</robot_name>",
            "<robot_name>mecanum_robot</robot_name>",
            content
        )
        content = re.sub(
            r"<robot_radius>.*?</robot_radius>",
            "<robot_radius>0.16</robot_radius>",
            content
        )

        # Update seed and scenario file in plugin
        if "<seed>" in content:
            content = re.sub(r"<seed>\d+</seed>", f"<seed>{seed}</seed>", content)
        else:
            content = content.replace(
                "<randomize_politeness>true</randomize_politeness>",
                f"<randomize_politeness>true</randomize_politeness>\n      <seed>{seed}</seed>"
            )

        # Add default_scenario_file if provided
        scenario_file = scenario_file or kwargs.get("scenario_file", None)
        if scenario_file:
            if "<default_scenario_file>" in content:
                content = re.sub(r"<default_scenario_file>.*?</default_scenario_file>", f"<default_scenario_file>{scenario_file}</default_scenario_file>", content)
            else:
                content = content.replace(
                    f"<seed>{seed}</seed>",
                    f"<seed>{seed}</seed>\n      <default_scenario_file>{scenario_file}</default_scenario_file>"
                )

        # Update real_time_factor if specified
        if real_time_factor is not None:
            content = re.sub(
                r"<real_time_factor>.*?</real_time_factor>",
                f"<real_time_factor>{real_time_factor}</real_time_factor>",
                content
            )

        # Remove existing human definitions
        content = re.sub(r"<!-- Human \d+:.*?-->", "", content, flags=re.DOTALL)
        content = re.sub(r"<actor name=\"human_\d+_visual\">.*?</actor>", "", content, flags=re.DOTALL)
        content = re.sub(r"<model name=\"human_\d+_proxy\">.*?</model>", "", content, flags=re.DOTALL)

        # Generate N humans
        scenario_humans = scenario_humans or kwargs.get("scenario_humans", None)
        humans_sdf = []
        y_offsets = [0.0, 0.15, -0.15, 0.20, -0.20]
        scales = [0.95, 1.02, 0.98, 1.05, 0.92]
        proxy_z_default = 0.80 if corridor_type == "090" else 0.875

        for i in range(1, num_humans + 1):
            idx_str = f"{i:02d}"
            if scenario_humans and i <= len(scenario_humans):
                sh = scenario_humans[i - 1]
                st = sh.get("start", {})
                x_pos = float(st.get("x", x_start + (i - 1) * 2.0))
                y_pos = float(st.get("y", 0.0))
                yaw_pos = float(st.get("yaw", 3.14159))
                scale = float(sh.get("scale", scales[(i - 1) % len(scales)]))
                proxy_z = float(sh.get("proxy_z", proxy_z_default))
            else:
                x_pos = x_start + (i - 1) * (x_span / max(num_humans - 1, 1))
                y_pos = y_offsets[(i - 1) % len(y_offsets)]
                yaw_pos = 3.14159
                scale = scales[(i - 1) % len(scales)]
                proxy_z = proxy_z_default

            actor_sdf = f"""
    <!-- Human {idx_str}: Walker -->
    <actor name="human_{idx_str}_visual">
      <pose>{x_pos} {y_pos} 0.0 0 0 {yaw_pos}</pose>
      <skin>
        <filename>model://human/meshes/walk_blue.dae</filename>
        <scale>{scale}</scale>
      </skin>
      <animation name="walk">
        <filename>model://human/meshes/walk_blue.dae</filename>
        <interpolate_x>false</interpolate_x>
      </animation>
      <script>
        <loop>false</loop>
        <delay_start>0</delay_start>
        <auto_start>false</auto_start>
        <trajectory id="0" type="walk" tension="0">
          <waypoint>
            <time>0</time>
            <pose>{x_pos} {y_pos} 0.0 0 0 {yaw_pos}</pose>
          </waypoint>
          <waypoint>
            <time>1</time>
            <pose>{x_pos} {y_pos} 0.0 0 0 {yaw_pos}</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>"""

            proxy_sdf = f"""
    <model name="human_{idx_str}_proxy">
      <pose>{x_pos} {y_pos} {proxy_z} 0 0 {yaw_pos}</pose>
      <link name="link">
        <gravity>false</gravity>
        <kinematic>true</kinematic>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.28</radius>
              <length>1.70</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.001</radius>
              <length>0.001</length>
            </cylinder>
          </geometry>
          <transparency>1.0</transparency>
        </visual>
      </link>
    </model>"""

            if representation == "actor_and_cylinder":
                humans_sdf.append(actor_sdf)
                humans_sdf.append(proxy_sdf)
            elif representation == "cylinder_only":
                humans_sdf.append(proxy_sdf)
            elif representation == "actor_only":
                humans_sdf.append(actor_sdf)

        # Inject humans before </world>
        injection_text = "\n".join(humans_sdf) + "\n  </world>"
        content = content.replace("</world>", injection_text)

        out_name = out_filename if out_filename else f"world_n{num_humans}_{corridor_type}_{representation}.sdf"
        out_path = os.path.join(self.output_dir, out_name)
        with open(out_path, "w") as f:
            f.write(content)
        return out_path


    def generate_robot_model(
        self,
        enable_lidar: bool = True,
        enable_depth: bool = True,
        depth_width: int = 640,
        depth_height: int = 480,
        depth_fps: int = 30,
        initial_x: float = 3.0,
        initial_y: float = 0.0,
        initial_z: float = 0.01,
    ) -> str:
        """
        Generates a patched TurtleBot3 Burger model with specific sensor configurations.
        """
        with open(self.burger_model_path, "r") as f:
            content = f.read()

        # Update root model pose so in-process reset restores canonical spawn pose
        content = re.sub(
            r"<pose>.*?</pose>",
            f"<pose>{initial_x} {initial_y} {initial_z} 0 0 0</pose>",
            content,
            count=1
        )

        # Handle LiDAR
        if not enable_lidar:
            # Comment out or disable gpu_lidar
            content = re.sub(
                r"<sensor name=\"hls_lfcd_lds\" type=\"gpu_lidar\">.*?</sensor>",
                "<!-- lidar disabled -->",
                content,
                flags=re.DOTALL
            )

        # Handle Depth camera
        if not enable_depth:
            content = re.sub(
                r"<sensor name=\"d435i_rgbd\" type=\"rgbd_camera\">.*?</sensor>",
                "<!-- depth camera disabled -->",
                content,
                flags=re.DOTALL
            )
        else:
            # Patch resolution and update rate
            content = re.sub(
                r"<sensor name=\"d435i_rgbd\" type=\"rgbd_camera\">\s*<always_on>true</always_on>\s*<visualize>true</visualize>\s*<update_rate>\d+</update_rate>",
                f"<sensor name=\"d435i_rgbd\" type=\"rgbd_camera\">\n        <always_on>true</always_on>\n        <visualize>true</visualize>\n        <update_rate>{depth_fps}</update_rate>",
                content
            )
            content = re.sub(
                r"<image>\s*<width>\d+</width>\s*<height>\d+</height>",
                f"<image>\n            <width>{depth_width}</width>\n            <height>{depth_height}</height>",
                content
            )

        out_name = f"burger_lidar{int(enable_lidar)}_depth{int(enable_depth)}_{depth_width}x{depth_height}_{depth_fps}fps.sdf"
        out_path = os.path.join(self.output_dir, out_name)
        with open(out_path, "w") as f:
            f.write(content)
        return out_path
