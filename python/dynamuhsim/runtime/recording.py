"""Record the existing pilot telemetry schema without importing ROS or labels into a planner."""
import base64
from collections import Counter
import json
import math
import zlib


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def endpoint(pose):
    return dict(x=pose.position.x, y=pose.position.y, yaw=yaw(pose.orientation))


class TrialRecording:
    STREAMS = ("gazebo_poses", "ground_truth", "scans", "odom", "amcl", "commands", "plans",
               "local_costmaps", "feedback", "roslogs", "tf")

    def __init__(self):
        self.phase = "warmup"
        self.latest = {}
        self.counts = Counter()
        self.rows = {key: [] for key in self.STREAMS}
        self.frames = []
        self.faults = []
        self.scene_sink = None

    def simtime(self):
        clock = self.latest.get("clock")
        return clock.clock.sec + clock.clock.nanosec / 1e9 if clock is not None else 0.

    def receive(self, key, msg):
        self.counts[key] += 1
        self.latest[key] = msg
        if key == "gt":
            value = json.loads(msg.data)
            if not value.get("adapter_ok"):
                self.faults.append(value)
                raise RuntimeError("Terminal adapter fault: " + value.get("error", "unknown"))
            self.rows["ground_truth"].append(value)
            self.latest[key] = value
        elif key == "scan":
            self.rows["scans"].append(dict(time=stamp(msg), ranges=[float(v) if math.isfinite(v) else None for v in msg.ranges],
                                           angle_min=msg.angle_min, angle_increment=msg.angle_increment,
                                           range_min=msg.range_min, range_max=msg.range_max))
        elif key == "odom":
            self.rows["odom"].append(dict(time=stamp(msg), **endpoint(msg.pose.pose)))
        elif key == "amcl":
            self.rows["amcl"].append(dict(time=stamp(msg), **endpoint(msg.pose.pose), covariance=list(msg.pose.covariance)))
        elif key == "cmd":
            self.rows["commands"].append(dict(time=stamp(msg), observed_sim=self.simtime(), vx=msg.twist.linear.x,
                                              vy=msg.twist.linear.y, wz=msg.twist.angular.z, phase=self.phase))
        elif key == "plan":
            self.rows["plans"].append(dict(time=stamp(msg), poses=[endpoint(p.pose) for p in msg.poses], phase=self.phase))
        elif key == "local_costmap":
            data = base64.b64encode(zlib.compress(bytes((int(v) + 256) % 256 for v in msg.data))).decode()
            self.rows["local_costmaps"].append(dict(time=stamp(msg), frame=msg.header.frame_id,
                width=msg.info.width, height=msg.info.height, resolution=msg.info.resolution,
                origin=endpoint(msg.info.origin), data=data, phase=self.phase))
        elif key == "log" and msg.level >= 30:
            self.rows["roslogs"].append(dict(time=self.simtime(), level=msg.level, node=msg.name, text=msg.msg))

    def scene(self, msg):
        entities = {p.name: endpoint(p) for p in msg.pose
                    if p.name == "mecanum_robot" or p.name.startswith("common_") and p.name.endswith("_proxy")}
        if entities:
            self.rows["gazebo_poses"].append(dict(time=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
                                                  poses=entities, phase=self.phase))
            if self.scene_sink is not None:
                self.scene_sink(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec, entities, self.phase)

    def camera(self, msg):
        if self.phase == "active":
            self.frames.append((msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
                                msg.width, msg.height, msg.step, bytes(msg.data)))

    def feedback(self, msg):
        value = msg.feedback
        self.rows["feedback"].append(dict(time=self.simtime(), distance_remaining=value.distance_remaining,
                                           recoveries=value.number_of_recoveries))

    def write(self, out):
        for name, rows in self.rows.items():
            with (out / (name + ".jsonl")).open("w") as stream:
                for row in rows:
                    stream.write(json.dumps(row, allow_nan=False) + "\n")
        if not self.frames:
            return None
        from PIL import Image
        images = [Image.frombytes("RGB", (w, h), data, "raw", "RGB", stride).quantize(colors=96)
                  for t, w, h, stride, data in self.frames]
        times = [frame[0] for frame in self.frames]
        for index, label in [(0, "initial"), (len(images) // 2, "middle"), (len(images) - 1, "final")]:
            images[index].convert("RGB").save(out / (label + ".png"))
        durations = [max(20, round((b - a) * 250 / 10) * 10) for a, b in zip(times, times[1:])] + [250]
        images[0].save(out / "camera_preview_4x.gif", save_all=True, append_images=images[1:], duration=durations, loop=0)
        return dict(source="Live Gazebo observer RGB, not AI", frame_times=times,
                    playback_speed=4, gif_duration_ms=sum(durations))
