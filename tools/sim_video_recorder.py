#!/usr/bin/env python3
"""Record a presentation video of the Gazebo simulation (for YouTube / the tutorial).

What it does:
  1. Spawns two static "film" cameras into the running world (nothing is changed on disk):
       wide  - the robot, the nuts and the bolt
       bolt  - a close-up of the bolt fixture (shown as an inset)
  2. Bridges their images to ROS and composes every frame:
       wide view + bolt inset + the robot's current stage (from its [LAUNCHER_STATUS] log)
       + a live wrist-torque bar (/wrist_ft) + the thread turn counter (/bolt_fixture/joint_states)
  3. Writes frames at a fixed rate of SIMULATION time, so motion looks real-time even when
     Gazebo runs slower than real time.
  4. Saves stage timestamps (JSON) so tools/make_sim_video.sh can speed up long parts.

Usage (ROS 2 sourced, simulation running; then start the robot task):
    python3 tools/sim_video_recorder.py --out media_and_files/videos/raw.mp4
Stops by itself a few seconds after "Tightening complete" (or --max-seconds, or Ctrl+C).
"""
import argparse
from collections import deque
import json
import math
import os
import re
import signal
import subprocess
import tempfile
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from PIL import Image as PilImage
from PIL import ImageDraw, ImageFont
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

WIDTH, HEIGHT = 1280, 720
FONT_BOLD = "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf"
FONT_REGULAR = "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf"
SNUG_TORQUE = 1.0      # N*m, the node's default tighten_snug_torque
TORQUE_SCALE = 2.0     # N*m at the right end of the torque bar


# ----------------------------------------------------------------------------- cameras
def look_at_rpy(camera, target):
    """Gazebo camera orientation (roll, pitch, yaw) that looks from `camera` at `target`.
    A Gazebo camera looks along its +x axis; positive pitch tilts it down."""
    dx, dy, dz = (t - c for t, c in zip(target, camera))
    return 0.0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)


def camera_sdf(name, topic, camera, target, hfov, rate):
    roll, pitch, yaw = look_at_rpy(camera, target)
    x, y, z = camera
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
    <link name="link">
      <sensor name="film_camera" type="camera">
        <always_on>true</always_on>
        <update_rate>{rate}</update_rate>
        <topic>{topic}</topic>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
          <clip><near>0.03</near><far>30</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""


def ign(*args, timeout=10.0):
    return subprocess.run(["ign", *args], capture_output=True, text=True, timeout=timeout).stdout


def model_xyz(name):
    lines = ign("model", "-m", name, "-p").splitlines()
    for index, line in enumerate(lines):
        if line.strip().startswith("- Pose ["):
            return tuple(float(v) for v in lines[index + 1].strip(" []").split())
    return None


def spawn(world, name, sdf_text, tmp_dir):
    path = Path(tmp_dir) / f"{name}.sdf"
    path.write_text(sdf_text)
    ign("service", "-s", f"/world/{world}/remove", "--reqtype", "ignition.msgs.Entity",
        "--reptype", "ignition.msgs.Boolean", "--timeout", "3000", "--req", f'name: "{name}" type: MODEL')
    reply = ign("service", "-s", f"/world/{world}/create", "--reqtype", "ignition.msgs.EntityFactory",
                "--reptype", "ignition.msgs.Boolean", "--timeout", "5000",
                "--req", f'sdf_filename: "{path}" name: "{name}" allow_renaming: false')
    return "data: true" in reply


# ----------------------------------------------------------------------------- recorder
class Recorder(Node):
    def __init__(self, out_path, fps, max_seconds, tail_seconds):
        super().__init__("sim_video_recorder", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.fps, self.max_seconds, self.tail_seconds = fps, max_seconds, tail_seconds
        self.writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (WIDTH, HEIGHT))
        self.frames = {"wide": None, "bolt": None}
        self.stage, self.detail = "Starting", ""
        self.torque_baseline, self.torque, self.peak = None, 0.0, 0.0
        # (sim time, torque) of the last second: the bar shows the peak so a short snug
        # spike (a fraction of a second) is still visible at 15 frames per second.
        self.torque_history = deque()
        self.snug_peak = None
        self.turns = None
        self.summary = ""
        self.stages = []          # (video seconds, stage) for the editing script
        self.frame_count = 0
        self.finish_at = None
        self.done = threading.Event()
        self.font_big = ImageFont.truetype(FONT_BOLD, 30)
        self.font_mid = ImageFont.truetype(FONT_BOLD, 22)
        self.font_small = ImageFont.truetype(FONT_REGULAR, 18)

        self.create_subscription(Image, "/video/wide/image", lambda m: self.on_image("wide", m), qos_profile_sensor_data)
        self.create_subscription(Image, "/video/bolt/image", lambda m: self.on_image("bolt", m), qos_profile_sensor_data)
        self.create_subscription(Log, "/rosout", self.on_log, 100)
        self.create_subscription(WrenchStamped, "/wrist_ft", self.on_wrench, qos_profile_sensor_data)
        self.create_subscription(JointState, "/bolt_fixture/joint_states", self.on_bolt, 10)
        self.create_timer(1.0 / fps, self.on_tick)  # simulation time

    def on_image(self, key, msg):
        self.frames[key] = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)

    def on_log(self, msg):
        text = re.sub(r"\x1b\[[0-9;]*m", "", msg.msg)  # the node colours some lines (ANSI codes)
        if "[LAUNCHER_STATUS]" in text:
            case = re.search(r"robot_case=([^;]*)", text)
            detail = re.search(r"detail=(.*)$", text)
            if case and case.group(1).strip() != self.stage:
                self.stage = case.group(1).strip()
                self.stages.append((self.frame_count / self.fps, self.stage))
                self.get_logger().info(f"[{self.frame_count / self.fps:6.1f}s] {self.stage}")
            self.detail = detail.group(1).strip() if detail and detail.group(1).strip() != "-" else ""
            # The node reports wall-clock seconds ("..., 359 s"); in a simulation video that is
            # misleading (Gazebo runs slower than real time), so drop it from the caption.
            self.detail = re.sub(r",\s*\d+\s*s$", "", self.detail)
            if self.stage == "Tightening complete" and self.finish_at is None:
                self.finish_at = self.frame_count + int(self.tail_seconds * self.fps)
        if text.startswith("Nut is snug:"):
            self.summary = re.sub(r",\s*\d+\s*s$", "", text.replace("Nut is snug:", "").strip())
            peak = re.search(r"peak torque ([0-9.]+)", text)
            self.snug_peak = float(peak.group(1)) if peak else None

    def on_wrench(self, msg):
        tz = msg.wrench.torque.z
        if self.torque_baseline is None:
            self.torque_baseline = tz
        # Smooth for display only.
        self.torque = 0.8 * self.torque + 0.2 * abs(tz - self.torque_baseline)
        self.peak = max(self.peak, self.torque)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.torque_history.append((now, abs(tz - self.torque_baseline)))
        while self.torque_history and self.torque_history[0][0] < now - 1.0:
            self.torque_history.popleft()

    def on_bolt(self, msg):
        if "thread" in msg.name:
            self.turns = -msg.position[msg.name.index("thread")] / (2 * math.pi)

    # ------------------------------------------------------------------ drawing
    def compose(self):
        wide = self.frames["wide"]
        base = PilImage.fromarray(wide) if wide is not None else PilImage.new("RGB", (WIDTH, HEIGHT), (20, 24, 32))
        overlay = PilImage.new("RGBA", base.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)

        # Title strip.
        draw.rectangle([0, 0, WIDTH, 44], fill=(10, 14, 22, 200))
        draw.text((16, 7), "JAKA S12  |  AI nut pick  +  thread  +  tighten  (ROS 2 + Gazebo)", font=self.font_mid,
                  fill=(240, 244, 250, 255))

        # Current stage.
        draw.rounded_rectangle([16, 60, 700, 140 if self.detail else 108], radius=10, fill=(10, 14, 22, 190))
        draw.text((30, 66), self.stage, font=self.font_big, fill=(196, 181, 253, 255))
        if self.detail:
            draw.text((30, 106), self.detail[:70], font=self.font_small, fill=(226, 232, 240, 255))

        # Torque bar and thread turns.
        x0, y0 = 16, HEIGHT - 116
        draw.rounded_rectangle([x0, y0, x0 + 470, HEIGHT - 16], radius=10, fill=(10, 14, 22, 190))
        shown = max([value for _, value in self.torque_history], default=0.0)  # 1 s peak hold
        draw.text((x0 + 14, y0 + 8), f"Wrist torque  {shown:4.2f} N·m", font=self.font_mid, fill=(240, 244, 250, 255))
        bar_x0, bar_x1, bar_y0, bar_y1 = x0 + 14, x0 + 456, y0 + 44, y0 + 62
        draw.rectangle([bar_x0, bar_y0, bar_x1, bar_y1], fill=(51, 65, 85, 255))
        fill_x = bar_x0 + (bar_x1 - bar_x0) * min(1.0, shown / TORQUE_SCALE)
        colour = (239, 68, 68, 255) if shown >= SNUG_TORQUE else (34, 197, 94, 255)
        draw.rectangle([bar_x0, bar_y0, fill_x, bar_y1], fill=colour)
        snug_x = bar_x0 + (bar_x1 - bar_x0) * SNUG_TORQUE / TORQUE_SCALE
        draw.line([snug_x, bar_y0 - 4, snug_x, bar_y1 + 4], fill=(250, 204, 21, 255), width=3)
        turns = "no nut on the bolt" if self.turns is None else f"{self.turns:4.2f} turns"
        draw.text((x0 + 14, y0 + 70), f"Thread: {turns}     snug = {SNUG_TORQUE} N·m", font=self.font_small,
                  fill=(226, 232, 240, 255))

        # Snug banner once the robot reports the nut is snug.
        if self.snug_peak is not None:
            label = f"SNUG REACHED   {self.snug_peak:.2f} N·m"
            box = draw.textbbox((0, 0), label, font=self.font_big)
            bx = (WIDTH - (box[2] - box[0])) // 2
            draw.rounded_rectangle([bx - 24, 150, bx + box[2] - box[0] + 24, 204], radius=12, fill=(22, 163, 74, 230))
            draw.text((bx, 156), label, font=self.font_big, fill=(255, 255, 255, 255))

        frame = PilImage.alpha_composite(base.convert("RGBA"), overlay).convert("RGB")

        # Bolt close-up inset (bottom right).
        bolt = self.frames["bolt"]
        if bolt is not None:
            inset = PilImage.fromarray(bolt).resize((448, 252))
            frame.paste((250, 204, 21), [WIDTH - 448 - 22, HEIGHT - 252 - 22, WIDTH - 14, HEIGHT - 14])
            frame.paste(inset, (WIDTH - 448 - 18, HEIGHT - 252 - 18))
            ImageDraw.Draw(frame).text((WIDTH - 448 - 10, HEIGHT - 252 - 16), "Bolt close-up", font=self.font_small,
                                       fill=(250, 204, 21))
        return cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR)

    def on_tick(self):
        if self.done.is_set() or self.frames["wide"] is None:
            return
        self.writer.write(self.compose())
        self.frame_count += 1
        if (self.finish_at is not None and self.frame_count >= self.finish_at) or \
                self.frame_count >= self.max_seconds * self.fps:
            self.done.set()

    def close(self, stages_path):
        self.writer.release()
        Path(stages_path).write_text(json.dumps(
            {"fps": self.fps, "frames": self.frame_count, "stages": self.stages, "summary": self.summary}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="media_and_files/videos/raw.mp4")
    parser.add_argument("--world", default="empty_world")
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--max-seconds", type=float, default=600.0, help="simulation seconds")
    parser.add_argument("--tail-seconds", type=float, default=4.0)
    args = parser.parse_args()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    # Film cameras: wide shot of the cell, close-up next to wherever the bolt fixture is.
    tmp = tempfile.mkdtemp(prefix="sim_video_")
    bolt = model_xyz("bolt_fixture") or (-0.30, 0.75, 1.015)
    ok_wide = spawn(args.world, "video_camera_wide", camera_sdf(
        "video_camera_wide", "/video/wide/image", (-1.12, 1.22, 1.52), (-0.30, 0.46, 1.10), 1.05, args.fps), tmp)
    ok_bolt = spawn(args.world, "video_camera_bolt", camera_sdf(
        "video_camera_bolt", "/video/bolt/image",
        (bolt[0] + 0.20, bolt[1] - 0.20, bolt[2] + 0.17), (bolt[0], bolt[1], bolt[2] + 0.035), 0.75, args.fps), tmp)
    print(f"film cameras spawned: wide={ok_wide} bolt={ok_bolt} (bolt at {bolt})")

    bridge = subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
         "/video/wide/image@sensor_msgs/msg/Image[ignition.msgs.Image",
         "/video/bolt/image@sensor_msgs/msg/Image[ignition.msgs.Image"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

    rclpy.init()
    recorder = Recorder(out, args.fps, args.max_seconds, args.tail_seconds)
    signal.signal(signal.SIGTERM, lambda *_: recorder.done.set())
    try:
        while rclpy.ok() and not recorder.done.is_set():
            rclpy.spin_once(recorder, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stages_path = out.with_suffix(".json")
        recorder.close(stages_path)
        os.killpg(os.getpgid(bridge.pid), signal.SIGINT)
        print(f"wrote {out} ({recorder.frame_count} frames, {recorder.frame_count / args.fps:.1f} s) and {stages_path}")
        recorder.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
