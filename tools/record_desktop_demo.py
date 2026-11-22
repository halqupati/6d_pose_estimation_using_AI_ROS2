#!/usr/bin/env python3
"""Record a full desktop demo: launcher + Gazebo + RViz + YOLO window, from "Start All" to the end.

Runs everything on an X display of its own (e.g. a private Xtigervnc :97, 2560x1440), so your
desktop is not touched:
  1. starts tools/project_launcher.py on that display (task chosen with LAUNCHER_TASK),
     with LAUNCHER_DEMO_VIEW=1, so the simulation starts with demo_view:=true: Gazebo and
     RViz without side panels, zoomed in on the nuts, the bolt and the robot,
  2. keeps the four windows tiled 2x2 with python-xlib (no window manager needed):
        Gazebo GUI | RViz
        launcher   | YOLO "Stable Detection" window
  3. clicks the launcher's green "Start All" button with a real (XTest) mouse click,
  4. records the whole display with ffmpeg (x11grab),
  5. logs the robot's stages ([LAUNCHER_STATUS] on /rosout) with their time in the recording,
  6. stops a few seconds after "Next nut detected" (or --max-minutes).
Then tools/make_demo_video.py cuts it into the final video.

Usage (ROS 2 sourced; DISPLAY must be the recording display):
    DISPLAY=:97 python3 tools/record_desktop_demo.py --out media_and_files/videos/desktop_raw.mkv
"""
import argparse
import json
import os
import re
import signal
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.msg import Log
from Xlib import X, display
from Xlib.ext import xtest

ROOT = Path(__file__).resolve().parents[1]
# Window title (substring) -> (x, y, width, height) on a 2560x1440 display. The bottom row is
# taller so the launcher's dashboard (status, presets, telemetry) fits.
LAYOUT = {
    "Gazebo": (0, 0, 1280, 640),
    "RViz": (1280, 0, 1280, 640),
    "JAKA S12 Control Deck": (0, 640, 1280, 800),
    "Stable Detection": (1280, 640, 1280, 800),
}
START_ALL_GREEN = (22, 163, 74)  # launcher START colour (#16A34A)


class Desktop:
    """Window tiling and mouse clicks on the recording display."""

    def __init__(self) -> None:
        self.d = display.Display()
        self.root = self.d.screen().root

    def windows(self):
        found = {}
        for window in self.root.query_tree().children:
            try:
                name = window.get_wm_name()
                attrs = window.get_attributes()
            except Exception:  # window vanished
                continue
            if not name or attrs.map_state != X.IsViewable:
                continue
            for key in LAYOUT:
                if key in str(name):
                    found[key] = window
        return found

    def tile(self) -> dict:
        found = self.windows()
        for key, window in found.items():
            x, y, w, h = LAYOUT[key]
            try:
                geometry = window.get_geometry()
                if (geometry.x, geometry.y, geometry.width, geometry.height) != (x, y, w, h):
                    window.configure(x=x, y=y, width=w, height=h, border_width=0)
            except Exception:
                pass
        self.d.sync()
        return found

    def screenshot(self, x, y, w, h) -> np.ndarray:
        raw = self.root.get_image(x, y, w, h, X.ZPixmap, 0xFFFFFFFF).data
        return np.frombuffer(raw, np.uint8).reshape(h, w, 4)[:, :, :3][:, :, ::-1]  # BGRX -> RGB

    def click(self, x: int, y: int) -> None:
        xtest.fake_input(self.d, X.MotionNotify, x=x, y=y)
        self.d.sync()
        time.sleep(0.4)
        xtest.fake_input(self.d, X.ButtonPress, 1)
        self.d.sync()
        time.sleep(0.12)
        xtest.fake_input(self.d, X.ButtonRelease, 1)
        self.d.sync()

    def find_start_all(self):
        """Centre of the green Start All button in the launcher's header (top 130 px of its tile)."""
        x0, y0, w0, _ = LAYOUT["JAKA S12 Control Deck"]
        image = self.screenshot(x0, y0, w0, 130).astype(int)
        mask = np.all(np.abs(image - np.array(START_ALL_GREEN)) < 12, axis=2)
        ys, xs = np.nonzero(mask)
        if len(xs) < 200:
            return None
        # Start All is the left-most green button in the header.
        left = xs < xs.min() + 160
        return int(x0 + xs[left].mean()), int(y0 + ys[left].mean())


class StageLogger:
    """Collects [LAUNCHER_STATUS] stages from /rosout with their time since recording start."""

    def __init__(self, t0: float) -> None:
        self.t0 = t0
        self.stages = []
        self.stage = None
        rclpy.init()
        self.node = rclpy.create_node("demo_stage_logger")
        self.node.create_subscription(Log, "/rosout", self.on_log, 200)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def on_log(self, msg: Log) -> None:
        text = re.sub(r"\x1b\[[0-9;]*m", "", msg.msg)
        match = re.search(r"\[LAUNCHER_STATUS\] robot_case=([^;]*)", text)
        if match and match.group(1).strip() != self.stage:
            self.stage = match.group(1).strip()
            self.stages.append((round(time.time() - self.t0, 2), self.stage))
            print(f"[{time.time() - self.t0:7.1f}s] {self.stage}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # Matroska stays playable even if ffmpeg is killed (MP4 needs its index written at the end).
    parser.add_argument("--out", default="media_and_files/videos/desktop_raw.mkv")
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--max-minutes", type=float, default=45.0)
    parser.add_argument("--tail-seconds", type=float, default=10.0)
    args = parser.parse_args()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    screen = os.environ["DISPLAY"]

    desktop = Desktop()
    width, height = desktop.d.screen().width_in_pixels, desktop.d.screen().height_in_pixels

    launcher = subprocess.Popen(["python3", str(ROOT / "tools/project_launcher.py")], cwd=ROOT,
                                env=dict(os.environ, LAUNCHER_TASK=os.environ.get("LAUNCHER_TASK", "pick_tighten"),
                                         LAUNCHER_DEMO_VIEW="1"),  # zoomed-in Gazebo + RViz
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    for _ in range(60):
        if "JAKA S12 Control Deck" in desktop.tile():
            break
        time.sleep(0.5)
    time.sleep(3.0)
    desktop.tile()

    ffmpeg = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "x11grab", "-draw_mouse", "1", "-framerate", str(args.fps),
         "-video_size", f"{width}x{height}", "-i", screen,
         "-c:v", "libx264", "-preset", "ultrafast", "-crf", "26", "-pix_fmt", "yuv420p", "-f", "matroska", str(out)],
        stdin=subprocess.PIPE, preexec_fn=os.setsid)
    t0 = time.time()
    logger = StageLogger(t0)
    events = []

    button = None
    for _ in range(90):  # the launcher may still be drawing its widgets (slow on a busy machine)
        time.sleep(1.0)
        desktop.tile()
        button = desktop.find_start_all()
        if button is not None:
            break
    if button is None:
        print("Start All button not found; clicking the header's default position")
        button = (LAYOUT["JAKA S12 Control Deck"][0] + 1120, LAYOUT["JAKA S12 Control Deck"][1] + 66)
    desktop.click(*button)
    events.append((round(time.time() - t0, 2), "Start All clicked"))
    print(f"[{time.time() - t0:7.1f}s] Start All clicked at {button}", flush=True)

    finish_at = None
    m0 = time.monotonic()  # does not advance while the machine is suspended
    try:
        while time.monotonic() - m0 < args.max_minutes * 60:
            desktop.tile()  # windows appear one after another
            if finish_at is None and logger.stage in ("Next nut detected", "Tightening failed"):
                finish_at = time.monotonic() + args.tail_seconds
            if finish_at is not None and time.monotonic() >= finish_at:
                break
            time.sleep(1.0)
    finally:
        # Save the stage log first: it is needed for editing even if ffmpeg is slow to finish.
        Path(out.with_suffix(".json")).write_text(json.dumps(
            {"events": events, "stages": logger.stages, "duration": round(time.time() - t0, 2)}, indent=2))
        ffmpeg.stdin.write(b"q")
        ffmpeg.stdin.flush()
        try:
            ffmpeg.wait(timeout=600)  # encoding can lag behind a busy machine
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(ffmpeg.pid), signal.SIGINT)
        os.killpg(os.getpgid(launcher.pid), signal.SIGTERM)  # the launcher closes what it started
        print(f"wrote {out} and {out.with_suffix('.json')}")


if __name__ == "__main__":
    main()
