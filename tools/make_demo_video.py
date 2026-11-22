#!/usr/bin/env python3
"""Cut a desktop recording from record_desktop_demo.py into a portfolio / YouTube video.

Chapters come from the robot stages logged during the recording. Each chapter gets a caption
bar and is sped up to a target length (shown as "xN" on screen); the detection and the snug
moment play at (or near) normal speed. Title and end cards are added.
Output: 1920x1080, 30 fps, H.264, yuv420p, +faststart.

Usage:
    python3 tools/make_demo_video.py media_and_files/videos/desktop_raw.mp4 media_and_files/videos/demo.mp4
"""
import argparse
import json
import subprocess
from pathlib import Path

FONT_BOLD = "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf"
FONT_REGULAR = "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf"
BACKGROUND = "0x0b1220"


def esc(text: str) -> str:
    return text.replace("\\", "\\\\").replace(":", "\\:").replace("'", "’").replace("%", "\\%")


def first(stages, prefix, after=0.0):
    return next((t for t, s in stages if s.startswith(prefix) and t >= after), None)


def card(label, lines, seconds):
    chain = f"color=c={BACKGROUND}:s=1920x1080:d={seconds}:r=30"
    for text, size, colour, dy in lines:
        font = FONT_BOLD if size >= 40 else FONT_REGULAR
        chain += (f",drawtext=fontfile={font}:text='{esc(text)}':fontsize={size}:fontcolor={colour}"
                  f":x=(w-text_w)/2:y=(h-text_h)/2+{dy}")
    return chain + f",fade=t=in:st=0:d=0.6,fade=t=out:st={seconds - 0.6}:d=0.6,format=yuv420p,setsar=1[{label}]"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("raw")
    parser.add_argument("out")
    parser.add_argument("--title", default="AI Robotic Nut Picking & Tightening")
    parser.add_argument("--subtitle", default="ROS 2  |  MoveIt 2  |  Gazebo  |  YOLOv12-seg + AAE 6D pose  |  JAKA S12")
    parser.add_argument("--end-line", default="ROS 2  |  Computer vision  |  Motion planning  |  Simulation  |  Robot control")
    args = parser.parse_args()

    info = json.loads(Path(args.raw).with_suffix(".json").read_text())
    stages = info["stages"]
    duration = info["duration"]
    click = info["events"][0][0] if info["events"] else 0.0

    pick = first(stages, "Sequence 0")
    bolt = first(stages, "Finding the bolt", after=pick or 0)
    stroke1 = first(stages, "Tightening: stroke 1", after=bolt or 0)
    snug = first(stages, "Tightening: snug", after=stroke1 or 0)
    strokes = [t for t, s in stages if s.startswith("Tightening: stroke") and stroke1 and snug and stroke1 <= t < snug]
    last_stroke = strokes[-1] if strokes else snug
    next_det = first(stages, "Detecting next nut", after=snug or 0)
    if None in (pick, bolt, stroke1, snug):
        raise SystemExit(f"recording is missing stages: pick={pick} bolt={bolt} stroke1={stroke1} snug={snug}")
    detect_start = max(click + 5, pick - 14)  # the last seconds before the robot moves = detection

    # (caption, start, end, target seconds or None for real time)
    chapters = [
        ("1  Start the system: Gazebo, RViz, MoveIt 2, AAE and YOLO", max(0.0, click - 2), detect_start, 16),
        ("2  AI detection: YOLOv12 segmentation + AAE 6D pose", detect_start, pick, 12),
        ("3  Plan, move and grasp the nut (MoveIt 2)", pick, bolt, 12),
        ("4  Find the bolt with the depth camera, place the nut", bolt, stroke1, 12),
        ("5  Tightening in 180° strokes, wrist torque monitored", stroke1, last_stroke, 22),
        ("6  Snug torque reached: the robot stops", last_stroke, next_det or snug + 6, 10),
        ("7  Next detection: the cell asks for the next nut", next_det or snug + 6, duration, 12),
    ]

    filters, labels = [], []
    for index, (caption, start, end, target) in enumerate(chapters):
        if end is None or end - start < 0.5:
            continue
        length = end - start
        speed = max(1.0, length / target) if target else 1.0
        speed = round(speed, 1) if speed < 10 else round(speed)
        overlay = (f"drawbox=x=0:y=0:w=iw:h=78:color=0x0b1220@0.82:t=fill,"
                   f"drawtext=fontfile={FONT_BOLD}:text='{esc(caption)}':fontsize=38:fontcolor=white:x=28:y=16")
        if speed > 1.05:
            overlay += (f",drawtext=fontfile={FONT_BOLD}:text='{esc(f'x{speed:g} speed')}':fontsize=34"
                        f":fontcolor=0xfacc15:x=w-text_w-28:y=19")
        filters.append(f"[0:v]trim={start:.2f}:{end:.2f},setpts=(PTS-STARTPTS)/{speed},"
                       f"scale=1920:1080:flags=lanczos,fps=30,{overlay},format=yuv420p,setsar=1[c{index}]")
        labels.append(f"[c{index}]")
        print(f"chapter {index + 1}: {start:7.1f}-{end:7.1f}s ({length:6.1f}s) x{speed:g} -> {length / speed:5.1f}s")

    title = card("title", [(args.title, 72, "white", -60), (args.subtitle, 34, "0xc4b5fd", 30),
                           ("Full system demo, recorded live from the simulation desktop", 28, "0x94a3b8", 100)], 4.5)
    ending = card("ending", [("Detect  ->  Pick  ->  Place  ->  Tighten  ->  Next part", 52, "white", -50),
                             (args.end_line, 32, "0xc4b5fd", 30),
                             ("Available for your robotics project", 30, "0x94a3b8", 100)], 5.0)
    graph = ";".join([title, ending] + filters) + ";" + "[title]" + "".join(labels) + "[ending]" + \
        f"concat=n={len(labels) + 2}:v=1:a=0[out]"

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-i", args.raw,
                    "-filter_complex", graph, "-map", "[out]",
                    "-c:v", "libx264", "-preset", "slow", "-crf", "20", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", args.out], check=True)
    probe = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration,size",
                            "-of", "default=nw=1", args.out], capture_output=True, text=True).stdout
    print(f"wrote {args.out}\n{probe}")


if __name__ == "__main__":
    main()
