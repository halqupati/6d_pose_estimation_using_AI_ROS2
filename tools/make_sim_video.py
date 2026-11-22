#!/usr/bin/env python3
"""Turn a raw recording from sim_video_recorder.py into a short YouTube-ready video.

Layout:
  title card (4 s)  ->  recording, with the tightening strokes sped up  ->  end card (5 s)

The tightening part is found from the stage timestamps in <raw>.json: strokes 1 to N-1 are
played faster with an on-screen "x N speed" label; the last stroke (torque rising to snug and
the robot stopping) plays at normal speed.
Output: H.264, 1280x720, 30 fps, yuv420p, +faststart (what YouTube recommends).

Usage:
    python3 tools/make_sim_video.py media_and_files/videos/raw.mp4 media_and_files/videos/jaka_s12_pick_tighten.mp4
"""
import argparse
import json
import subprocess
from pathlib import Path

FONT_BOLD = "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf"
FONT_REGULAR = "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf"
BACKGROUND = "0x0b1220"


def text(t: str) -> str:
    """Escape text for ffmpeg drawtext."""
    return t.replace("\\", "\\\\").replace(":", "\\:").replace("'", "’").replace("%", "\\%")


def card(label: str, lines: list[tuple[str, int, str, int]], seconds: float) -> str:
    """A filter chain producing a solid-colour card with centred lines of text.
    lines: (text, font size, colour, y offset from the centre)."""
    chain = f"color=c={BACKGROUND}:s=1280x720:d={seconds}:r=30"
    for line, size, colour, dy in lines:
        font = FONT_BOLD if size >= 34 else FONT_REGULAR
        chain += (f",drawtext=fontfile={font}:text='{text(line)}':fontsize={size}:fontcolor={colour}"
                  f":x=(w-text_w)/2:y=(h-text_h)/2+{dy}")
    chain += f",fade=t=in:st=0:d=0.6,fade=t=out:st={seconds - 0.6}:d=0.6,format=yuv420p,setsar=1[{label}]"
    return chain


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("raw")
    parser.add_argument("out")
    parser.add_argument("--speed", type=float, default=4.0, help="speed-up of the tightening strokes")
    parser.add_argument("--title", default="AI Nut Picking & Tightening")
    parser.add_argument("--subtitle", default="JAKA S12  |  YOLO + AAE 6D pose  |  ROS 2 + MoveIt 2 + Gazebo")
    args = parser.parse_args()

    info = json.loads(Path(args.raw).with_suffix(".json").read_text())
    duration = info["frames"] / info["fps"]
    stages = info["stages"]
    start = next((t for t, s in stages if s.startswith("Tightening: stroke 1")), None)
    snug = next((t for t, s in stages if s.startswith("Tightening: snug")), None)
    # Speed up every stroke except the last one: the last stroke shows the torque rising to
    # the snug threshold and the robot stopping, which viewers should see at normal speed.
    strokes = [t for t, s in stages if s.startswith("Tightening: stroke") and snug is not None and t < snug]
    end = strokes[-1] if len(strokes) >= 2 else snug
    summary = info.get("summary") or ""

    title = card("title", [(args.title, 56, "white", -40), (args.subtitle, 26, "0xc4b5fd", 40),
                           ("Simulation", 22, "0x94a3b8", 100)], 4.0)
    ending = card("ending", [("Nut picked, threaded and tightened to snug", 40, "white", -50),
                             (summary, 26, "0xc4b5fd", 20),
                             ("Step-by-step tutorial series: coming soon", 24, "0x94a3b8", 80)], 5.0)

    if start is not None and end is not None and end > start:
        speed_label = (f"drawtext=fontfile={FONT_BOLD}:text='{text(f'x{args.speed:g} speed')}':fontsize=34"
                       f":fontcolor=0xfacc15:box=1:boxcolor=0x0b1220@0.7:boxborderw=10:x=w-text_w-24:y=60")
        body = (f"[0:v]trim=0:{start},setpts=PTS-STARTPTS,fps=30,format=yuv420p,setsar=1[b1];"
                f"[0:v]trim={start}:{end},setpts=(PTS-STARTPTS)/{args.speed},fps=30,{speed_label},"
                f"format=yuv420p,setsar=1[b2];"
                f"[0:v]trim={end}:{duration},setpts=PTS-STARTPTS,fps=30,format=yuv420p,setsar=1[b3];")
        parts = "[title][b1][b2][b3][ending]concat=n=5:v=1:a=0[out]"
    else:  # no tightening stages found: keep the recording as it is
        body = "[0:v]setpts=PTS-STARTPTS,fps=30,format=yuv420p,setsar=1[b1];"
        parts = "[title][b1][ending]concat=n=3:v=1:a=0[out]"

    graph = f"{title};{ending};{body}{parts}"
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-i", args.raw,
                    "-filter_complex", graph, "-map", "[out]",
                    "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", args.out], check=True)
    probe = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration,size",
                            "-of", "default=nw=1", args.out], capture_output=True, text=True).stdout
    print(f"wrote {args.out}\n{probe}")


if __name__ == "__main__":
    main()
