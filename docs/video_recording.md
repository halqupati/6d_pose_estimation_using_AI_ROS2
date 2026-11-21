# Recording a video of the simulation

Two tools turn a simulation run into a short, YouTube-ready video.

## 1. Record: `tools/sim_video_recorder.py`

With the simulation running:

```bash
source jaka_ros_ws/install/setup.bash
python3 tools/sim_video_recorder.py --out media_and_files/videos/raw.mp4
```

Then start the robot task (for example, in the launcher, **Task: Pick + tighten** → start the
bin-picking nodes).

What it does:

- **Film cameras:** it spawns two static cameras into the running world, a wide shot of the cell
  and a close-up next to wherever `bolt_fixture` is. Nothing on disk changes. Gazebo renders them
  on the GPU, so the picture does not depend on the Gazebo GUI.
- **Overlays on every frame:**
  - the robot's current stage, read from the bin-picking node's `[LAUNCHER_STATUS]` log on `/rosout`
  - a live wrist-torque bar from `/wrist_ft`, with the snug threshold marked
  - the thread turn counter from `/bolt_fixture/joint_states`
  - the bolt close-up as an inset
- **Simulation time:** frames are written at a fixed rate of simulation time (15 fps), so motion
  looks real-time even when Gazebo runs slower than real time.
- **Stopping:** it stops 4 s after "Tightening complete" (or after `--max-seconds`), and writes
  `raw.json` with the time of every stage.

## 2. Edit: `tools/make_sim_video.py`

```bash
python3 tools/make_sim_video.py media_and_files/videos/raw.mp4 media_and_files/videos/jaka_s12_pick_tighten.mp4
```

It builds:

1. a title card (4 s; `--title`, `--subtitle`)
2. the recording, with the tightening strokes (from "Tightening: stroke 1" to "Tightening: snug")
   sped up `--speed` times (default 4×) and labelled on screen
3. an end card (5 s) with the run's result (turns, strokes, peak torque, time)

The output is H.264, 1280×720, 30 fps, yuv420p, with `+faststart`, which is what YouTube
recommends.

The videos are not committed to git (`media_and_files/videos/` is ignored); upload them
directly.
