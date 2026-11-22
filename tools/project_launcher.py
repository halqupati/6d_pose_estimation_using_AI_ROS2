#!/usr/bin/env python3

import math
import os
import queue
import re
import shlex
import shutil
import signal
import socket
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText
from urllib.error import URLError
from urllib.request import urlopen


ROOT_DIR = Path(__file__).resolve().parents[1]
ROS_SETUP = ROOT_DIR / "jaka_ros_ws" / "install" / "setup.bash"
AAE_WORKSPACE_DIR = ROOT_DIR / "aae_infer" / "autoencoder_ws"
SIM_LAUNCH_CANDIDATES = [
    "jaka_s12_with_two_camera_simulation_with_pcl_and_custom_gripper_uder_test.launch.py",
    "jaka_s12_with_two_camera_simulation_with_pcl_and_custom_gripper.launch.py",
]
BIN_PICKING_LAUNCH_CANDIDATES = [
    "run_bin_picking_nodes_to_control_jaka_s12.launch.py",
    "run_bin_picking_nodes.launch.py",
]
YOLO_SCRIPT_CANDIDATES = [
    "yolo_ros_infer_sim_v11_to_control_jaka_s12.py",
    "yolo_ros_infer_sim_v11.py",
]

CORE_PROCESS_KEYS = ("sim", "bin_picking")
STATUS_COLORS = {
    "running": ("#163F2A", "#51D88A", "#D9FBE8"),
    "stopped": ("#5B4A11", "#F4C95D", "#FFF6D9"),
    "warning": ("#5A3010", "#FF9B4A", "#FFE8D6"),
    # Matching process found, but it was NOT started by this launcher (other terminal,
    # real-robot bringup, ...). The launcher never pushes live settings into it.
    "external": ("#14324A", "#60A5FA", "#DBEAFE"),
}

BG = "#0D1117"
PANEL_BG = "#111827"
PANEL_ALT = "#0F172A"
TEXT = "#E5E7EB"
MUTED = "#94A3B8"
ENTRY_BG = "#0B1220"
ENTRY_FG = "#E2E8F0"
ACCENT = "#22C55E"
ACCENT_SOFT = "#1A2E22"
DANGER = "#DC2626"
DANGER_ACTIVE = "#991B1B"
STOP = "#F59E0B"
STOP_ACTIVE = "#B45309"
START = "#16A34A"
START_ACTIVE = "#166534"
WARNING_WINDOW_SEC = 8.0
LAUNCHER_STATUS_PREFIX = "[LAUNCHER_STATUS]"
PROCESS_START_TIMEOUT_SEC = 8.0
# Cold Gazebo + MoveIt start with repeated `ros2 control list_controllers` polling
# regularly exceeds 60 s on this machine; 120 s avoids false "not ready" failures.
SIM_READY_TIMEOUT_SEC = 120.0
AAE_READY_TIMEOUT_SEC = 90.0
STARTUP_POLL_INTERVAL_SEC = 0.5

# This launcher drives the Gazebo simulation only. The bin-picking node is always
# started with robot_mode:=sim; the physical robot uses
# jaka_hardware_bringup/robot_hw.launch.py (robot_mode:=real) instead.
LAUNCHER_ROBOT_MODE = "sim"
BIN_PICKING_NODE_NAME = "/bin_picking_node_v5_to_control_jaka_s12"

# Motion presets (velocity, acceleration, descent_step [m], gripper_time [s]).
# "Safe" equals the real-robot limits enforced by the node in robot_mode=real.
# The faster presets are SIMULATION presets: this launcher only ever starts the
# node with robot_mode:=sim, and the node rejects them in robot_mode=real.
# All were verified end-to-end (grasp, transport, place) in Gazebo.
# In sim the descent is one Cartesian move, so descent_step only matters on hardware.
MOTION_PRESETS = {
    "Safe": (0.30, 0.30, 0.002, 1.20),
    "Balanced": (0.60, 0.50, 0.004, 1.00),
    "Fast": (0.90, 0.80, 0.006, 1.00),
    "Very Fast": (1.00, 1.00, 0.010, 0.60),
}
PRESET_COLORS = {
    "Safe": ("#334155", "#1E293B"),
    "Balanced": ("#2563EB", "#1D4ED8"),
    "Fast": ("#7C3AED", "#5B21B6"),
    "Very Fast": ("#BE123C", "#9F1239"),
}
# Explicitly selected simulation preset at startup (shown as "Active preset" in the UI).
DEFAULT_PRESET = "Fast"
DEFAULT_MOTION_PROFILE = MOTION_PRESETS[DEFAULT_PRESET]
GZ_STATS_TOPICS = ("/world/empty_world/stats", "/stats")

# Processes the cleanup matcher must never stop, even if their command line contains a
# process pattern (see find_process_targets). Compared with the executable's basename.
NON_TARGET_EXECUTABLES = {
    "bash", "sh", "dash", "zsh", "fish", "sudo", "timeout", "nohup", "env",
    "cc1plus", "cc1", "c++", "g++", "gcc", "clang", "clang++", "ld", "collect2", "as",
    "make", "gmake", "cmake", "ninja", "colcon", "ccache",
    "grep", "egrep", "rg", "ugrep", "sed", "awk", "tail", "head", "less", "cat", "find",
    "vim", "nvim", "nano", "emacs", "code", "codium", "git", "ps", "pgrep", "pkill",
}

# Tasks the bin-picking node can run (its task_mode launch argument).
TASK_MODES = {
    "Pick & place": "pick_place",
    "Tighten (bolt)": "tighten",          # a nut already threaded on the bolt
    "Pick + tighten": "pick_tighten",     # pick a nut, put it on the bolt, tighten it
}
# Simulator ground truth of the bolt fixture's thread (worlds/bolt_m20), shown in the
# "Bolt Thread" tile. The robot itself never uses this; it detects snug from wrist torque.
BOLT_JOINT_STATE_TOPIC = "/world/empty_world/model/bolt_fixture/joint_state"
BOLT_FIXTURE_MODEL = "bolt_fixture"
BOLT_SEAT_Z = 0.020                  # plate top in the fixture frame (worlds/bolt_m20)

# One nut type for the whole perception stack: passed to both AAE and YOLO so they
# can never be started with mismatched models. "m20" = legacy trained models,
# "m6" = DIN 934 M6 (requires trained weights; see aae_ros_infer_sim_v4.py).
NUT_MODEL = os.environ.get("NUT_MODEL", "m20").strip().lower()


def shell_quote(path: Path | str) -> str:
    return shlex.quote(str(path))


def detect_conda_executable() -> Path | None:
    candidates: list[Path] = []

    conda_exe = os.environ.get("CONDA_EXE")
    if conda_exe:
        candidates.append(Path(conda_exe).expanduser())

    which_conda = shutil.which("conda")
    if which_conda:
        candidates.append(Path(which_conda))

    for base in ("~/miniconda3", "~/anaconda3", "~/mambaforge", "~/miniforge3"):
        candidates.append(Path(base).expanduser() / "bin" / "conda")

    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()

    return None


def detect_first_existing(directory: Path, candidates: list[str]) -> str:
    for filename in candidates:
        if (directory / filename).exists():
            return filename
    return candidates[-1]


def detect_sim_launch() -> str:
    launch_dir = ROOT_DIR / "jaka_ros_ws" / "src" / "bin_packing_pkg" / "launch"
    return detect_first_existing(launch_dir, SIM_LAUNCH_CANDIDATES)


def detect_bin_picking_launch() -> str:
    launch_dir = ROOT_DIR / "jaka_ros_ws" / "src" / "bin_packing_pkg" / "launch"
    return detect_first_existing(launch_dir, BIN_PICKING_LAUNCH_CANDIDATES)


def detect_yolo_script() -> str:
    script_dir = ROOT_DIR / "yolov12_seg_infer"
    return detect_first_existing(script_dir, YOLO_SCRIPT_CANDIDATES)


def build_ros_launch_command(package: str, launch_file: str, launch_args: dict[str, float | str] | None = None) -> str:
    command = f"source {shell_quote(ROS_SETUP)} && ros2 launch {shlex.quote(package)} {shlex.quote(launch_file)}"
    if launch_args:
        rendered_args = [f"{name}:={value}" for name, value in launch_args.items()]
        command = f"{command} {' '.join(shlex.quote(arg) for arg in rendered_args)}"
    return command


def build_ros_shell_command(command: str) -> str:
    return f"source {shell_quote(ROS_SETUP)} && {command}"


def run_shell_capture(command: str, timeout_sec: float = 15.0) -> tuple[int, str]:
    try:
        completed = subprocess.run(
            ["/bin/bash", "-lc", command],
            cwd=ROOT_DIR,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            check=False,
        )
        output = (completed.stdout or "") + (completed.stderr or "")
        return completed.returncode, output
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        return 124, f"{stdout}{stderr}"


def controller_is_active(controllers_output: str, controller_name: str) -> bool:
    # `ros2 control list_controllers` colours its output even when piped
    # ("\x1b[92mactive\x1b[0m"); strip the ANSI codes or no controller ever looks active
    # and Start All waits for a "simulation ready" that never comes.
    controllers_output = re.sub(r"\x1b\[[0-9;]*m", "", controllers_output)
    for line in controllers_output.splitlines():
        parts = line.split()
        if not parts or parts[0] != controller_name:
            continue
        if "active" in {part.lower() for part in parts[1:]}:
            return True
    return False


def is_tcp_port_available(port: int, host: str = "127.0.0.1") -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.2)
        return sock.connect_ex((host, port)) != 0


def choose_available_port(preferred_ports: list[int], fallback_start: int = 5010, fallback_count: int = 20) -> int:
    for port in preferred_ports:
        if is_tcp_port_available(port):
            return port

    for port in range(fallback_start, fallback_start + fallback_count):
        if is_tcp_port_available(port):
            return port

    return preferred_ports[-1]


def build_conda_python_command(
    conda_exe: Path | None,
    env_name: str,
    script_dir: Path,
    script_name: str,
    env_vars: dict[str, str] | None = None,
) -> str:
    if conda_exe is None:
        return (
            "echo 'Conda executable not found. Set CONDA_EXE or install conda in a standard location.' "
            "&& exit 1"
        )

    rendered_env = ""
    if env_vars:
        rendered_env = " ".join(f"{name}={shlex.quote(value)}" for name, value in env_vars.items()) + " "

    return (
        f"source {shell_quote(ROS_SETUP)} && "
        f"cd {shell_quote(script_dir)} && "
        f"{shell_quote(conda_exe)} run --no-capture-output -n {shlex.quote(env_name)} "
        f"env {rendered_env}PYTHONUNBUFFERED=1 python3 {shlex.quote(script_name)}"
    )


@dataclass
class ProcessConfig:
    key: str
    title: str
    command: str
    cleanup_patterns: tuple[str, ...] = ()


def _iter_process_rows() -> list[tuple[int, int, str]]:
    completed = subprocess.run(
        ["ps", "-eo", "pid=,pgid=,args="],
        capture_output=True,
        text=True,
        check=False,
    )
    rows: list[tuple[int, int, str]] = []
    for raw_line in completed.stdout.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = line.split(None, 2)
        if len(parts) < 3:
            continue
        try:
            pid = int(parts[0])
            pgid = int(parts[1])
        except ValueError:
            continue
        rows.append((pid, pgid, parts[2]))
    return rows


def find_process_targets(patterns: tuple[str, ...]) -> dict[int, int]:
    if not patterns:
        return {}

    current_pid = os.getpid()
    targets: dict[int, int] = {}
    for pid, pgid, args in _iter_process_rows():
        if pid == current_pid or "project_launcher.py" in args:
            continue
        # Never match tools that merely mention a pattern in their arguments: compiling
        # bin_picking_node_v5_to_control_jaka_s12.cpp, grepping for it, editing it or a
        # shell running such a command. (A launcher-started bash wrapper is still stopped:
        # its children match by themselves and the whole process group is terminated.)
        executable = os.path.basename(args.split(None, 1)[0])
        if executable in NON_TARGET_EXECUTABLES:
            continue
        if any(pattern in args for pattern in patterns):
            targets[pid] = pgid
    return targets


def _pid_exists(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def terminate_process_targets(targets: dict[int, int], immediate: bool = False, timeout_sec: float = 3.0) -> None:
    if not targets:
        return

    group_signal = signal.SIGKILL if immediate else signal.SIGTERM
    pid_signal = signal.SIGKILL if immediate else signal.SIGTERM

    for pgid in sorted(set(targets.values())):
        try:
            os.killpg(pgid, group_signal)
        except ProcessLookupError:
            pass

    for pid in sorted(targets):
        try:
            os.kill(pid, pid_signal)
        except ProcessLookupError:
            pass

    if immediate:
        return

    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        remaining = [pid for pid in targets if _pid_exists(pid)]
        if not remaining:
            return
        time.sleep(0.1)

    for pgid in sorted(set(targets.values())):
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    for pid in sorted(targets):
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


class ManagedProcess:
    def __init__(self, config: ProcessConfig, log_queue: queue.Queue):
        self.config = config
        self.log_queue = log_queue
        self.process: subprocess.Popen | None = None
        self.stop_requested = False
        self.stop_label = "Stopped"
        self.stop_state = "stopped"

    def is_running(self) -> bool:
        return self.is_owned_running() or bool(self._external_targets())

    def is_owned_running(self) -> bool:
        """True only for a process this launcher started itself."""
        return self.process is not None and self.process.poll() is None

    def _external_targets(self, max_age_sec: float = 0.5) -> dict[int, int]:
        # The UI polls is_running() ~10x per 100 ms tick; each call scans all of /proc.
        # A short cache keeps the launcher light without changing behaviour.
        now = time.time()
        cached = getattr(self, "_targets_cache", None)
        if cached is not None and now - cached[0] < max_age_sec:
            return dict(cached[1])
        targets = find_process_targets(self.config.cleanup_patterns)
        self._targets_cache = (now, targets)
        return dict(targets)

    def start(self, command: str) -> None:
        if self.is_running():
            self.log_queue.put(("log", self.config.key, "[launcher] process already running; start request ignored\n"))
            self.log_queue.put(("status", self.config.key, ("Running", "running")))
            return

        self.stop_requested = False
        self.stop_label = "Stopped"
        self.stop_state = "stopped"
        self.log_queue.put(("log", self.config.key, f"$ {command}\n"))
        self.process = subprocess.Popen(
            ["/bin/bash", "-lc", command],
            cwd=ROOT_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
        )

        self.log_queue.put(("status", self.config.key, ("Running", "running")))

        threading.Thread(target=self._read_output, daemon=True).start()
        threading.Thread(target=self._watch_exit, daemon=True).start()

    def stop(self, immediate: bool = False, owned_only: bool = False) -> None:
        if not (self.is_owned_running() if owned_only else self.is_running()):
            return

        self.stop_requested = True
        self.stop_label = "Emergency Stop" if immediate else "Stopped"
        self.stop_state = "warning" if immediate else "stopped"
        targets = {} if owned_only else self._external_targets()

        if self.process is not None:
            try:
                targets[self.process.pid] = os.getpgid(self.process.pid)
            except ProcessLookupError:
                pass

        threading.Thread(target=self._stop_targets, args=(targets, immediate), daemon=True).start()

    def _stop_targets(self, targets: dict[int, int], immediate: bool) -> None:
        terminate_process_targets(targets, immediate=immediate)
        if not self.is_running():
            self.log_queue.put(("status", self.config.key, (self.stop_label, self.stop_state)))
        else:
            self.log_queue.put(("status", self.config.key, ("Warning", "warning")))
            self.log_queue.put(("log", self.config.key, "[launcher] some matching processes are still running after stop request\n"))

    def _read_output(self) -> None:
        assert self.process is not None
        assert self.process.stdout is not None
        for line in self.process.stdout:
            self.log_queue.put(("log", self.config.key, line))

    def _watch_exit(self) -> None:
        assert self.process is not None
        return_code = self.process.wait()
        if self.stop_requested:
            label = self.stop_label
            state = self.stop_state
        elif return_code == 0:
            label = "Exited (0)"
            state = "warning"
        else:
            label = f"Exited ({return_code})"
            state = "warning"

        self.log_queue.put(("status", self.config.key, (label, state)))
        self.log_queue.put(("log", self.config.key, f"[process exited with code {return_code}]\n"))
        self.process = None


class ProjectLauncherApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("JAKA S12 Control Deck")
        screen_w, screen_h = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.root.geometry(f"{min(1880, screen_w - 40)}x{min(1040, screen_h - 80)}+20+20")
        self.root.minsize(1100, 700)
        self.root.configure(bg=BG)

        self.log_queue: queue.Queue = queue.Queue()
        self.conda_exe = detect_conda_executable()
        self.aae_port = choose_available_port([5001, 5003, 5005])

        default_velocity, default_acceleration, default_descent, default_gripper = DEFAULT_MOTION_PROFILE
        self.velocity_var = tk.DoubleVar(value=default_velocity)
        self.acceleration_var = tk.DoubleVar(value=default_acceleration)
        self.grasp_descent_step_var = tk.DoubleVar(value=default_descent)
        self.gripper_motion_duration_var = tk.DoubleVar(value=default_gripper)
        self.velocity_text = tk.StringVar()
        self.acceleration_text = tk.StringVar()
        self.grasp_descent_step_text = tk.StringVar()
        self.gripper_motion_duration_text = tk.StringVar()
        self.robot_status_text = tk.StringVar(value="Stopped")
        self.robot_detail_text = tk.StringVar(value="Launcher idle")
        self.robot_status_label: tk.Label | None = None
        self.collision_seen = False
        self.collision_active = False
        self.collision_status_text = tk.StringVar(value="Unknown")
        self.collision_detail_text = tk.StringVar(value="Waiting for collision monitor data.")
        self.collision_indicator_canvas: tk.Canvas | None = None
        self.collision_indicator_oval: int | None = None
        self.picked_count_text = tk.StringVar(value="0")
        self.pick_time_text = tk.StringVar(value="--")
        self.pick_time_detail_text = tk.StringVar(value="Waiting for detected nut pose.")
        self.robot_case_text = tk.StringVar(value="Idle")
        self.robot_case_detail_text = tk.StringVar(value="Bin-picking process not running.")
        self.error_count = 0
        self.error_count_text = tk.StringVar(value="0")
        self.error_log_widget: ScrolledText | None = None
        self.warning_until = 0.0
        self.warning_message = ""
        self.emergency_latched = False
        self.startup_in_progress = False
        self.startup_cancelled = False

        # Dashboard additions: active preset, cycle timing, simulator speed, pipeline pills.
        self.active_preset_text = tk.StringVar(value=f"Active preset: {DEFAULT_PRESET}")
        self.preset_buttons: dict[str, tk.Button] = {}
        self.cycle_time_text = tk.StringVar(value="--")
        self.cycle_detail_text = tk.StringVar(value="Detect, pick, place, reset.")
        self.picked_detail_text = tk.StringVar(value="Completed cycles this run.")
        self.rtf_text = tk.StringVar(value="--")
        self.rtf_detail_text = tk.StringVar(value="1.0 = real time.")
        self.cycle_start_time: float | None = None
        self.cycle_times: list[float] = []
        self.last_picked_count = 0
        self.retry_count = 0
        self.pipeline_pills: dict[str, tuple[tk.Label, str]] = {}
        self.closing = False
        # Initial task: LAUNCHER_TASK=pick_place|tighten|pick_tighten (default pick_place).
        initial_task = next((label for label, mode in TASK_MODES.items()
                             if mode == os.environ.get("LAUNCHER_TASK", "pick_place")), "Pick & place")
        self.task_label_var = tk.StringVar(value=initial_task)
        self.bolt_text = tk.StringVar(value="--")
        self.bolt_detail_text = tk.StringVar(value="Sim ground truth (bolt fixture).")

        self.process_configs = self._default_configs()
        self.processes: dict[str, ManagedProcess] = {}
        self.command_vars: dict[str, tk.StringVar] = {}
        self.process_status_texts: dict[str, tk.StringVar] = {}
        self.process_status_labels: dict[str, tk.Label] = {}
        self.process_states: dict[str, str] = {}
        self.log_widgets: dict[str, ScrolledText] = {}

        self._configure_styles()
        self._build_ui()
        self._on_motion_profile_changed()
        self._refresh_dynamic_commands()
        self._recompute_robot_status()

        self.root.after(100, self._drain_log_queue)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        threading.Thread(target=self._sim_speed_monitor_loop, daemon=True).start()

    def _sim_speed_monitor_loop(self) -> None:
        """Poll Gazebo's real-time factor every ~2 s while the simulation runs."""
        while not self.closing:
            value = None
            sim = self.processes.get("sim")
            if sim is not None and sim.is_running():
                for topic in GZ_STATS_TOPICS:
                    try:
                        completed = subprocess.run(
                            ["ign", "topic", "-e", "-t", topic, "-n", "1"],
                            capture_output=True, text=True, timeout=3.0, check=False,
                        )
                    except (OSError, subprocess.TimeoutExpired):
                        continue
                    for line in completed.stdout.splitlines():
                        if line.strip().startswith("real_time_factor:"):
                            try:
                                value = float(line.split(":", 1)[1])
                            except ValueError:
                                value = None
                            break
                    if value is not None:
                        break
            bolt_state = self._read_bolt_state() if (sim is not None and sim.is_running()) else None
            if not self.closing:
                self.root.after(0, lambda v=value: self._update_sim_speed(v))
                self.root.after(0, lambda s=bolt_state: self._update_bolt_thread(s))
            time.sleep(2.0)

    @staticmethod
    def _read_bolt_state() -> tuple[float, float] | str | None:
        """Bolt fixture ground truth from Gazebo: (turns tightened, nut gap to the seat [m]),
        "empty" if the bolt has no nut, or None if unavailable."""
        try:
            model = subprocess.run(["ign", "model", "-m", BOLT_FIXTURE_MODEL, "-l", "nut"],
                                   capture_output=True, text=True, timeout=3.0, check=False).stdout
            if "Name: nut" not in model:
                return "empty" if "Name: bolt" in subprocess.run(
                    ["ign", "model", "-m", BOLT_FIXTURE_MODEL, "-l", "bolt"],
                    capture_output=True, text=True, timeout=3.0, check=False).stdout else None
            lines = model.splitlines()
            index = next(i for i, line in enumerate(lines) if line.strip().startswith("- Pose ["))
            nut_bottom = float(lines[index + 1].strip(" []").split()[2])  # in the fixture frame
            joints = subprocess.run(["ign", "topic", "-e", "-t", BOLT_JOINT_STATE_TOPIC, "-n", "1"],
                                    capture_output=True, text=True, timeout=3.0, check=False).stdout
        except (OSError, subprocess.TimeoutExpired, StopIteration, ValueError, IndexError):
            return None
        angle = 0.0  # protobuf text output omits a field that is exactly zero
        if 'name: "thread"' in joints:
            for line in joints.split('name: "thread"', 1)[1].splitlines():
                if line.strip().startswith("position:"):
                    angle = float(line.split(":", 1)[1])
                    break
        return -angle / (2.0 * math.pi), nut_bottom - BOLT_SEAT_Z  # clockwise = tightening

    def _update_bolt_thread(self, state: tuple[float, float] | str | None) -> None:
        if state is None:
            self.bolt_text.set("--")
            self.bolt_detail_text.set("Sim ground truth (bolt fixture).")
            return
        if state == "empty":
            self.bolt_text.set("No nut")
            self.bolt_detail_text.set("Bolt is empty (sim truth).")
            return
        turns, gap = state
        self.bolt_text.set(f"{turns:.2f} turns")
        self.bolt_detail_text.set("Seated on the plate (sim truth)." if gap < 5e-5
                                  else f"{gap * 1000:.1f} mm above the seat (sim truth).")

    def _on_task_changed(self) -> None:
        self._refresh_dynamic_commands()
        task = self.task_label_var.get()
        running = self.processes.get("bin_picking") and self.processes["bin_picking"].is_owned_running()
        self._append_launcher_log(
            "bin_picking",
            f"Task set to '{task}' (task_mode={TASK_MODES[task]})"
            + (". Restart Bin Picking Nodes to apply it." if running else "; used at the next start."),
        )

    def _update_sim_speed(self, value: float | None) -> None:
        if value is None:
            self.rtf_text.set("--")
            self.rtf_detail_text.set("Shown while Gazebo runs.")
            return
        self.rtf_text.set(f"{value:.2f}x")
        if value < 0.2:
            self.rtf_detail_text.set("Much slower than real time.")
        elif value < 0.6:
            self.rtf_detail_text.set("Slower than real time.")
        else:
            self.rtf_detail_text.set("Near real time.")

    def _configure_styles(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL_BG)
        style.configure("AltPanel.TFrame", background=PANEL_ALT)
        style.configure("TLabel", background=BG, foreground=TEXT)
        style.configure("Muted.TLabel", background=BG, foreground=MUTED)
        style.configure("Panel.TLabel", background=PANEL_BG, foreground=TEXT)
        style.configure("PanelMuted.TLabel", background=PANEL_BG, foreground=MUTED)
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=PANEL_ALT, foreground=TEXT, padding=(14, 8))
        style.map("TNotebook.Tab", background=[("selected", PANEL_BG)], foreground=[("selected", TEXT)])
        style.configure("TEntry", fieldbackground=ENTRY_BG, foreground=ENTRY_FG, insertcolor=ENTRY_FG)
        style.configure("Treeview", background=PANEL_BG, foreground=TEXT, fieldbackground=PANEL_BG)

    def _build_ui(self) -> None:
        root_frame = tk.Frame(self.root, bg=BG, padx=12, pady=10)
        root_frame.pack(fill=tk.BOTH, expand=True)

        header = tk.Frame(root_frame, bg=PANEL_BG, padx=18, pady=10, highlightthickness=1, highlightbackground="#1F2937")
        header.pack(fill=tk.X, pady=(0, 6))

        title_col = tk.Frame(header, bg=PANEL_BG)
        title_col.pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Label(
            title_col,
            text="JAKA S12 Control Deck",
            bg=PANEL_BG,
            fg="#F8FAFC",
            font=("TkDefaultFont", 18, "bold"),
        ).pack(anchor="w")
        tk.Label(
            title_col,
            text="Gazebo simulation launcher (robot_mode=sim). The physical robot uses jaka_hardware_bringup.",
            bg=PANEL_BG,
            fg=MUTED,
            font=("TkDefaultFont", 10),
        ).pack(anchor="w", pady=(4, 0))

        pills = tk.Frame(title_col, bg=PANEL_BG)
        pills.pack(anchor="w", pady=(10, 0))
        for key, label in (("sim", "Gazebo"), ("bin_picking", "Bin picking"), ("aae", "AAE"), ("yolo", "YOLO")):
            bg, _, fg = STATUS_COLORS["stopped"]
            pill = tk.Label(pills, text=f"● {label}: Stopped", bg=bg, fg=fg,
                            font=("TkDefaultFont", 9, "bold"), padx=10, pady=3)
            pill.pack(side=tk.LEFT, padx=(0, 6))
            self.pipeline_pills[key] = (pill, label)
        tk.Label(
            pills,
            text=f"Mode: SIM    Nut model: {NUT_MODEL.upper()}    Task:",
            bg=PANEL_BG,
            fg="#CBD5E1",
            font=("TkDefaultFont", 9, "bold"),
        ).pack(side=tk.LEFT, padx=(10, 4))
        # Task selector: passed to the bin-picking node as task_mode at its next start.
        task_box = ttk.Combobox(pills, textvariable=self.task_label_var, values=list(TASK_MODES),
                                state="readonly", width=14)
        task_box.pack(side=tk.LEFT)
        task_box.bind("<<ComboboxSelected>>", lambda _event: self._on_task_changed())

        button_col = tk.Frame(header, bg=PANEL_BG)
        button_col.pack(side=tk.RIGHT)
        self._make_button(button_col, "Start All", START, START_ACTIVE, self.start_all, width=12).pack(side=tk.LEFT, padx=(0, 8))
        self._make_button(button_col, "Stop All", STOP, STOP_ACTIVE, self.stop_all, width=12).pack(side=tk.LEFT, padx=(0, 8))
        self._make_button(button_col, "EMERGENCY STOP", DANGER, DANGER_ACTIVE, self.emergency_stop, width=18).pack(side=tk.LEFT)

        info = tk.Frame(root_frame, bg=BG)
        info.pack(fill=tk.X, pady=(0, 6))
        conda_text = str(self.conda_exe) if self.conda_exe else "not found"
        tk.Label(
            info,
            text=f"Workspace: {ROOT_DIR}    |    Conda: {conda_text}",
            bg=BG,
            fg=MUTED,
            font=("TkDefaultFont", 9),
        ).pack(anchor="w")

        main_pane = ttk.Panedwindow(root_frame, orient=tk.VERTICAL)
        main_pane.pack(fill=tk.BOTH, expand=True)

        top_pane = tk.Frame(main_pane, bg=BG)
        bottom_pane = tk.Frame(main_pane, bg=BG)
        main_pane.add(top_pane, weight=3)
        main_pane.add(bottom_pane, weight=2)

        top_scroll_frame = tk.Frame(top_pane, bg=BG)
        top_scroll_frame.pack(fill=tk.BOTH, expand=True)

        top_canvas = tk.Canvas(
            top_scroll_frame,
            bg=BG,
            highlightthickness=0,
            bd=0,
        )
        top_scrollbar = tk.Scrollbar(top_scroll_frame, orient=tk.VERTICAL, command=top_canvas.yview)
        top_canvas.configure(yscrollcommand=top_scrollbar.set)
        top_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        top_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        controls_content = tk.Frame(top_canvas, bg=BG)
        controls_window = top_canvas.create_window((0, 0), window=controls_content, anchor="nw")

        def _update_controls_scrollregion(_event) -> None:
            top_canvas.configure(scrollregion=top_canvas.bbox("all"))

        def _resize_controls_width(event) -> None:
            top_canvas.itemconfigure(controls_window, width=event.width)

        controls_content.bind("<Configure>", _update_controls_scrollregion)
        top_canvas.bind("<Configure>", _resize_controls_width)

        dashboard = tk.Frame(controls_content, bg=BG)
        dashboard.pack(fill=tk.X, pady=(0, 10))
        dashboard.grid_columnconfigure(0, weight=1)
        dashboard.grid_columnconfigure(1, weight=1)
        dashboard.grid_rowconfigure(1, weight=1)

        status_card = tk.Frame(dashboard, bg=PANEL_BG, padx=16, pady=10, highlightthickness=1, highlightbackground="#1F2937")
        status_card.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        tk.Label(status_card, text="Robot Status", bg=PANEL_BG, fg=MUTED, font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        self.robot_status_label = tk.Label(
            status_card,
            textvariable=self.robot_status_text,
            bg="#5B4A11",
            fg="#FFF6D9",
            font=("TkDefaultFont", 16, "bold"),
            padx=18,
            pady=6,
        )
        self.robot_status_label.pack(anchor="w", pady=(10, 8))
        tk.Label(
            status_card,
            textvariable=self.robot_detail_text,
            bg=PANEL_BG,
            fg=TEXT,
            wraplength=520,
            justify=tk.LEFT,
            font=("TkDefaultFont", 10),
        ).pack(anchor="w")

        motion_card = tk.Frame(dashboard, bg=PANEL_BG, padx=16, pady=10, highlightthickness=1, highlightbackground="#1F2937")
        motion_card.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        tk.Label(motion_card, text="Motion Profile", bg=PANEL_BG, fg=MUTED, font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        tk.Label(
            motion_card,
            text="Simulation presets. Safe = real-robot limits; faster presets are rejected by the node in robot_mode=real.",
            bg=PANEL_BG,
            fg=TEXT,
            wraplength=560,
            justify=tk.LEFT,
            font=("TkDefaultFont", 9),
        ).pack(anchor="w", pady=(4, 8))

        self._build_slider_row(motion_card, "Velocity", self.velocity_var, self.velocity_text)
        self._build_slider_row(motion_card, "Acceleration", self.acceleration_var, self.acceleration_text)
        self._build_slider_row(
            motion_card,
            "Descent step*",
            self.grasp_descent_step_var,
            self.grasp_descent_step_text,
            from_=0.001,
            to=0.020,
            resolution=0.001,
        )
        self._build_slider_row(
            motion_card,
            "Gripper Time",
            self.gripper_motion_duration_var,
            self.gripper_motion_duration_text,
            from_=0.05,
            to=1.50,
            resolution=0.05,
        )

        tk.Label(
            motion_card,
            text="* Descent step only affects the physical robot; in simulation the final descent is one smooth move.",
            bg=PANEL_BG,
            fg=MUTED,
            font=("TkDefaultFont", 8),
        ).pack(anchor="w", pady=(2, 0))

        preset_row = tk.Frame(motion_card, bg=PANEL_BG)
        preset_row.pack(fill=tk.X, pady=(8, 0))
        for index, name in enumerate(MOTION_PRESETS):
            bg, active_bg = PRESET_COLORS[name]
            button = self._make_button(
                preset_row, name, bg, active_bg,
                lambda preset=name: self.set_motion_profile(*MOTION_PRESETS[preset]),
                width=9,
            )
            button.pack(side=tk.LEFT, padx=(0 if index == 0 else 6, 0))
            self.preset_buttons[name] = button
        self._make_button(preset_row, "Apply Live", START, START_ACTIVE, self.apply_motion_profile, width=11).pack(side=tk.RIGHT)
        tk.Label(
            motion_card,
            textvariable=self.active_preset_text,
            bg=PANEL_BG,
            fg="#C4B5FD",
            font=("TkDefaultFont", 9, "bold"),
        ).pack(anchor="w", pady=(6, 0))

        # Live telemetry: one compact row of tiles so everything fits on short (e.g. 1920x884 VNC) screens.
        telemetry_card = tk.Frame(dashboard, bg=PANEL_BG, padx=14, pady=10, highlightthickness=1, highlightbackground="#1F2937")
        telemetry_card.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
        tk.Label(
            telemetry_card,
            text="Live Telemetry",
            bg=PANEL_BG,
            fg=MUTED,
            font=("TkDefaultFont", 10, "bold"),
        ).grid(row=0, column=0, columnspan=8, sticky="w", pady=(0, 6))

        tiles = (
            # (column, title, value variable, detail variable, value colour, column weight)
            (0, "Current Case", self.robot_case_text, self.robot_case_detail_text, "#F8FAFC", 3),
            (1, "Picked Nuts", self.picked_count_text, self.picked_detail_text, "#F8FAFC", 2),
            (2, "Cycle Time", self.cycle_time_text, self.cycle_detail_text, "#F8FAFC", 2),
            (3, "Detect -> Pick", self.pick_time_text, self.pick_time_detail_text, "#F8FAFC", 2),
            (4, "Sim Speed", self.rtf_text, self.rtf_detail_text, "#F8FAFC", 2),
            (6, "Errors", self.error_count_text, None, "#FCA5A5", 2),
            (7, "Bolt Thread", self.bolt_text, self.bolt_detail_text, "#F8FAFC", 2),
        )
        for column, title, value_var, detail_var, colour, weight in tiles:
            telemetry_card.grid_columnconfigure(column, weight=weight, uniform="tiles")
            tile = self._build_tile(telemetry_card, column, title)
            tk.Label(tile, textvariable=value_var, bg=PANEL_ALT, fg=colour,
                     wraplength=260 if column == 0 else 0, justify=tk.LEFT,
                     font=("TkDefaultFont", 13 if column == 0 else 18, "bold")).pack(anchor="w", pady=(4, 2))
            if detail_var is not None:
                tk.Label(tile, textvariable=detail_var, bg=PANEL_ALT, fg=TEXT, wraplength=170 if column else 300,
                         justify=tk.LEFT, font=("TkDefaultFont", 8)).pack(anchor="w")
            else:
                tk.Label(tile, text="Warnings and failures.", bg=PANEL_ALT, fg=TEXT,
                         font=("TkDefaultFont", 8)).pack(anchor="w")

        telemetry_card.grid_columnconfigure(5, weight=2, uniform="tiles")
        collision_box = self._build_tile(telemetry_card, 5, "Collision")
        collision_row = tk.Frame(collision_box, bg=PANEL_ALT)
        collision_row.pack(anchor="w", pady=(6, 2))
        self.collision_indicator_canvas = tk.Canvas(
            collision_row,
            width=18,
            height=18,
            bg=PANEL_ALT,
            highlightthickness=0,
            bd=0,
        )
        self.collision_indicator_canvas.pack(side=tk.LEFT, padx=(0, 8))
        self.collision_indicator_oval = self.collision_indicator_canvas.create_oval(2, 2, 16, 16, fill="#64748B", outline="")
        tk.Label(
            collision_row,
            textvariable=self.collision_status_text,
            bg=PANEL_ALT,
            fg=TEXT,
            font=("TkDefaultFont", 13, "bold"),
        ).pack(side=tk.LEFT)
        tk.Label(
            collision_box,
            textvariable=self.collision_detail_text,
            bg=PANEL_ALT,
            fg=TEXT,
            wraplength=210,
            justify=tk.LEFT,
            font=("TkDefaultFont", 8),
        ).pack(anchor="w")

        process_card = tk.Frame(controls_content, bg=PANEL_BG, padx=16, pady=10, highlightthickness=1, highlightbackground="#1F2937")
        process_card.pack(fill=tk.X, pady=(0, 14))
        tk.Label(process_card, text="Processes", bg=PANEL_BG, fg=MUTED, font=("TkDefaultFont", 10, "bold")).pack(anchor="w", pady=(0, 10))

        table = tk.Frame(process_card, bg=PANEL_BG)
        table.pack(fill=tk.X)
        table.grid_columnconfigure(2, weight=1)

        headers = ("Process", "Status", "Command", "Controls")
        for idx, title in enumerate(headers):
            tk.Label(table, text=title, bg=PANEL_BG, fg="#CBD5E1", font=("TkDefaultFont", 10, "bold")).grid(
                row=0, column=idx, sticky="w", padx=(0, 8), pady=(0, 8)
            )

        for row, config in enumerate(self.process_configs, start=1):
            self.processes[config.key] = ManagedProcess(config, self.log_queue)
            self.command_vars[config.key] = tk.StringVar(value=config.command)
            self.process_status_texts[config.key] = tk.StringVar(value="Stopped")
            self.process_states[config.key] = "stopped"

            tk.Label(table, text=config.title, bg=PANEL_BG, fg=TEXT, font=("TkDefaultFont", 10, "bold")).grid(
                row=row, column=0, sticky="w", padx=(0, 8), pady=6
            )

            status_label = tk.Label(
                table,
                textvariable=self.process_status_texts[config.key],
                bg=STATUS_COLORS["stopped"][0],
                fg=STATUS_COLORS["stopped"][2],
                font=("TkDefaultFont", 9, "bold"),
                padx=10,
                pady=4,
            )
            status_label.grid(row=row, column=1, sticky="w", padx=(0, 8), pady=6)
            self.process_status_labels[config.key] = status_label

            entry = tk.Entry(
                table,
                textvariable=self.command_vars[config.key],
                bg=ENTRY_BG,
                fg=ENTRY_FG,
                insertbackground=ENTRY_FG,
                relief=tk.FLAT,
                highlightthickness=1,
                highlightbackground="#1F2937",
                highlightcolor="#2563EB",
            )
            entry.grid(row=row, column=2, sticky="ew", pady=6)

            controls = tk.Frame(table, bg=PANEL_BG)
            controls.grid(row=row, column=3, sticky="w", padx=(8, 0), pady=6)
            self._make_button(controls, "Start", START, START_ACTIVE, lambda key=config.key: self.start_process(key), width=9).pack(side=tk.LEFT)
            self._make_button(controls, "Stop", STOP, STOP_ACTIVE, lambda key=config.key: self.stop_process(key), width=9).pack(side=tk.LEFT, padx=(8, 0))

        log_notebook = ttk.Notebook(bottom_pane)
        log_notebook.pack(fill=tk.BOTH, expand=True)

        error_tab = tk.Frame(log_notebook, bg=PANEL_ALT)
        log_notebook.add(error_tab, text="Errors")
        error_toolbar = tk.Frame(error_tab, bg=PANEL_ALT, padx=10, pady=8)
        error_toolbar.pack(fill=tk.X)
        tk.Label(
            error_toolbar,
            text="Aggregated warnings, errors, failed exits, and collision alerts from all launcher-managed processes.",
            bg=PANEL_ALT,
            fg=MUTED,
            font=("TkDefaultFont", 9),
        ).pack(side=tk.LEFT)
        self._make_button(error_toolbar, "Clear Errors", "#334155", "#1E293B", self.clear_errors, width=12).pack(side=tk.RIGHT)

        self.error_log_widget = ScrolledText(
            error_tab,
            wrap=tk.WORD,
            font=("TkFixedFont", 10),
            bg="#020617",
            fg="#FECACA",
            insertbackground="#FECACA",
            selectbackground="#991B1B",
            relief=tk.FLAT,
            padx=10,
            pady=10,
        )
        self.error_log_widget.pack(fill=tk.BOTH, expand=True)
        self.error_log_widget.configure(state=tk.DISABLED)

        for config in self.process_configs:
            tab = tk.Frame(log_notebook, bg=PANEL_ALT)
            log_notebook.add(tab, text=config.title)
            log_widget = ScrolledText(
                tab,
                wrap=tk.WORD,
                font=("TkFixedFont", 10),
                bg="#020617",
                fg="#E2E8F0",
                insertbackground="#E2E8F0",
                selectbackground="#1D4ED8",
                relief=tk.FLAT,
                padx=10,
                pady=10,
            )
            log_widget.pack(fill=tk.BOTH, expand=True)
            log_widget.configure(state=tk.DISABLED)
            self.log_widgets[config.key] = log_widget

        self._update_collision_indicator()

        def _place_sash() -> None:
            # winfo_height() is 1 until the window is mapped; the old after(0) call therefore
            # always fell back to 420 px and clipped the presets and the whole telemetry row.
            self.root.update_idletasks()
            pane_height = main_pane.winfo_height()
            if pane_height < 200:
                self.root.after(100, _place_sash)
                return
            main_pane.sashpos(0, int(pane_height * 0.70))

        self.root.after(150, _place_sash)

    def _build_slider_row(
        self,
        parent: tk.Widget,
        label: str,
        variable: tk.DoubleVar,
        text_var: tk.StringVar,
        from_: float = 0.10,
        to: float = 1.00,
        resolution: float = 0.05,
    ) -> None:
        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill=tk.X, pady=4)
        tk.Label(row, text=label, bg=PANEL_BG, fg=TEXT, width=12, anchor="w").pack(side=tk.LEFT)
        scale = tk.Scale(
            row,
            from_=from_,
            to=to,
            resolution=resolution,
            orient=tk.HORIZONTAL,
            showvalue=False,
            variable=variable,
            command=lambda _value: self._on_motion_profile_changed(),
            bg=PANEL_BG,
            fg=TEXT,
            troughcolor="#1F2937",
            highlightthickness=0,
            activebackground=ACCENT,
            length=230,
        )
        scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 12))
        tk.Label(row, textvariable=text_var, bg=PANEL_BG, fg="#F8FAFC", width=6, anchor="e", font=("TkDefaultFont", 10, "bold")).pack(side=tk.RIGHT)

    def _build_tile(self, parent: tk.Widget, column: int, title: str) -> tk.Frame:
        tile = tk.Frame(parent, bg=PANEL_ALT, padx=10, pady=8, highlightthickness=1, highlightbackground="#1F2937")
        tile.grid(row=1, column=column, sticky="nsew", padx=(0 if column == 0 else 6, 0))
        tk.Label(tile, text=title, bg=PANEL_ALT, fg=MUTED, font=("TkDefaultFont", 9, "bold")).pack(anchor="w")
        return tile

    def _make_button(self, parent: tk.Widget, text: str, bg: str, active_bg: str, command, width: int = 10) -> tk.Button:
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg=bg,
            fg="#F8FAFC",
            activebackground=active_bg,
            activeforeground="#F8FAFC",
            relief=tk.FLAT,
            bd=0,
            padx=12,
            pady=8,
            width=width,
            cursor="hand2",
            font=("TkDefaultFont", 10, "bold"),
        )

    def _update_collision_indicator(self) -> None:
        if self.collision_indicator_canvas is None or self.collision_indicator_oval is None:
            return

        if not self.collision_seen:
            color = "#64748B"
        elif self.collision_active:
            color = "#EF4444"
        else:
            color = "#22C55E"

        self.collision_indicator_canvas.itemconfigure(self.collision_indicator_oval, fill=color)

    def _reset_bin_picking_telemetry(self) -> None:
        self.collision_seen = False
        self.collision_active = False
        self.collision_status_text.set("Unknown")
        self.collision_detail_text.set("Waiting for collision monitor data.")
        self.picked_count_text.set("0")
        self.pick_time_text.set("--")
        self.pick_time_detail_text.set("Waiting for first detected nut pose.")
        self.robot_case_text.set("Starting")
        self.robot_case_detail_text.set("Bin-picking process launch requested.")
        self.cycle_start_time = None
        self.cycle_times = []
        self.last_picked_count = 0
        self.retry_count = 0
        self.cycle_time_text.set("--")
        self.cycle_detail_text.set("Detect, pick, place, reset.")
        self.picked_detail_text.set("Completed cycles this run.")
        self._update_collision_indicator()

    def _set_bin_picking_offline(self) -> None:
        self.collision_seen = False
        self.collision_active = False
        self.collision_status_text.set("Offline")
        self.collision_detail_text.set("Bin-picking stack is not running.")
        self.pick_time_text.set("--")
        self.pick_time_detail_text.set("Bin-picking stack is not running.")
        self.robot_case_text.set("Stopped")
        self.robot_case_detail_text.set("Bin-picking process not running.")
        self._update_collision_indicator()

    def clear_errors(self) -> None:
        self.error_count = 0
        self.error_count_text.set("0")
        if self.error_log_widget is None:
            return
        self.error_log_widget.configure(state=tk.NORMAL)
        self.error_log_widget.delete("1.0", tk.END)
        self.error_log_widget.configure(state=tk.DISABLED)

    def _append_error(self, line: str) -> None:
        self.error_count += 1
        self.error_count_text.set(str(self.error_count))
        if self.error_log_widget is None:
            return
        self.error_log_widget.configure(state=tk.NORMAL)
        self.error_log_widget.insert(tk.END, line)
        self.error_log_widget.see(tk.END)
        self.error_log_widget.configure(state=tk.DISABLED)

    def _is_error_line(self, key: str, line: str) -> bool:
        if LAUNCHER_STATUS_PREFIX in line:
            return False

        lowered = line.lower()
        if "traceback" in lowered:
            return True
        if "[error]" in lowered or "[warn]" in lowered:
            return True
        if "collision alert" in lowered:
            return True
        if "[process exited with code " in lowered and "[process exited with code 0]" not in lowered:
            return True
        if "[launcher]" in lowered and ("failed" in lowered or "warning" in lowered or "emergency stop" in lowered):
            return True
        if key in CORE_PROCESS_KEYS and "failed" in lowered and "[launcher_status]" not in lowered:
            return True
        return False

    def _handle_launcher_status_line(self, key: str, line: str) -> None:
        marker_index = line.find(LAUNCHER_STATUS_PREFIX)
        if marker_index < 0:
            return

        payload = line[marker_index + len(LAUNCHER_STATUS_PREFIX):].strip()
        fields: dict[str, str] = {}
        for part in payload.split(";"):
            part = part.strip()
            if not part or "=" not in part:
                continue
            name, value = part.split("=", 1)
            fields[name.strip()] = value.strip()

        if "robot_case" in fields:
            self.robot_case_text.set(fields["robot_case"])
            detail = fields.get("detail", "")
            if detail and detail != "-":
                self.robot_case_detail_text.set(detail)
            elif key == "bin_picking":
                self.robot_case_detail_text.set("Launcher status update received.")

        robot_case = fields.get("robot_case", "")
        if robot_case.startswith("Sequence 0") and self.cycle_start_time is None:
            self.cycle_start_time = time.time()  # retries stay inside the same cycle
        if robot_case == "Request new detection":
            self.retry_count += 1

        if "picked_count" in fields:
            self.picked_count_text.set(fields["picked_count"])
            try:
                picked = int(fields["picked_count"])
            except ValueError:
                picked = self.last_picked_count
            if picked > self.last_picked_count and self.cycle_start_time is not None:
                self.cycle_times.append(time.time() - self.cycle_start_time)
                self.cycle_start_time = None
                average = sum(self.cycle_times) / len(self.cycle_times)
                self.cycle_time_text.set(f"{self.cycle_times[-1]:.1f}s")
                self.cycle_detail_text.set(
                    f"Last cycle. Avg {average:.1f}s ({len(self.cycle_times)} cycles)."
                )
            self.last_picked_count = picked
            self.picked_detail_text.set(
                f"Completed. Retries: {self.retry_count}."
            )

        if "pick_time_sec" in fields or "pick_timer_state" in fields:
            pick_time_state = fields.get("pick_timer_state", "").strip()
            pick_time_sec = None
            if "pick_time_sec" in fields:
                try:
                    pick_time_sec = float(fields["pick_time_sec"])
                except ValueError:
                    pick_time_sec = None

            if pick_time_state == "running" and pick_time_sec is not None:
                self.pick_time_text.set(f"{pick_time_sec:.2f}s")
                self.pick_time_detail_text.set("Detect-to-pick timer is running for the current nut.")
            elif pick_time_state == "last" and pick_time_sec is not None and pick_time_sec > 0.0:
                self.pick_time_text.set(f"{pick_time_sec:.2f}s")
                self.pick_time_detail_text.set("Last completed detect-to-pick time.")
            elif pick_time_state == "idle":
                self.pick_time_text.set("--")
                self.pick_time_detail_text.set("Waiting for detected nut pose.")

        if "collision" in fields:
            self.collision_seen = True
            self.collision_active = fields["collision"] in {"1", "true", "True", "yes", "on"}
            self.collision_status_text.set("Collision" if self.collision_active else "Clear")
            detail = fields.get("detail", "")
            if detail and detail != "-":
                self.collision_detail_text.set(detail)
            elif self.collision_active:
                self.collision_detail_text.set("Collision detected in current robot state.")
            else:
                self.collision_detail_text.set("Collision monitor reports clear.")
            self._update_collision_indicator()

    def _default_configs(self) -> list[ProcessConfig]:
        sim_launch = detect_sim_launch()
        bin_launch = detect_bin_picking_launch()
        yolo_script = detect_yolo_script()
        return [
            ProcessConfig(
                key="sim",
                title="Simulation",
                command=build_ros_launch_command("bin_packing_pkg", sim_launch),
                cleanup_patterns=(
                    f"ros2 launch bin_packing_pkg {sim_launch}",
                    "ign gazebo server",
                    "gz sim",
                    "move_group",
                    "rviz2",
                    "robot_state_publisher",
                    "parameter_bridge",
                    "depth_image_proc",
                    "spawner joint_state_broadcaster",
                    "spawner jaka_s12_controller",
                    "spawner custom_gripper_controller",
                    "ros_gz_sim create",
                ),
            ),
            ProcessConfig(
                key="bin_picking",
                title="Bin Picking Nodes",
                command=self._build_bin_picking_command(bin_launch),
                cleanup_patterns=(
                    f"ros2 launch bin_packing_pkg {bin_launch}",
                    "bin_picking_node_v5_to_control_jaka_s12",
                    "bin_picking_node_v7_to_control_jaka_s12",
                ),
            ),
            ProcessConfig(
                key="aae",
                title="AAE Inference",
                command=self._build_aae_command(),
                cleanup_patterns=(
                    "python3 aae_ros_infer_sim_v4.py",
                ),
            ),
            ProcessConfig(
                key="yolo",
                title="YOLOv12 Seg",
                command=self._build_yolo_command(yolo_script),
                cleanup_patterns=(
                    "python3 yolo_ros_infer_sim_v11_to_control_jaka_s12.py",
                    "python3 yolo_ros_infer_sim_v11.py",
                ),
            ),
        ]

    def _build_bin_picking_command(self, launch_file: str | None = None) -> str:
        bin_launch = launch_file or detect_bin_picking_launch()
        return build_ros_launch_command(
            "bin_packing_pkg",
            bin_launch,
            {
                "use_sim_time": "true",
                "robot_mode": LAUNCHER_ROBOT_MODE,
                "task_mode": TASK_MODES[self.task_label_var.get()],
                "arm_velocity_scaling": f"{self.velocity_var.get():.2f}",
                "arm_acceleration_scaling": f"{self.acceleration_var.get():.2f}",
                "grasp_descent_step": f"{self.grasp_descent_step_var.get():.4f}",
                "gripper_motion_duration": f"{self.gripper_motion_duration_var.get():.2f}",
            },
        )

    def _build_aae_command(self) -> str:
        return build_conda_python_command(
            self.conda_exe,
            "aae_py37_tf26",
            AAE_WORKSPACE_DIR,
            "aae_ros_infer_sim_v4.py",
            {
                "AAE_SERVER_PORT": str(self.aae_port),
                "AE_WORKSPACE_PATH": str(AAE_WORKSPACE_DIR),
                "AAE_NUT_MODEL": NUT_MODEL,
            },
        )

    def _build_yolo_command(self, script_name: str | None = None) -> str:
        yolo_script = script_name or detect_yolo_script()
        return build_conda_python_command(
            self.conda_exe,
            "yolov12_py10",
            ROOT_DIR / "yolov12_seg_infer",
            yolo_script,
            {"AAE_SERVER_PORT": str(self.aae_port), "NUT_MODEL": NUT_MODEL},
        )

    def _append_launcher_log(self, key: str, message: str) -> None:
        if not message.endswith("\n"):
            message += "\n"
        self.log_queue.put(("log", key, f"[launcher] {message}"))

    def _run_ros_cli(self, command: str, timeout_sec: float = 15.0) -> tuple[int, str]:
        return run_shell_capture(build_ros_shell_command(command), timeout_sec=timeout_sec)

    def _startup_should_abort(self) -> bool:
        return self.startup_cancelled or self.emergency_latched

    def _simulation_stack_ready(self) -> bool:
        rc, controllers_output = self._run_ros_cli("ros2 control list_controllers", timeout_sec=15.0)
        if rc != 0:
            return False

        required_controllers = (
            "joint_state_broadcaster",
            "jaka_s12_controller",
            "custom_gripper_controller",
        )
        if not all(controller_is_active(controllers_output, name) for name in required_controllers):
            return False

        rc, nodes_output = self._run_ros_cli("ros2 node list", timeout_sec=10.0)
        if rc != 0:
            return False

        required_nodes = ("/move_group", "/controller_manager", "/robot_state_publisher")
        return all(node in nodes_output for node in required_nodes)

    def _aae_server_ready(self) -> bool:
        try:
            with urlopen(f"http://127.0.0.1:{self.aae_port}/health", timeout=1.5) as response:
                return response.status == 200
        except URLError:
            return False
        except OSError:
            return False

    def _wait_for_condition(self, condition, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self._startup_should_abort():
                return False
            if condition():
                return True
            time.sleep(STARTUP_POLL_INTERVAL_SEC)
        return False

    def _sleep_with_abort(self, duration_sec: float) -> bool:
        deadline = time.time() + duration_sec
        while time.time() < deadline:
            if self._startup_should_abort():
                return False
            time.sleep(min(STARTUP_POLL_INTERVAL_SEC, max(0.0, deadline - time.time())))
        return True

    def _start_process_from_sequence(self, key: str, timeout_sec: float = PROCESS_START_TIMEOUT_SEC) -> bool:
        if self._startup_should_abort():
            return False
        started = threading.Event()

        def _start() -> None:
            self.start_process(key)
            started.set()

        self.root.after(0, _start)
        started.wait(timeout=2.0)

        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self._startup_should_abort():
                return False
            if self.processes[key].is_running():
                return True
            time.sleep(0.25)
        return self.processes[key].is_running()

    def _report_startup_failure(self, key: str, message: str) -> None:
        self.warning_until = time.time() + WARNING_WINDOW_SEC
        self.warning_message = message
        self._append_launcher_log(key, message)
        self.root.after(0, self._recompute_robot_status)

    def _start_all_sequence(self) -> None:
        try:
            self._append_launcher_log("sim", "Starting simulation stack.")
            if not self._start_process_from_sequence("sim"):
                if self._startup_should_abort():
                    self._append_launcher_log("sim", "Startup cancelled before simulation finished starting.")
                    return
                self._report_startup_failure("sim", "Simulation process did not stay up after start.")
                return

            self._append_launcher_log("sim", "Waiting for simulation controllers and MoveIt to become ready.")
            if not self._wait_for_condition(self._simulation_stack_ready, SIM_READY_TIMEOUT_SEC):
                if self._startup_should_abort():
                    self._append_launcher_log("sim", "Startup cancelled while waiting for simulation readiness.")
                    return
                self._report_startup_failure(
                    "sim",
                    "Simulation stack did not become ready in time. Check controllers, MoveIt, and ros_gz_bridge.",
                )
                return

            self._append_launcher_log("bin_picking", "Starting bin-picking nodes with simulation time enabled.")
            if not self._start_process_from_sequence("bin_picking"):
                if self._startup_should_abort():
                    self._append_launcher_log("bin_picking", "Startup cancelled before bin-picking finished starting.")
                    return
                self._report_startup_failure("bin_picking", "Bin-picking process did not stay up after start.")
                return

            if not self._sleep_with_abort(2.0):
                self._append_launcher_log("bin_picking", "Startup cancelled during post-launch settle time.")
                return

            self._append_launcher_log("aae", f"Starting AAE inference on port {self.aae_port}.")
            if not self._start_process_from_sequence("aae"):
                if self._startup_should_abort():
                    self._append_launcher_log("aae", "Startup cancelled before AAE finished starting.")
                    return
                self._report_startup_failure("aae", "AAE process did not stay up after start.")
                return

            self._append_launcher_log("aae", "Waiting for AAE health endpoint.")
            if not self._wait_for_condition(self._aae_server_ready, AAE_READY_TIMEOUT_SEC):
                if self._startup_should_abort():
                    self._append_launcher_log("aae", "Startup cancelled while waiting for AAE readiness.")
                    return
                self._report_startup_failure(
                    "aae",
                    f"AAE server did not report healthy on port {self.aae_port}.",
                )
                return

            self._append_launcher_log("yolo", "Starting YOLO detection after AAE is healthy.")
            if not self._start_process_from_sequence("yolo"):
                if self._startup_should_abort():
                    self._append_launcher_log("yolo", "Startup cancelled before YOLO finished starting.")
                    return
                self._report_startup_failure("yolo", "YOLO process did not stay up after start.")
                return

            self._append_launcher_log(
                "yolo",
                "Launcher startup sequence completed. The stack now matches the working manual order.",
            )
            self.root.after(0, self._recompute_robot_status)
        finally:
            self.startup_in_progress = False

    def _refresh_dynamic_commands(self) -> None:
        for config in self.process_configs:
            if config.key == "bin_picking":
                self.command_vars[config.key].set(self._build_bin_picking_command())
            elif config.key == "aae":
                self.command_vars[config.key].set(self._build_aae_command())
            elif config.key == "yolo":
                self.command_vars[config.key].set(self._build_yolo_command())

    def _on_motion_profile_changed(self) -> None:
        self.velocity_text.set(f"{self.velocity_var.get():.2f}")
        self.acceleration_text.set(f"{self.acceleration_var.get():.2f}")
        self.grasp_descent_step_text.set(f"{self.grasp_descent_step_var.get():.4f}")
        self.gripper_motion_duration_text.set(f"{self.gripper_motion_duration_var.get():.2f}")
        current = (
            self.velocity_var.get(),
            self.acceleration_var.get(),
            self.grasp_descent_step_var.get(),
            self.gripper_motion_duration_var.get(),
        )
        active = next(
            (name for name, values in MOTION_PRESETS.items()
             if all(abs(a - b) < 1e-6 for a, b in zip(current, values))),
            None,
        )
        self.active_preset_text.set(f"Active preset: {active}" if active else "Active preset: Custom (sliders)")
        for name, button in self.preset_buttons.items():
            button.configure(relief=tk.SUNKEN if name == active else tk.FLAT, bd=3 if name == active else 0)
        if hasattr(self, "command_vars") and "bin_picking" in self.command_vars:
            self._refresh_dynamic_commands()
        self._recompute_robot_status()

    def set_motion_profile(
        self,
        velocity: float,
        acceleration: float,
        grasp_descent_step: float | None = None,
        gripper_motion_duration: float | None = None,
    ) -> None:
        self.velocity_var.set(velocity)
        self.acceleration_var.set(acceleration)
        if grasp_descent_step is not None:
            self.grasp_descent_step_var.set(grasp_descent_step)
        if gripper_motion_duration is not None:
            self.gripper_motion_duration_var.set(gripper_motion_duration)
        self._on_motion_profile_changed()

    def apply_motion_profile(self) -> None:
        self._refresh_dynamic_commands()
        velocity = self.velocity_var.get()
        acceleration = self.acceleration_var.get()
        grasp_descent_step = self.grasp_descent_step_var.get()
        gripper_motion_duration = self.gripper_motion_duration_var.get()
        self._append_log(
            "bin_picking",
            "[launcher] motion profile selected: "
            f"velocity={velocity:.2f}, acceleration={acceleration:.2f}, "
            f"grasp_descent_step={grasp_descent_step:.4f}, gripper_motion_duration={gripper_motion_duration:.2f}\n",
        )

        bin_process = self.processes.get("bin_picking")
        owned_running = (
            bin_process is not None and bin_process.process is not None and bin_process.process.poll() is None
        )
        if bin_process is not None and not owned_running and bin_process.is_running():
            # A matching node exists but was NOT started by this launcher (e.g. the real-robot
            # bringup). Never push simulation motion settings into it.
            self.warning_until = time.time() + WARNING_WINDOW_SEC
            self.warning_message = "Apply Live refused: bin-picking node was not started by this launcher"
            self._append_log(
                "bin_picking",
                "[launcher] Apply Live refused: a bin-picking node is running that this launcher did not start. "
                "Simulation presets are never applied to externally launched (possibly real-robot) nodes.\n",
            )
            self._recompute_robot_status()
            return

        if not owned_running:
            self.robot_detail_text.set(
                "Motion profile prepared for next start: "
                f"velocity={velocity:.2f}, acceleration={acceleration:.2f}, "
                f"grasp_descent_step={grasp_descent_step:.4f}, gripper_motion_duration={gripper_motion_duration:.2f}."
            )
            self._recompute_robot_status()
            return

        threading.Thread(target=self._apply_motion_profile_live, daemon=True).start()

    def _apply_motion_profile_live(self) -> None:
        velocity = self.velocity_var.get()
        acceleration = self.acceleration_var.get()
        grasp_descent_step = self.grasp_descent_step_var.get()
        gripper_motion_duration = self.gripper_motion_duration_var.get()
        node_name = BIN_PICKING_NODE_NAME

        # Interlock: only push settings into a node that declares itself robot_mode=sim.
        rc, mode_output = self._run_ros_cli(
            f"ros2 param get {shlex.quote(node_name)} robot_mode", timeout_sec=10.0
        )
        if rc != 0 or f"String value is: {LAUNCHER_ROBOT_MODE}" not in mode_output:
            self.warning_until = time.time() + WARNING_WINDOW_SEC
            self.warning_message = "Apply Live refused: target node is not robot_mode=sim"
            self.log_queue.put(
                (
                    "log",
                    "bin_picking",
                    "[launcher] Apply Live refused: could not confirm "
                    f"{node_name} robot_mode={LAUNCHER_ROBOT_MODE}. Output: {mode_output.strip() or '<empty>'}\n",
                )
            )
            self.root.after(0, self._recompute_robot_status)
            return

        command = build_ros_shell_command(
            f"ros2 param set {shlex.quote(node_name)} arm_velocity_scaling {velocity:.2f} && "
            f"ros2 param set {shlex.quote(node_name)} arm_acceleration_scaling {acceleration:.2f} && "
            f"ros2 param set {shlex.quote(node_name)} grasp_descent_step {grasp_descent_step:.4f} && "
            f"ros2 param set {shlex.quote(node_name)} gripper_motion_duration {gripper_motion_duration:.2f}"
        )
        self.log_queue.put(("log", "bin_picking", f"$ {command}\n"))
        completed = subprocess.run(
            ["/bin/bash", "-lc", command],
            cwd=ROOT_DIR,
            text=True,
            capture_output=True,
        )
        if completed.stdout:
            self.log_queue.put(("log", "bin_picking", completed.stdout))
        if completed.stderr:
            self.log_queue.put(("log", "bin_picking", completed.stderr))

        if completed.returncode == 0:
            self.robot_detail_text.set(
                "Applied live motion profile: "
                f"velocity={velocity:.2f}, acceleration={acceleration:.2f}, "
                f"grasp_descent_step={grasp_descent_step:.4f}, gripper_motion_duration={gripper_motion_duration:.2f}."
            )
        else:
            self.warning_until = time.time() + WARNING_WINDOW_SEC
            self.warning_message = "Failed to apply live motion profile"
            self.log_queue.put(("log", "bin_picking", "[launcher] live motion profile update failed\n"))
        self.root.after(0, self._recompute_robot_status)

    def start_process(self, key: str) -> None:
        if key == "bin_picking":
            self._refresh_dynamic_commands()
            self._reset_bin_picking_telemetry()

        command = self.command_vars[key].get().strip()
        if not command:
            messagebox.showerror("Empty command", f"No command configured for {key}.")
            return

        if self.emergency_latched:
            self.emergency_latched = False
            self.warning_until = 0.0
            self.warning_message = ""

        self.processes[key].start(command)
        self._recompute_robot_status()

    def stop_process(self, key: str) -> None:
        self.processes[key].stop()
        self._recompute_robot_status()

    def start_all(self) -> None:
        if self.startup_in_progress:
            self._append_launcher_log("sim", "Start All is already in progress.")
            return
        self.emergency_latched = False
        self.startup_cancelled = False
        self.startup_in_progress = True
        threading.Thread(target=self._start_all_sequence, daemon=True).start()

    def stop_all(self) -> None:
        self.startup_cancelled = True
        for key in ("yolo", "aae", "bin_picking", "sim"):
            self.processes[key].stop()
        self._recompute_robot_status()

    def emergency_stop(self) -> None:
        self.emergency_latched = True
        self.startup_cancelled = True
        self.warning_until = time.time() + WARNING_WINDOW_SEC
        self.warning_message = "Emergency stop issued. Managed processes were killed immediately."
        for key in ("yolo", "aae", "bin_picking", "sim"):
            self.processes[key].stop(immediate=True)
            self.log_queue.put(("log", key, "[launcher] EMERGENCY STOP issued\n"))
        self._recompute_robot_status()

    def _append_log(self, key: str, line: str) -> None:
        widget = self.log_widgets[key]
        widget.configure(state=tk.NORMAL)
        widget.insert(tk.END, line)
        widget.see(tk.END)
        widget.configure(state=tk.DISABLED)

        self._handle_launcher_status_line(key, line)

        if self._is_error_line(key, line):
            self._append_error(f"[{key}] {line}")

        if key in CORE_PROCESS_KEYS and ("[WARN]" in line or "[ERROR]" in line or "Traceback" in line):
            self.warning_until = time.time() + WARNING_WINDOW_SEC
            self.warning_message = f"{self.processes[key].config.title} reported a warning or error."

    def _set_process_status(self, key: str, text: str, state: str) -> None:
        self.process_status_texts[key].set(text)
        self.process_states[key] = state
        bg, _, fg = STATUS_COLORS[state]
        self.process_status_labels[key].configure(bg=bg, fg=fg)
        if key in self.pipeline_pills:
            pill, label = self.pipeline_pills[key]
            pill.configure(text=f"● {label}: {text}", bg=bg, fg=fg)

    def _recompute_robot_status(self) -> None:
        core_owned = all(self.processes.get(key) and self.processes[key].is_owned_running() for key in CORE_PROCESS_KEYS)
        core_running = all(self.processes.get(key) and self.processes[key].is_running() for key in CORE_PROCESS_KEYS)
        any_running = any(process.is_running() for process in self.processes.values())
        any_owned = any(process.is_owned_running() for process in self.processes.values())
        any_warning_state = any(self.process_states.get(key) == "warning" for key in CORE_PROCESS_KEYS)
        warning_active = (
            self.emergency_latched or
            self.collision_active or
            any_warning_state or
            (time.time() < self.warning_until)
        )

        if warning_active:
            state = "warning"
            text = "Warning"
            if self.collision_active:
                detail = (
                    f"Collision detected. Current case: {self.robot_case_text.get()}. "
                    f"Picked nuts: {self.picked_count_text.get()}."
                )
            else:
                detail = self.warning_message or "Operator attention required."
        elif core_owned:
            state = "running"
            text = "Running"
            detail = (
                f"Simulation and bin-picking are active. Case: {self.robot_case_text.get()}. "
                f"Picked nuts: {self.picked_count_text.get()}. {self.active_preset_text.get()} "
                f"(velocity={self.velocity_var.get():.2f}, acceleration={self.acceleration_var.get():.2f}). "
                f"AAE port={self.aae_port}."
            )
        elif any_running and not any_owned:
            state = "external"
            text = "External"
            detail = (
                "Matching ROS/Gazebo processes are running that this launcher did not start "
                "(another terminal or the real-robot bringup). Live controls are not applied to them. "
                "Stop All will terminate them."
            )
        elif core_running:
            state = "running"
            text = "Running"
            detail = (
                "Simulation and bin-picking are active (partly started outside this launcher). "
                f"Case: {self.robot_case_text.get()}. Picked nuts: {self.picked_count_text.get()}."
            )
        elif any_running:
            state = "warning"
            text = "Partial"
            detail = (
                "Only part of the stack is running. "
                f"Last known case: {self.robot_case_text.get()}. Picked nuts: {self.picked_count_text.get()}."
            )
        else:
            state = "stopped"
            text = "Stopped"
            detail = (
                f"Robot stack is not running. Prepared motion profile: velocity={self.velocity_var.get():.2f}, "
                f"acceleration={self.acceleration_var.get():.2f}. Prepared AAE port={self.aae_port}."
            )

        self.robot_status_text.set(text)
        self.robot_detail_text.set(detail)
        if self.robot_status_label is not None:
            bg, _, fg = STATUS_COLORS[state]
            self.robot_status_label.configure(bg=bg, fg=fg)

    def _drain_log_queue(self) -> None:
        while True:
            try:
                event, key, payload = self.log_queue.get_nowait()
            except queue.Empty:
                break

            if event == "log":
                self._append_log(key, payload)
            elif event == "status":
                text, state = payload
                self._set_process_status(key, text, state)

        self._sync_process_statuses()
        self._recompute_robot_status()
        self.root.after(100, self._drain_log_queue)

    def _sync_process_statuses(self) -> None:
        for key, process in self.processes.items():
            state = self.process_states.get(key)
            if process.is_owned_running():
                if state != "running":
                    self._set_process_status(key, "Running", "running")
            elif process.is_running():
                if state != "external":
                    self._set_process_status(key, "External", "external")
            elif state in ("running", "external"):
                self._set_process_status(key, "Stopped", "stopped")

        if not self.processes["bin_picking"].is_running():
            self._set_bin_picking_offline()

    def _on_close(self) -> None:
        # Closing the window stops only what this launcher started. Use "Stop All" to also
        # clean up matching processes started elsewhere.
        self.closing = True
        self.startup_cancelled = True
        # Synchronous: stop() works in daemon threads that would die with the window,
        # leaving the (setsid) children orphaned.
        for key in ("yolo", "aae", "bin_picking", "sim"):
            process = self.processes[key].process
            if process is None or process.poll() is not None:
                continue
            try:
                terminate_process_targets({process.pid: os.getpgid(process.pid)}, immediate=False, timeout_sec=3.0)
            except ProcessLookupError:
                pass
        self.root.destroy()


def main() -> None:
    if not ROS_SETUP.exists():
        raise SystemExit(f"ROS overlay not found: {ROS_SETUP}")

    root = tk.Tk()
    app = ProjectLauncherApp(root)
    if app.conda_exe is None:
        messagebox.showwarning(
            "Conda not found",
            "Conda was not detected automatically. ROS launch buttons will still work, but the AAE and YOLO "
            "commands may need manual edits in the GUI.",
        )
    root.mainloop()


if __name__ == "__main__":
    main()
