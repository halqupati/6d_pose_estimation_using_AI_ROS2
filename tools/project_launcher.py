#!/usr/bin/env python3

import os
import queue
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


ROOT_DIR = Path(__file__).resolve().parents[1]
ROS_SETUP = ROOT_DIR / "jaka_ros_ws" / "install" / "setup.bash"
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
        return (self.process is not None and self.process.poll() is None) or bool(self._external_targets())

    def _external_targets(self) -> dict[int, int]:
        return find_process_targets(self.config.cleanup_patterns)

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

    def stop(self, immediate: bool = False) -> None:
        if not self.is_running():
            return

        self.stop_requested = True
        self.stop_label = "Emergency Stop" if immediate else "Stopped"
        self.stop_state = "warning" if immediate else "stopped"
        targets = self._external_targets()

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
        self.root.geometry("1520x960")
        self.root.configure(bg=BG)

        self.log_queue: queue.Queue = queue.Queue()
        self.conda_exe = detect_conda_executable()
        self.aae_port = choose_available_port([5001, 5003, 5005])

        self.velocity_var = tk.DoubleVar(value=0.55)
        self.acceleration_var = tk.DoubleVar(value=0.40)
        self.velocity_text = tk.StringVar()
        self.acceleration_text = tk.StringVar()
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
        self.robot_case_text = tk.StringVar(value="Idle")
        self.robot_case_detail_text = tk.StringVar(value="Bin-picking process not running.")
        self.error_count = 0
        self.error_count_text = tk.StringVar(value="0")
        self.error_log_widget: ScrolledText | None = None
        self.warning_until = 0.0
        self.warning_message = ""
        self.emergency_latched = False

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
        root_frame = tk.Frame(self.root, bg=BG, padx=16, pady=16)
        root_frame.pack(fill=tk.BOTH, expand=True)

        header = tk.Frame(root_frame, bg=PANEL_BG, padx=18, pady=16, highlightthickness=1, highlightbackground="#1F2937")
        header.pack(fill=tk.X, pady=(0, 12))

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
            text="Unified launcher, motion profile control, robot status, and emergency stop.",
            bg=PANEL_BG,
            fg=MUTED,
            font=("TkDefaultFont", 10),
        ).pack(anchor="w", pady=(4, 0))

        button_col = tk.Frame(header, bg=PANEL_BG)
        button_col.pack(side=tk.RIGHT)
        self._make_button(button_col, "Start All", START, START_ACTIVE, self.start_all, width=12).pack(side=tk.LEFT, padx=(0, 8))
        self._make_button(button_col, "Stop All", STOP, STOP_ACTIVE, self.stop_all, width=12).pack(side=tk.LEFT, padx=(0, 8))
        self._make_button(button_col, "EMERGENCY STOP", DANGER, DANGER_ACTIVE, self.emergency_stop, width=18).pack(side=tk.LEFT)

        info = tk.Frame(root_frame, bg=BG)
        info.pack(fill=tk.X, pady=(0, 12))
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
        dashboard.pack(fill=tk.X, pady=(0, 14))
        dashboard.grid_columnconfigure(0, weight=1)
        dashboard.grid_columnconfigure(1, weight=1)
        dashboard.grid_rowconfigure(1, weight=1)

        status_card = tk.Frame(dashboard, bg=PANEL_BG, padx=18, pady=16, highlightthickness=1, highlightbackground="#1F2937")
        status_card.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        tk.Label(status_card, text="Robot Status", bg=PANEL_BG, fg=MUTED, font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        self.robot_status_label = tk.Label(
            status_card,
            textvariable=self.robot_status_text,
            bg="#5B4A11",
            fg="#FFF6D9",
            font=("TkDefaultFont", 18, "bold"),
            padx=18,
            pady=10,
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

        motion_card = tk.Frame(dashboard, bg=PANEL_BG, padx=18, pady=16, highlightthickness=1, highlightbackground="#1F2937")
        motion_card.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        tk.Label(motion_card, text="Motion Profile", bg=PANEL_BG, fg=MUTED, font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        tk.Label(
            motion_card,
            text="Use this to make the robot safer or faster. Apply live while bin-picking is running.",
            bg=PANEL_BG,
            fg=TEXT,
            wraplength=520,
            justify=tk.LEFT,
            font=("TkDefaultFont", 10),
        ).pack(anchor="w", pady=(6, 12))

        self._build_slider_row(motion_card, "Velocity", self.velocity_var, self.velocity_text)
        self._build_slider_row(motion_card, "Acceleration", self.acceleration_var, self.acceleration_text)

        preset_row = tk.Frame(motion_card, bg=PANEL_BG)
        preset_row.pack(fill=tk.X, pady=(10, 0))
        self._make_button(preset_row, "Safe", "#334155", "#1E293B", lambda: self.set_motion_profile(0.30, 0.25), width=10).pack(side=tk.LEFT)
        self._make_button(preset_row, "Balanced", "#2563EB", "#1D4ED8", lambda: self.set_motion_profile(0.55, 0.40), width=10).pack(side=tk.LEFT, padx=(8, 0))
        self._make_button(preset_row, "Fast", "#7C3AED", "#5B21B6", lambda: self.set_motion_profile(0.80, 0.60), width=10).pack(side=tk.LEFT, padx=(8, 0))
        self._make_button(preset_row, "Apply Live", START, START_ACTIVE, self.apply_motion_profile, width=12).pack(side=tk.RIGHT)

        telemetry_card = tk.Frame(dashboard, bg=PANEL_BG, padx=18, pady=16, highlightthickness=1, highlightbackground="#1F2937")
        telemetry_card.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(12, 0))
        telemetry_card.grid_columnconfigure(0, weight=1)
        telemetry_card.grid_columnconfigure(1, weight=1)
        telemetry_card.grid_columnconfigure(2, weight=2)
        telemetry_card.grid_columnconfigure(3, weight=1)

        tk.Label(
            telemetry_card,
            text="Live Telemetry",
            bg=PANEL_BG,
            fg=MUTED,
            font=("TkDefaultFont", 10, "bold"),
        ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 12))

        collision_box = tk.Frame(telemetry_card, bg=PANEL_ALT, padx=14, pady=12, highlightthickness=1, highlightbackground="#1F2937")
        collision_box.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        tk.Label(collision_box, text="Collision", bg=PANEL_ALT, fg=MUTED, font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        collision_row = tk.Frame(collision_box, bg=PANEL_ALT)
        collision_row.pack(anchor="w", pady=(8, 4))
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
            font=("TkDefaultFont", 14, "bold"),
        ).pack(side=tk.LEFT)
        tk.Label(
            collision_box,
            textvariable=self.collision_detail_text,
            bg=PANEL_ALT,
            fg=TEXT,
            wraplength=250,
            justify=tk.LEFT,
            font=("TkDefaultFont", 9),
        ).pack(anchor="w")

        picked_box = tk.Frame(telemetry_card, bg=PANEL_ALT, padx=14, pady=12, highlightthickness=1, highlightbackground="#1F2937")
        picked_box.grid(row=1, column=1, sticky="nsew", padx=8)
        tk.Label(picked_box, text="Picked Nuts", bg=PANEL_ALT, fg=MUTED, font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        tk.Label(
            picked_box,
            textvariable=self.picked_count_text,
            bg=PANEL_ALT,
            fg="#F8FAFC",
            font=("TkDefaultFont", 26, "bold"),
        ).pack(anchor="w", pady=(10, 4))
        tk.Label(
            picked_box,
            text="Completed pick-and-place cycles in this launcher run.",
            bg=PANEL_ALT,
            fg=TEXT,
            wraplength=220,
            justify=tk.LEFT,
            font=("TkDefaultFont", 9),
        ).pack(anchor="w")

        case_box = tk.Frame(telemetry_card, bg=PANEL_ALT, padx=14, pady=12, highlightthickness=1, highlightbackground="#1F2937")
        case_box.grid(row=1, column=2, sticky="nsew", padx=8)
        tk.Label(case_box, text="Current Case", bg=PANEL_ALT, fg=MUTED, font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        tk.Label(
            case_box,
            textvariable=self.robot_case_text,
            bg=PANEL_ALT,
            fg="#F8FAFC",
            wraplength=360,
            justify=tk.LEFT,
            font=("TkDefaultFont", 15, "bold"),
        ).pack(anchor="w", pady=(10, 4))
        tk.Label(
            case_box,
            textvariable=self.robot_case_detail_text,
            bg=PANEL_ALT,
            fg=TEXT,
            wraplength=360,
            justify=tk.LEFT,
            font=("TkDefaultFont", 9),
        ).pack(anchor="w")

        errors_box = tk.Frame(telemetry_card, bg=PANEL_ALT, padx=14, pady=12, highlightthickness=1, highlightbackground="#1F2937")
        errors_box.grid(row=1, column=3, sticky="nsew", padx=(8, 0))
        tk.Label(errors_box, text="Errors", bg=PANEL_ALT, fg=MUTED, font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        tk.Label(
            errors_box,
            textvariable=self.error_count_text,
            bg=PANEL_ALT,
            fg="#FCA5A5",
            font=("TkDefaultFont", 26, "bold"),
        ).pack(anchor="w", pady=(10, 4))
        tk.Label(
            errors_box,
            text="Warnings, failures, collision alerts, and tracebacks.",
            bg=PANEL_ALT,
            fg=TEXT,
            wraplength=220,
            justify=tk.LEFT,
            font=("TkDefaultFont", 9),
        ).pack(anchor="w")

        process_card = tk.Frame(controls_content, bg=PANEL_BG, padx=18, pady=16, highlightthickness=1, highlightbackground="#1F2937")
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
        self.root.after(0, lambda: main_pane.sashpos(0, max(420, int(self.root.winfo_height() * 0.64))))

    def _build_slider_row(self, parent: tk.Widget, label: str, variable: tk.DoubleVar, text_var: tk.StringVar) -> None:
        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill=tk.X, pady=4)
        tk.Label(row, text=label, bg=PANEL_BG, fg=TEXT, width=12, anchor="w").pack(side=tk.LEFT)
        scale = tk.Scale(
            row,
            from_=0.10,
            to=1.00,
            resolution=0.05,
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
        self.robot_case_text.set("Starting")
        self.robot_case_detail_text.set("Bin-picking process launch requested.")
        self._update_collision_indicator()

    def _set_bin_picking_offline(self) -> None:
        self.collision_seen = False
        self.collision_active = False
        self.collision_status_text.set("Offline")
        self.collision_detail_text.set("Bin-picking stack is not running.")
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

        if "picked_count" in fields:
            self.picked_count_text.set(fields["picked_count"])

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
                "arm_velocity_scaling": f"{self.velocity_var.get():.2f}",
                "arm_acceleration_scaling": f"{self.acceleration_var.get():.2f}",
            },
        )

    def _build_aae_command(self) -> str:
        return build_conda_python_command(
            self.conda_exe,
            "aae_py37_tf26",
            ROOT_DIR / "aae_infer" / "autoencoder_ws",
            "aae_ros_infer_sim_v4.py",
            {"AAE_SERVER_PORT": str(self.aae_port)},
        )

    def _build_yolo_command(self, script_name: str | None = None) -> str:
        yolo_script = script_name or detect_yolo_script()
        return build_conda_python_command(
            self.conda_exe,
            "yolov12_py10",
            ROOT_DIR / "yolov12_seg_infer",
            yolo_script,
            {"AAE_SERVER_PORT": str(self.aae_port)},
        )

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
        if hasattr(self, "command_vars") and "bin_picking" in self.command_vars:
            self._refresh_dynamic_commands()
        self._recompute_robot_status()

    def set_motion_profile(self, velocity: float, acceleration: float) -> None:
        self.velocity_var.set(velocity)
        self.acceleration_var.set(acceleration)
        self._on_motion_profile_changed()

    def apply_motion_profile(self) -> None:
        self._refresh_dynamic_commands()
        velocity = self.velocity_var.get()
        acceleration = self.acceleration_var.get()
        self._append_log("bin_picking", f"[launcher] motion profile selected: velocity={velocity:.2f}, acceleration={acceleration:.2f}\n")

        if not self.processes.get("bin_picking") or not self.processes["bin_picking"].is_running():
            self.robot_detail_text.set(
                f"Motion profile prepared for next start: velocity={velocity:.2f}, acceleration={acceleration:.2f}."
            )
            self._recompute_robot_status()
            return

        threading.Thread(target=self._apply_motion_profile_live, daemon=True).start()

    def _apply_motion_profile_live(self) -> None:
        velocity = self.velocity_var.get()
        acceleration = self.acceleration_var.get()
        node_name = "/bin_picking_node_v5_to_control_jaka_s12"
        command = build_ros_shell_command(
            f"ros2 param set {shlex.quote(node_name)} arm_velocity_scaling {velocity:.2f} && "
            f"ros2 param set {shlex.quote(node_name)} arm_acceleration_scaling {acceleration:.2f}"
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
                f"Applied live motion profile: velocity={velocity:.2f}, acceleration={acceleration:.2f}."
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
        self.emergency_latched = False
        startup_order = ("sim", "bin_picking", "aae", "yolo")
        startup_delay_ms = 3500
        for index, key in enumerate(startup_order):
            self.root.after(index * startup_delay_ms, lambda process_key=key: self.start_process(process_key))

    def stop_all(self) -> None:
        for key in ("yolo", "aae", "bin_picking", "sim"):
            self.processes[key].stop()
        self._recompute_robot_status()

    def emergency_stop(self) -> None:
        self.emergency_latched = True
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

    def _recompute_robot_status(self) -> None:
        core_running = all(self.processes.get(key) and self.processes[key].is_running() for key in CORE_PROCESS_KEYS)
        any_running = any(process.is_running() for process in self.processes.values())
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
        elif core_running:
            state = "running"
            text = "Running"
            detail = (
                f"Simulation and bin-picking are active. Case: {self.robot_case_text.get()}. "
                f"Picked nuts: {self.picked_count_text.get()}. Motion profile: velocity={self.velocity_var.get():.2f}, "
                f"acceleration={self.acceleration_var.get():.2f}. AAE port={self.aae_port}."
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
            if process.is_running():
                if self.process_states.get(key) != "running":
                    self._set_process_status(key, "Running", "running")
            elif self.process_states.get(key) == "running":
                self._set_process_status(key, "Stopped", "stopped")

        if not self.processes["bin_picking"].is_running():
            self._set_bin_picking_offline()

    def _on_close(self) -> None:
        running = [key for key, process in self.processes.items() if process.is_running()]
        if running:
            self.stop_all()
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
