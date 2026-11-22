#!/usr/bin/env python3
"""Phase 0 check: does the SDF screw joint behave like an M20 thread in Gazebo Fortress?

Runs jaka_description/worlds/tests/screw_thread_test.sdf headless, drives the thread
with a torque and checks:
  1. self-locking: no rotation under gravity or under a torque below the thread friction,
  2. pitch: 2.5 mm of travel per revolution,
  3. direction: clockwise (negative angle) tightens = nut moves down (right-hand thread),
  4. seat: the nut stops at the plate (joint lower limit) while torque is still applied.

Usage (ROS 2 Humble sourced):
    python3 tools/screw_thread_test.py
It uses its own Gazebo partition and ROS domain, so it never talks to a running sim.

Why a ROS bridge for the torque: a one-shot `ign topic -p` often loses its message
before the connection is set up. Re-publishing at 50 Hz through ros_gz_bridge is reliable.
"""
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "jaka_ros_ws/src/jaka_description/worlds/tests/screw_thread_test.sdf"

PITCH_M = 0.0025                  # M20 coarse thread: 2.5 mm per revolution
SEAT_ANGLE = -3 * 2 * math.pi     # joint lower limit: 3 turns below the start pose
START_Z = 0.0275                  # nut bottom at the start pose [m]
SEAT_Z = 0.020                    # fixture plate top [m]
THREAD_FRICTION = 0.05            # <friction> of the thread joint [N*m]

MODEL = "nut_on_bolt"
WORLD_NAME = "screw_thread_test"
TORQUE_TOPIC = f"/model/{MODEL}/joint/thread/cmd_force"
JOINT_STATE_TOPIC = f"/world/{WORLD_NAME}/model/{MODEL}/joint_state"

# Isolation: a private Gazebo partition and ROS domain for this test only.
ENV = dict(os.environ, IGN_PARTITION=f"screw_thread_test_{os.getpid()}",
           ROS_DOMAIN_ID="81", ROS_LOCALHOST_ONLY="1")
os.environ.update({k: ENV[k] for k in ("IGN_PARTITION", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")})


# --------------------------------------------------------------------------- helpers
def start(cmd: list[str]) -> subprocess.Popen:
    """Start a background process in its own process group (so we can stop all of it)."""
    return subprocess.Popen(cmd, env=ENV, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            preexec_fn=os.setsid)


def stop(proc: subprocess.Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def read_thread_state() -> tuple[float, float]:
    """Return (angle [rad], angular velocity [rad/s]) of the thread joint."""
    out = subprocess.run(["ign", "topic", "-e", "-t", JOINT_STATE_TOPIC, "-n", "1"],
                         env=ENV, capture_output=True, text=True, timeout=10).stdout
    section = out.split('name: "thread"', 1)[1]
    values = {}
    for line in section.splitlines():
        key, _, value = line.strip().partition(":")
        if key in ("position", "velocity") and key not in values:
            values[key] = float(value)
    # A field equal to zero is omitted from the protobuf text output.
    return values.get("position", 0.0), values.get("velocity", 0.0)


def read_nut_z() -> float:
    """Return the height of the nut's bottom face [m]."""
    out = subprocess.run(["ign", "model", "-m", MODEL, "-l", "nut"],
                         env=ENV, capture_output=True, text=True, timeout=10).stdout
    lines = out.splitlines()
    # "- Inertial Pose [...]" comes first; the link pose is the line "- Pose [...]".
    index = next(i for i, line in enumerate(lines) if line.strip().startswith("- Pose ["))
    xyz = lines[index + 1].strip(" []").split()
    return float(xyz[2])


class TorqueCommander:
    """Publishes the commanded thread torque at 50 Hz (through ros_gz_bridge)."""

    def __init__(self) -> None:
        import rclpy
        from std_msgs.msg import Float64
        rclpy.init()
        self._rclpy, self._msg_type = rclpy, Float64
        self.node = rclpy.create_node("screw_thread_test")
        self.pub = self.node.create_publisher(Float64, TORQUE_TOPIC, 10)
        self.torque = 0.0
        self.node.create_timer(0.02, self._publish)
        self._thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._thread.start()

    def _publish(self) -> None:
        self.pub.publish(self._msg_type(data=self.torque))

    def set(self, torque: float, hold_sec: float) -> None:
        self.torque = torque
        time.sleep(hold_sec)

    def shutdown(self) -> None:
        self.torque = 0.0
        time.sleep(0.2)
        self.node.destroy_node()
        self._rclpy.shutdown()


def settle(commander: TorqueCommander) -> tuple[float, float]:
    """Remove the torque, wait for the nut to stop, return (angle, z) at rest."""
    commander.set(0.0, 1.0)
    return read_thread_state()[0], read_nut_z()


# --------------------------------------------------------------------------- test
def main() -> int:
    if not WORLD.exists():
        print(f"World not found: {WORLD}")
        return 2

    sim = start(["ign", "gazebo", "-s", "-r", str(WORLD)])
    bridge = start(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                    f"{TORQUE_TOPIC}@std_msgs/msg/Float64]ignition.msgs.Double"])
    results: list[tuple[str, bool, str]] = []
    commander = None
    try:
        time.sleep(6.0)  # let Gazebo load the world and the bridge connect
        commander = TorqueCommander()
        commander.set(0.0, 2.0)

        # 1. Self-locking: gravity alone, then a torque below the thread friction.
        angle0, z0 = read_thread_state()[0], read_nut_z()
        results.append(("start pose", abs(z0 - START_Z) < 1e-4, f"z = {z0 * 1000:.2f} mm (expect {START_Z * 1000:.1f})"))
        commander.set(-0.6 * THREAD_FRICTION, 2.0)
        angle1, z1 = settle(commander)
        results.append(("self-locking (torque 0.6 x friction)", abs(angle1 - angle0) < 0.01,
                        f"moved {math.degrees(angle1 - angle0):+.2f} deg"))

        # 2 + 3. Tighten for a short time, measure travel per revolution and direction.
        commander.set(-0.15, 0.6)
        angle2, z2 = settle(commander)
        turns = (angle2 - angle1) / (2 * math.pi)
        mm_per_rev = (z2 - z1) * 1000 / turns if abs(turns) > 1e-3 else float("nan")
        results.append(("clockwise torque tightens (nut moves down)", turns < 0 and z2 < z1,
                        f"{turns:+.2f} turns, dz = {(z2 - z1) * 1000:+.2f} mm"))
        results.append(("pitch 2.5 mm/rev", abs(mm_per_rev - PITCH_M * 1000) < 0.02,
                        f"{mm_per_rev:.3f} mm/rev"))

        # 4. Seat: keep tightening; the nut must stop at the plate while torque is applied.
        commander.torque = -0.15
        deadline = time.time() + 20.0
        time.sleep(1.0)
        while time.time() < deadline and abs(read_thread_state()[1]) > 0.05:
            time.sleep(0.5)  # still running down the bolt
        time.sleep(1.0)      # torque stays applied: a seated nut must not move
        angle3, velocity3 = read_thread_state()
        z3 = read_nut_z()
        results.append(("stops at seat under torque",
                        abs(angle3 - SEAT_ANGLE) < 0.05 and abs(velocity3) < 0.05 and abs(z3 - SEAT_Z) < 2e-4,
                        f"angle = {angle3:.3f} rad (limit {SEAT_ANGLE:.3f}), z = {z3 * 1000:.2f} mm, "
                        f"vel = {velocity3:.3f} rad/s"))

        # Loosen again: counter-clockwise must move the nut up.
        commander.set(+0.15, 0.6)
        angle4, z4 = settle(commander)
        results.append(("counter-clockwise loosens (nut moves up)", angle4 > angle3 and z4 > z3,
                        f"{(angle4 - angle3) / (2 * math.pi):+.2f} turns, dz = {(z4 - z3) * 1000:+.2f} mm"))
    finally:
        if commander is not None:
            commander.shutdown()
        stop(bridge)
        stop(sim)

    print("\nScrew-thread check (M20, Gazebo Fortress / DART)")
    for name, ok, detail in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:45s} {detail}")
    passed = all(ok for _, ok, _ in results)
    print("\nResult:", "ALL PASSED" if passed else "FAILED")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
