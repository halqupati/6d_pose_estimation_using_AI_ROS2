#!/usr/bin/env python3
"""Simulation helper for the nut-tightening task: gives a nut its thread.

Gazebo cannot create a screw joint while a simulation runs, and a free nut cannot "catch"
a thread on a smooth cylinder. So when a nut has to be threaded onto the bolt, this node
swaps models: the empty bolt fixture (and the free nut, if any) is removed, and an identical
fixture is spawned in its place that also contains a nut joined to the bolt by a screw
joint (docs/tightening/phase0_screw_thread.md). The new nut appears exactly where the old
one was, so for the robot nothing changes.

Services (std_srvs/Trigger, simulation only):
  /bolt_fixture/engage_nut   a free nut lying on the bolt tip catches the thread
  /bolt_fixture/preload_nut  a nut already threaded `preload_turns` above the seat
                             (the tighten-only task)

The robot never reads anything from this node except success/failure; it tightens using
its own estimate of where it put the nut and its wrist torque.
"""
import math
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# Fixture geometry in the fixture model frame (table surface under the bolt axis).
# Must match jaka_description/worlds/bolt_m20/model.sdf.
SEAT_Z = 0.020        # plate top: a seated nut's bottom face
SHANK_TOP_Z = 0.040   # bolt tip
NUT_HEIGHT = 0.016    # M20 nut


def threaded_fixture_sdf(nut_bottom_z: float, nut_yaw: float, pitch: float) -> str:
    """SDF of the fixture with a nut on a screw joint, nut bottom at `nut_bottom_z`."""
    two_pi = 2.0 * math.pi
    lower = -two_pi * (nut_bottom_z - SEAT_Z) / pitch                     # seat
    upper = max(0.1, two_pi * (SHANK_TOP_Z + 0.002 - nut_bottom_z) / pitch)  # can unscrew off the tip
    thread_pitch = -two_pi / pitch  # rad/m in Fortress/DART; negative = right-hand thread
    hexagon = "".join(
        f"""
        <collision name="flats_{name}">
          <pose>0 0 {NUT_HEIGHT / 2} 0 0 {angle}</pose>
          <geometry><box><size>0.01732 0.030 {NUT_HEIGHT}</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>"""
        for name, angle in (("90", 0.0), ("150", math.pi / 3), ("30", 2 * math.pi / 3))
    )
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="bolt_m20_threaded">
    <link name="bolt">
      <inertial><mass>2.0</mass><inertia><ixx>2e-3</ixx><iyy>2e-3</iyy><izz>3e-3</izz></inertia></inertial>
      <collision name="plate"><pose>0 0 0.010 0 0 0</pose>
        <geometry><box><size>0.10 0.10 0.020</size></box></geometry></collision>
      <visual name="plate_visual"><pose>0 0 0.010 0 0 0</pose>
        <geometry><box><size>0.10 0.10 0.020</size></box></geometry>
        <material><ambient>0.20 0.22 0.25 1</ambient><diffuse>0.35 0.38 0.42 1</diffuse></material></visual>
      <collision name="shank"><pose>0 0 0.030 0 0 0</pose>
        <geometry><cylinder><radius>0.010</radius><length>0.020</length></cylinder></geometry></collision>
      <visual name="shank_visual"><pose>0 0 0.030 0 0 0</pose>
        <geometry><cylinder><radius>0.010</radius><length>0.020</length></cylinder></geometry>
        <material><ambient>0.30 0.30 0.30 1</ambient><diffuse>0.60 0.60 0.62 1</diffuse></material></visual>
    </link>
    <joint name="fixture_fixed" type="fixed"><parent>world</parent><child>bolt</child></joint>

    <link name="nut">
      <pose>0 0 {nut_bottom_z:.5f} 0 0 {nut_yaw:.5f}</pose>
      <inertial><pose>0 0 {NUT_HEIGHT / 2} 0 0 0</pose><mass>0.05</mass>
        <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia></inertial>{hexagon}
      <visual name="visual">
        <geometry><mesh><uri>model://nut/meshes/nut.stl</uri><scale>0.001 0.001 0.001</scale></mesh></geometry>
        <material><ambient>0.16 0.17 0.18 1</ambient><diffuse>0.50 0.53 0.58 1</diffuse></material>
      </visual>
    </link>

    <joint name="thread" type="screw">
      <parent>bolt</parent>
      <child>nut</child>
      <thread_pitch>{thread_pitch:.4f}</thread_pitch>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>{lower:.5f}</lower><upper>{upper:.5f}</upper></limit>
        <dynamics><damping>0.01</damping><friction>0.05</friction></dynamics>
      </axis>
    </joint>

    <plugin filename="ignition-gazebo-joint-state-publisher-system" name="ignition::gazebo::systems::JointStatePublisher"/>
  </model>
</sdf>
"""


class BoltFixtureManager(Node):
    def __init__(self) -> None:
        super().__init__("bolt_fixture_manager")
        self.world = self.declare_parameter("world_name", "empty_world").value
        self.fixture = self.declare_parameter("fixture_name", "bolt_fixture").value
        self.pitch = self.declare_parameter("thread_pitch", 0.0025).value
        self.preload_turns = self.declare_parameter("preload_turns", 3.0).value
        # How well the free nut must sit on the bolt tip to "catch the thread".
        self.xy_tolerance = self.declare_parameter("engage_xy_tolerance", 0.006).value
        self.tmp_dir = Path(tempfile.mkdtemp(prefix="bolt_fixture_"))

        self.create_service(Trigger, "/bolt_fixture/engage_nut", self.on_engage)
        self.create_service(Trigger, "/bolt_fixture/preload_nut", self.on_preload)
        self.get_logger().info("Bolt fixture manager ready (/bolt_fixture/engage_nut, /bolt_fixture/preload_nut)")

    # ------------------------------------------------------------------ Gazebo access
    def _ign(self, *args: str, timeout: float = 10.0) -> str:
        return subprocess.run(["ign", *args], capture_output=True, text=True, timeout=timeout).stdout

    def model_names(self) -> list[str]:
        out = self._ign("model", "--list")
        return [line.strip()[2:] for line in out.splitlines() if line.strip().startswith("- ")]

    def model_pose(self, name: str) -> tuple[float, float, float, float] | None:
        """(x, y, z, yaw) of a model in the world, or None if it does not exist."""
        lines = self._ign("model", "-m", name, "-p").splitlines()
        for index, line in enumerate(lines):
            if line.strip().startswith("- Pose ["):
                x, y, z = (float(v) for v in lines[index + 1].strip(" []").split())
                yaw = float(lines[index + 2].strip(" []").split()[2])
                return x, y, z, yaw
        return None

    def has_threaded_nut(self) -> bool:
        return "Name: nut" in self._ign("model", "-m", self.fixture, "-l", "nut")

    def remove_model(self, name: str) -> bool:
        reply = self._ign("service", "-s", f"/world/{self.world}/remove",
                          "--reqtype", "ignition.msgs.Entity", "--reptype", "ignition.msgs.Boolean",
                          "--timeout", "5000", "--req", f'name: "{name}" type: MODEL')
        return "data: true" in reply

    def spawn_threaded_fixture(self, pose, nut_bottom_z: float, nut_yaw: float) -> bool:
        x, y, z, yaw = pose
        sdf_file = self.tmp_dir / "bolt_m20_threaded.sdf"
        sdf_file.write_text(threaded_fixture_sdf(nut_bottom_z, nut_yaw, self.pitch))
        request = (f'sdf_filename: "{sdf_file}" name: "{self.fixture}" allow_renaming: false '
                   f"pose {{ position {{ x: {x} y: {y} z: {z} }} "
                   f"orientation {{ z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)} }} }}")
        reply = self._ign("service", "-s", f"/world/{self.world}/create",
                          "--reqtype", "ignition.msgs.EntityFactory", "--reptype", "ignition.msgs.Boolean",
                          "--timeout", "5000", "--req", request)
        return "data: true" in reply

    def swap_fixture(self, fixture_pose, nut_bottom_z: float, nut_yaw: float, free_nut: str | None) -> str | None:
        """Remove the free nut (optional) and the empty fixture, spawn the threaded one.
        Returns an error message, or None on success."""
        if free_nut is not None and not self.remove_model(free_nut):
            return f"could not remove free nut '{free_nut}'"
        if not self.remove_model(self.fixture):
            return f"could not remove fixture '{self.fixture}'"
        if not self.spawn_threaded_fixture(fixture_pose, nut_bottom_z, nut_yaw):
            return "could not spawn the threaded fixture"
        for _ in range(20):  # the entity appears a few simulation steps later
            if self.has_threaded_nut():
                return None
            time.sleep(0.2)
        return "threaded fixture did not appear"

    # ------------------------------------------------------------------ services
    def _fixture_or_error(self, response: Trigger.Response):
        pose = self.model_pose(self.fixture)
        if pose is None:
            response.success, response.message = False, f"fixture '{self.fixture}' not found"
            return None
        if self.has_threaded_nut():
            response.success, response.message = False, "the bolt already has a threaded nut"
            return None
        return pose

    def on_engage(self, _request, response: Trigger.Response) -> Trigger.Response:
        fixture_pose = self._fixture_or_error(response)
        if fixture_pose is None:
            return response
        fx, fy, fz, fyaw = fixture_pose

        # Find a free nut whose bottom face rests on the bolt tip, centred on the axis.
        candidate = None
        for name in self.model_names():
            if not name.startswith("nut"):
                continue
            pose = self.model_pose(name)
            if pose is None:
                continue
            x, y, z, yaw = pose
            off_axis = math.hypot(x - fx, y - fy)
            bottom = z - fz  # nut model frame = its bottom face
            if off_axis <= self.xy_tolerance and SHANK_TOP_Z - 0.003 <= bottom <= SHANK_TOP_Z + 0.006:
                candidate = (name, yaw, off_axis, bottom)
                break
        if candidate is None:
            response.success = False
            response.message = (f"no free nut on the bolt tip (within {self.xy_tolerance * 1000:.0f} mm of the "
                                f"axis, bottom face at the tip height)")
            return response

        name, yaw, off_axis, bottom = candidate
        # The thread "catches" with the nut bottom 1 mm below the tip.
        engaged_bottom = SHANK_TOP_Z - 0.001
        error = self.swap_fixture(fixture_pose, engaged_bottom, yaw - fyaw, free_nut=name)
        if error:
            response.success, response.message = False, error
            return response
        turns = (engaged_bottom - SEAT_Z) / self.pitch
        response.success = True
        response.message = (f"{name} engaged: off-axis {off_axis * 1000:.1f} mm, nut bottom "
                            f"{engaged_bottom * 1000:.1f} mm above the table, {turns:.2f} turns above the seat")
        self.get_logger().info(response.message)
        return response

    def on_preload(self, _request, response: Trigger.Response) -> Trigger.Response:
        fixture_pose = self._fixture_or_error(response)
        if fixture_pose is None:
            return response
        bottom = SEAT_Z + self.preload_turns * self.pitch
        error = self.swap_fixture(fixture_pose, bottom, 0.0, free_nut=None)
        if error:
            response.success, response.message = False, error
            return response
        response.success = True
        response.message = f"nut preloaded {self.preload_turns:.2f} turns above the seat"
        self.get_logger().info(response.message)
        return response


def main() -> None:
    rclpy.init()
    node = BoltFixtureManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
