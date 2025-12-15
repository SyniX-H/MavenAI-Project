import os
import subprocess
import time
import json
from pathlib import Path
from datetime import datetime
import signal
from typing import Dict


class GazeboSimRunner:
    def __init__(self, output_dir: str = "./simulation_output"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.sim_process = None
        self.node_process = None
        self.robot_urdf = self.output_dir / "simple_arm.urdf"

    # --------------------------------------------------
    # WORLD
    # --------------------------------------------------
    def create_world_file(self) -> str:
        world_content = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="pick_and_place_world">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    smokeless_plane>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

  </world>
</sdf>
"""
        world_file = self.output_dir / "pick_place.world"
        world_file.write_text(world_content)
        return str(world_file)

    # --------------------------------------------------
    # ROBOT URDF
    # --------------------------------------------------
    def create_robot_urdf(self):
        urdf = """<?xml version="1.0"?>
<robot name="simple_arm">

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.6 0.6 0.6 1"/>
      </material>
    </visual>
  </link>

</robot>
"""
        self.robot_urdf.write_text(urdf)

    # --------------------------------------------------
    # GAZEBO
    # --------------------------------------------------
    def launch_gazebo(self, world_file: str) -> bool:
        self.sim_process = subprocess.Popen(
            ["ign", "gazebo", world_file, "-r"],
            preexec_fn=os.setsid
        )
        time.sleep(5)
        return self.sim_process.poll() is None

    # --------------------------------------------------
    # SPAWN ROBOT (ROS 2 + GAZEBO)
    # --------------------------------------------------
    def spawn_robot(self) -> bool:
        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-name", "robot_arm",
            "-file", str(self.robot_urdf),
            "-x", "0", "-y", "0", "-z", "0.5"
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode == 0

    # --------------------------------------------------
    # CLEANUP
    # --------------------------------------------------
    def cleanup(self):
        if self.sim_process:
            os.killpg(os.getpgid(self.sim_process.pid), signal.SIGTERM)

    # --------------------------------------------------
    # MAIN PIPELINE
    # --------------------------------------------------
    def run_simulation(self, package_path: str, duration: int = 30) -> Dict:
        report = {"status": "unknown"}

        world = self.create_world_file()
        self.create_robot_urdf()

        if not self.launch_gazebo(world):
            report["status"] = "gazebo_failed"
            return report

        if not self.spawn_robot():
            report["status"] = "spawn_failed"
            return report

        time.sleep(duration)
        report["status"] = "completed"

        self.cleanup()
        return report


if __name__ == "__main__":
    runner = GazeboSimRunner()
    print(runner.run_simulation())

