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
        
    def setup_environment(self):
        """Source ROS2 and workspace"""
        commands = [
            "source /opt/ros/humble/setup.bash",
        ]
        return commands
    
    def create_world_file(self) -> str:
        """Create a simple Gazebo world with UR5 arm, cube, and target"""
        
        # First, check if UR5 SDF exists, if not create a simple one
        ur5_sdf_path = Path("ur5.sdf")
        
        world_content = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="pick_and_place_world">
    
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
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
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- UR5 from Gazebo Fuel -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/UR5</uri>
      <name>ur5_robot</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
    
    <model name="cube">
      <pose>0.5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.1</mass>
        </inertial>
      </link>
    </model>
    
    <model name="target">
      <pose>0.5 0.3 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>0.01</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
"""
        
        world_file = self.output_dir / "pick_place.world"
        with open(world_file, 'w') as f:
            f.write(world_content)
        
        return str(world_file)
    
    def launch_gazebo(self, world_file: str, headless: bool = False) -> bool:
        """Launch Gazebo with the world"""
        try:
            cmd = [
                "ign", "gazebo", world_file,
                "-r"
            ]
            
            if headless:
                cmd.append("-s")
            
            self.sim_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            time.sleep(5)
            
            if self.sim_process.poll() is None:
                print("✓ Gazebo launched successfully")
                return True
            else:
                print("✗ Gazebo failed to launch")
                return False
                
        except Exception as e:
            print(f"Error launching Gazebo: {e}")
            return False
    
    def spawn_robot(self, robot_model: str = "ur5") -> bool:
        """Spawn robotic arm in Gazebo"""
        try:
            spawn_cmd = [
                "ros2", "run", "gazebo_ros", "spawn_entity.py",
                "-entity", "robot_arm",
                "-database", robot_model,
                "-x", "0", "-y", "0", "-z", "0.5"
            ]
            
            result = subprocess.run(
                spawn_cmd,
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                print("✓ Robot spawned successfully")
                return True
            else:
                print(f"✗ Failed to spawn robot: {result.stderr}")
                return False
                
        except Exception as e:
            print(f"Error spawning robot: {e}")
            return False
    
    def run_ros_node(self, package_path: str, node_name: str = None) -> bool:
        """Execute the submitted ROS2 node"""
        try:
            build_cmd = f"cd {package_path} && colcon build"
            result = subprocess.run(
                build_cmd,
                shell=True,
                capture_output=True,
                text=True
            )
            
            if result.returncode != 0:
                print(f"Build failed: {result.stderr}")
                return False
            
            source_cmd = f"source {package_path}/install/setup.bash"
            
            if node_name:
                run_cmd = f"{source_cmd} && ros2 run {node_name}"
            else:
                print("No node name provided")
                return False
            
            self.node_process = subprocess.Popen(
                run_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            print("✓ ROS node started")
            return True
            
        except Exception as e:
            print(f"Error running node: {e}")
            return False
    
    def capture_simulation_state(self, duration: int = 10) -> Dict:
        """Monitor simulation and capture key metrics"""
        start_time = time.time()
        states = []
        
        while time.time() - start_time < duration:
            try:
                result = subprocess.run(
                    ["ros2", "topic", "echo", "/joint_states", "--once"],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                
                if result.returncode == 0:
                    states.append({
                        "timestamp": time.time() - start_time,
                        "joint_state": result.stdout[:200]
                    })
            except:
                pass
            
            time.sleep(1)
        
        return {
            "duration": duration,
            "states_captured": len(states),
            "states": states
        }
    
    def take_screenshot(self, filename: str = None):
        """Capture Gazebo screenshot"""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"screenshot_{timestamp}.png"
        
        screenshot_path = self.output_dir / filename
        
        try:
            subprocess.run(
                ["ign", "service", "-s", "/gui/screenshot", 
                 "--reqtype", "ignition.msgs.StringMsg",
                 f"--req", f"data: '{screenshot_path}'"],
                timeout=5
            )
            print(f"Screenshot saved: {screenshot_path}")
        except:
            print("Screenshot capture failed")
    
    def cleanup(self):
        """Stop all processes"""
        if self.node_process:
            try:
                os.killpg(os.getpgid(self.node_process.pid), signal.SIGTERM)
                print("✓ ROS node terminated")
            except:
                pass
        
        if self.sim_process:
            try:
                os.killpg(os.getpgid(self.sim_process.pid), signal.SIGTERM)
                print("✓ Gazebo terminated")
            except:
                pass
    
    def run_simulation(self, package_path: str, duration: int = 30) -> Dict:
        """Main simulation pipeline"""
        report = {
            "status": "unknown",
            "start_time": datetime.now().isoformat(),
            "duration": duration,
            "logs": [],
            "screenshots": []
        }
        
        try:
            # Step 1: Create world (with robot already included!)
            world_file = self.create_world_file()
            report["logs"].append("World file created with robot arm")
            
            # Step 2: Launch Gazebo (GUI mode on VM)
            if not self.launch_gazebo(world_file, headless=False):
                report["status"] = "failed"
                report["logs"].append("Failed to launch Gazebo")
                return report
            
            report["logs"].append("Gazebo launched successfully")
            report["logs"].append("Robot arm loaded from world file")
            
            # Step 3: Take initial screenshot
            time.sleep(3)
            self.take_screenshot("start.png")
            report["screenshots"].append("start.png")
            report["logs"].append("Initial screenshot captured")
            
            # Step 4: Capture simulation state
            report["logs"].append(f"Running simulation for {duration} seconds...")
            sim_data = self.capture_simulation_state(duration)
            report["simulation_data"] = sim_data
            report["logs"].append(f"Captured {sim_data['states_captured']} state snapshots")
            
            # Step 5: Take final screenshot
            self.take_screenshot("end.png")
            report["screenshots"].append("end.png")
            report["logs"].append("Final screenshot captured")
            
            report["logs"].append("Simulation completed successfully")
            report["status"] = "completed"
            
        except Exception as e:
            report["status"] = "error"
            report["logs"].append(f"Error: {str(e)}")
            import traceback
            report["logs"].append(traceback.format_exc())
        
        finally:
            self.cleanup()
        
        # Save report
        report_file = self.output_dir / "simulation_report.json"
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        return report


if __name__ == "__main__":
    runner = GazeboSimRunner()
    print("Simulation Runner module loaded successfully")
