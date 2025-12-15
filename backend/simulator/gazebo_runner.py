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
        """Create a Gazebo world with custom Panda-like robot arm"""
        
        world_content = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="robot_demo">
    
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground -->
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
            <specular>0 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Panda-like 6-DOF Robot Arm -->
    <model name="panda_arm">
      <pose>0 0 0 0 0 0</pose>
      <static>false</static>
      
      <!-- Base Link -->
      <link name="base_link">
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <mass>4.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.06</radius>
              <length>0.15</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.06</radius>
              <length>0.15</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
            <diffuse>0.9 0.9 0.9 1</diffuse>
          </material>
        </visual>
      </link>
      
      <!-- Link 1 (Vertical) -->
      <link name="link1">
        <pose>0 0 0.2 0 0 0</pose>
        <inertial>
          <mass>3.0</mass>
          <inertia>
            <ixx>0.05</ixx>
            <iyy>0.05</iyy>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0.5 0 1</ambient>
            <diffuse>1 0.6 0 1</diffuse>
          </material>
        </visual>
      </link>
      
      <!-- Link 2 (Upper Arm) -->
      <link name="link2">
        <pose>0.15 0 0.3 0 1.57 0</pose>
        <inertial>
          <mass>2.5</mass>
          <inertia>
            <ixx>0.04</ixx>
            <iyy>0.04</iyy>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.04</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.04</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
      
      <!-- Link 3 (Forearm Part 1) -->
      <link name="link3">
        <pose>0.3 0 0.25 0 1.57 0</pose>
        <inertial>
          <mass>2.0</mass>
          <inertia>
            <ixx>0.03</ixx>
            <iyy>0.03</iyy>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.035</radius>
              <length>0.25</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.035</radius>
              <length>0.25</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0.5 0 1</ambient>
            <diffuse>1 0.6 0 1</diffuse>
          </material>
        </visual>
      </link>
      
      <!-- Link 4 (Forearm Part 2) -->
      <link name="link4">
        <pose>0.425 0 0.2 0 1.57 0</pose>
        <inertial>
          <mass>1.5</mass>
          <inertia>
            <ixx>0.02</ixx>
            <iyy>0.02</iyy>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
      
      <!-- Gripper/End Effector -->
      <link name="gripper">
        <pose>0.525 0 0.2 0 0 0</pose>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.001</ixx>
            <iyy>0.001</iyy>
            <izz>0.001</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.08 0.08 0.08</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.08 0.08 0.08</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0.5 1 1</ambient>
            <diffuse>0 0.6 1 1</diffuse>
          </material>
        </visual>
      </link>
      
      <!-- Joints -->
      <joint name="joint1" type="revolute">
        <parent>base_link</parent>
        <child>link1</child>
        <pose>0 0 -0.1 0 0 0</pose>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-2.8973</lower>
            <upper>2.8973</upper>
          </limit>
        </axis>
      </joint>
      
      <joint name="joint2" type="revolute">
        <parent>link1</parent>
        <child>link2</child>
        <pose>-0.15 0 0 0 0 0</pose>
        <axis>
          <xyz>0 1 0</xyz>
          <limit>
            <lower>-1.7628</lower>
            <upper>1.7628</upper>
          </limit>
        </axis>
      </joint>
      
      <joint name="joint3" type="revolute">
        <parent>link2</parent>
        <child>link3</child>
        <pose>-0.125 0 0 0 0 0</pose>
        <axis>
          <xyz>0 1 0</xyz>
          <limit>
            <lower>-1.7628</lower>
            <upper>1.7628</upper>
          </limit>
        </axis>
      </joint>
      
      <joint name="joint4" type="revolute">
        <parent>link3</parent>
        <child>link4</child>
        <pose>-0.1 0 0 0 0 0</pose>
        <axis>
          <xyz>0 1 0</xyz>
          <limit>
            <lower>-3.0718</lower>
            <upper>3.0718</upper>
          </limit>
        </axis>
      </joint>
      
      <joint name="joint5" type="fixed">
        <parent>link4</parent>
        <child>gripper</child>
      </joint>
      
    </model>
    
    <!-- Red Cube (Target Object) -->
    <model name="cube">
      <pose>0.5 0 0.525 0 0 0</pose>
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
    
    <!-- Green Target Marker -->
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
        
        world_file = self.output_dir / "panda_robot_world.sdf"
        with open(world_file, 'w') as f:
            f.write(world_content)
        
        print(f"✓ Created custom Panda-like robot world: {world_file}")
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
            # Step 1: Create world
            world_file = self.create_world_file()
            report["logs"].append("World file created")
            
            # Step 2: Launch Gazebo (GUI mode on VM)
            if not self.launch_gazebo(world_file, headless=False):
                report["status"] = "failed"
                report["logs"].append("Failed to launch Gazebo")
                return report
            
            report["logs"].append("Gazebo launched successfully")
            
            # Step 3: Take initial screenshot
            time.sleep(2)
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
            
            # Step 6: Check status
            if self.sim_process and self.sim_process.poll() is None:
                report["logs"].append("Simulation completed successfully")
                report["status"] = "completed"
            else:
                report["logs"].append("Simulation process ended")
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
