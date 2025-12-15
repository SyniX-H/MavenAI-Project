import os
import zipfile
from pathlib import Path


def create_correct_package():
    """Create the correct ROS2 package"""
    
    base_dir = Path("test_packages/correct_package")
    base_dir.mkdir(parents=True, exist_ok=True)
    
    # Create resource directory
    resource_dir = base_dir / "resource"
    resource_dir.mkdir(exist_ok=True)
    (resource_dir / "correct_package").touch()
    
    # Create package directory
    pkg_dir = base_dir / "correct_package"
    pkg_dir.mkdir(exist_ok=True)
    
    # package.xml
    package_xml = '''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>correct_package</name>
  <version>1.0.0</version>
  <description>A correct pick and place package</description>
  <maintainer email="intern@example.com">Intern</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
'''
    
    # setup.py
    setup_py = '''from setuptools import setup

package_name = 'correct_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intern',
    maintainer_email='intern@example.com',
    description='A correct pick and place package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place = correct_package.pick_place_node:main',
        ],
    },
)
'''
    
    # setup.cfg
    setup_cfg = '''[develop]
script_dir=$base/lib/correct_package
[install]
install_scripts=$base/lib/correct_package
'''
    
    # pick_place_node.py
    pick_place_node = '''#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_joints = [0.0] * 6
        self.target_joints = [0.0] * 6
        self.state = 'idle'
        self.get_logger().info('Pick and Place Node Started')

    def joint_state_callback(self, msg):
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position[:6])

    def control_loop(self):
        if self.state == 'idle':
            self.target_joints = [0.5, -1.0, 1.5, -0.5, -1.57, 0.0]
            self.state = 'moving_to_grasp'

        elif self.state == 'moving_to_grasp':
            if self.joints_reached(self.target_joints):
                self.get_logger().info('Reached grasp position')
                self.state = 'grasping'

        elif self.state == 'grasping':
            self.get_logger().info('Grasping object')
            self.state = 'moving_to_place'
            self.target_joints = [0.0, -1.2, 1.8, -0.6, -1.57, 0.0]

        elif self.state == 'moving_to_place':
            if self.joints_reached(self.target_joints):
                self.get_logger().info('Reached place position')
                self.state = 'complete'

        self.publish_joint_command()

    def publish_joint_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint_{i+1}' for i in range(6)]
        msg.position = self.target_joints
        self.joint_pub.publish(msg)

    def joints_reached(self, target, tolerance=0.01):
        for i in range(6):
            if abs(self.current_joints[i] - target[i]) > tolerance:
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
    
    # Write files
    with open(base_dir / "package.xml", "w") as f:
        f.write(package_xml)
    
    with open(base_dir / "setup.py", "w") as f:
        f.write(setup_py)
    
    with open(base_dir / "setup.cfg", "w") as f:
        f.write(setup_cfg)
    
    with open(pkg_dir / "__init__.py", "w") as f:
        f.write("")
    
    with open(pkg_dir / "pick_place_node.py", "w") as f:
        f.write(pick_place_node)
    
    # Create ZIP
    zip_path = "test_packages/correct_package.zip"
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        for root, dirs, files in os.walk(base_dir):
            for file in files:
                file_path = os.path.join(root, file)
                arcname = os.path.relpath(file_path, base_dir.parent)
                zipf.write(file_path, arcname)
    
    print(f"✓ Created correct package at {zip_path}")


def create_faulty_package():
    """Create the faulty ROS2 package"""
    
    base_dir = Path("test_packages/faulty_package")
    base_dir.mkdir(parents=True, exist_ok=True)
    
    # Create package directory
    pkg_dir = base_dir / "faulty_package"
    pkg_dir.mkdir(exist_ok=True)
    
    # NO package.xml (intentional error)
    
    # setup.py
    setup_py = '''from setuptools import setup

package_name = 'faulty_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intern',
    maintainer_email='intern@example.com',
    description='A faulty package for testing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bad_node = faulty_package.bad_node:main',
        ],
    },
)
'''
    
    # bad_node.py (with intentional errors)
    bad_node = '''#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class BadNode(Node):
    def __init__(self):
        super().__init__('bad_node')

    self.pub = self.create_publisher(
            'bad_topic',
            '/joint_commands',
            10
        )

      self.get_logger().info('Bad Node Started')

    def bad_loop(self):
        while True:
            joint_value = 999.0

            msg = UndefinedMessage()
            msg.position = [joint_value] * 6

            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BadNode()

    node.bad_loop()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
    
    # Write files
    with open(base_dir / "setup.py", "w") as f:
        f.write(setup_py)
    
    with open(pkg_dir / "__init__.py", "w") as f:
        f.write("")
    
    with open(pkg_dir / "bad_node.py", "w") as f:
        f.write(bad_node)
    
    # Create ZIP
    zip_path = "test_packages/faulty_package.zip"
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        for root, dirs, files in os.walk(base_dir):
            for file in files:
                file_path = os.path.join(root, file)
                arcname = os.path.relpath(file_path, base_dir.parent)
                zipf.write(file_path, arcname)
    
    print(f"✓ Created faulty package at {zip_path}")


def main():
    print("=" * 60)
    print("Creating Test Packages")
    print("=" * 60)
    
    # Create test_packages directory
    Path("test_packages").mkdir(exist_ok=True)
    
    # Create both packages
    create_correct_package()
    create_faulty_package()
    
    print("\n" + "=" * 60)
    print("Test packages created successfully!")
    print("=" * 60)
    print("\nYou can now upload these packages via the web interface:")
    print("  - test_packages/correct_package.zip (should PASS)")
    print("  - test_packages/faulty_package.zip (should FAIL)")


if __name__ == "__main__":
    main()
