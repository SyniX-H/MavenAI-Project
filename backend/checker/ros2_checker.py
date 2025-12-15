import os
import json
import subprocess
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple
import re


class ROS2CodeChecker:
    def __init__(self, upload_dir: str = "/tmp/ros2_uploads"):
        self.upload_dir = Path(upload_dir)
        self.upload_dir.mkdir(parents=True, exist_ok=True)
        self.errors = []
        self.warnings = []
        self.info = []

    def extract_package(self, zip_path: str) -> str:
        """Extract uploaded ZIP file"""
        extract_path = self.upload_dir / Path(zip_path).stem
        extract_path.mkdir(parents=True, exist_ok=True)
        
        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
            zip_ref.extractall(extract_path)
        
        self.info.append(f"Extracted package to {extract_path}")
        return str(extract_path)

    def check_package_structure(self, package_path: str) -> bool:
        """Check for required ROS2 package files"""
        package_path = Path(package_path)
        
        # Find package.xml
        package_xml = list(package_path.rglob("package.xml"))
        if not package_xml:
            self.errors.append("Missing package.xml file")
            return False
        
        self.info.append(f"Found package.xml at {package_xml[0]}")
        
        # Check for CMakeLists.txt or setup.py
        has_cmake = bool(list(package_path.rglob("CMakeLists.txt")))
        has_setup = bool(list(package_path.rglob("setup.py")))
        
        if not (has_cmake or has_setup):
            self.errors.append("Missing CMakeLists.txt or setup.py")
            return False
        
        if has_cmake:
            self.info.append("Found CMakeLists.txt (C++ package)")
        if has_setup:
            self.info.append("Found setup.py (Python package)")
        
        return True

    def validate_python_syntax(self, package_path: str) -> bool:
        """Run flake8 on Python files"""
        python_files = list(Path(package_path).rglob("*.py"))
        
        if not python_files:
            self.info.append("No Python files found")
            return True
        
        has_errors = False
        for py_file in python_files:
            result = subprocess.run(
                ["flake8", str(py_file), "--max-line-length=100", 
                 "--ignore=W293,W291,W503,F401"],
                capture_output=True,
                text=True
            )
            
            if result.returncode != 0:
                has_errors = True
                self.errors.append(f"Syntax errors in {py_file.name}:\n{result.stdout}")
            else:
                self.info.append(f"✓ {py_file.name} passed syntax check")
        
        return not has_errors

    def validate_cpp_syntax(self, package_path: str) -> bool:
        """Basic C++ syntax check"""
        cpp_files = list(Path(package_path).rglob("*.cpp"))
        
        if not cpp_files:
            self.info.append("No C++ files found")
            return True
        
        has_errors = False
        for cpp_file in cpp_files:
            result = subprocess.run(
                ["g++", "-fsyntax-only", "-std=c++14", str(cpp_file)],
                capture_output=True,
                text=True
            )
            
            if result.returncode != 0:
                has_errors = True
                self.errors.append(f"Syntax errors in {cpp_file.name}:\n{result.stderr}")
            else:
                self.info.append(f"✓ {cpp_file.name} passed syntax check")
        
        return not has_errors

    def check_ros_components(self, package_path: str) -> Dict:
        """Detect ROS2 components (nodes, publishers, subscribers)"""
        components = {
            "nodes": [],
            "publishers": [],
            "subscribers": [],
            "services": [],
            "actions": []
        }
        
        python_files = list(Path(package_path).rglob("*.py"))
        cpp_files = list(Path(package_path).rglob("*.cpp"))
        
        # Check Python files
        for py_file in python_files:
            with open(py_file, 'r') as f:
                content = f.read()
                
                if re.search(r'rclpy\.init|Node\(', content):
                    components["nodes"].append(str(py_file.name))
                
                if re.search(r'create_publisher|Publisher', content):
                    components["publishers"].append(str(py_file.name))
                
                if re.search(r'create_subscription|Subscriber', content):
                    components["subscribers"].append(str(py_file.name))
                
                if re.search(r'create_service|create_client', content):
                    components["services"].append(str(py_file.name))
        
        # Check C++ files
        for cpp_file in cpp_files:
            with open(cpp_file, 'r') as f:
                content = f.read()
                
                if re.search(r'rclcpp::init|rclcpp::Node', content):
                    components["nodes"].append(str(cpp_file.name))
                
                if re.search(r'create_publisher', content):
                    components["publishers"].append(str(cpp_file.name))
                
                if re.search(r'create_subscription', content):
                    components["subscribers"].append(str(cpp_file.name))
        
        self.info.append(f"Detected components: {components}")
        return components

    def check_motion_safety(self, package_path: str) -> bool:
        """Check for basic motion safety patterns"""
        all_files = list(Path(package_path).rglob("*.py")) + \
                    list(Path(package_path).rglob("*.cpp"))
        
        for file in all_files:
            with open(file, 'r') as f:
                content = f.read()
                
                # Check for rate limiting in loops
                if re.search(r'while.*True', content, re.IGNORECASE):
                    if not re.search(r'rate\.sleep|sleep|Rate', content):
                        self.warnings.append(
                            f"{file.name}: Infinite loop without rate limiting detected"
                        )
                
                # Check for joint limits
                joint_values = re.findall(r'joint.*=.*?([-+]?\d+\.?\d*)', content)
                for value in joint_values:
                    try:
                        val = float(value)
                        if abs(val) > 6.28:
                            self.warnings.append(
                                f"{file.name}: Potentially unsafe joint value: {val}"
                            )
                    except ValueError:
                        pass
        
        return True

    def generate_report(self, package_path: str) -> Tuple[str, Dict]:
        """Generate validation report"""
        report_dict = {
            "package_path": package_path,
            "status": "PASS" if not self.errors else "FAIL",
            "errors": self.errors,
            "warnings": self.warnings,
            "info": self.info
        }
        
        # Text report
        text_report = "=" * 60 + "\n"
        text_report += "ROS2 PACKAGE VALIDATION REPORT\n"
        text_report += "=" * 60 + "\n\n"
        text_report += f"Package: {package_path}\n"
        text_report += f"Status: {report_dict['status']}\n\n"
        
        if self.errors:
            text_report += "ERRORS:\n"
            for error in self.errors:
                text_report += f"  ✗ {error}\n"
            text_report += "\n"
        
        if self.warnings:
            text_report += "WARNINGS:\n"
            for warning in self.warnings:
                text_report += f"  ⚠ {warning}\n"
            text_report += "\n"
        
        if self.info:
            text_report += "INFO:\n"
            for info in self.info:
                text_report += f"  ℹ {info}\n"
        
        return text_report, report_dict

    def validate_package(self, zip_path: str) -> Tuple[str, Dict]:
        """Main validation pipeline"""
        self.errors = []
        self.warnings = []
        self.info = []
        
        # Extract package
        package_path = self.extract_package(zip_path)
        
        # Run checks
        self.check_package_structure(package_path)
        self.validate_python_syntax(package_path)
        self.validate_cpp_syntax(package_path)
        self.check_ros_components(package_path)
        self.check_motion_safety(package_path)
        
        # Generate report
        return self.generate_report(package_path)


if __name__ == "__main__":
    checker = ROS2CodeChecker()
    print("Code Checker module loaded successfully")
