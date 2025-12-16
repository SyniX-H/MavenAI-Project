# ROS2 Code Checker & Simulator
**Powered by MavenAI**

> Advanced Robotics Testing Platform for validating and simulating ROS2 packages

---

## 🎯 Overview

This system provides a complete pipeline for testing ROS2 robotic code:
- **Code Validation**: Syntax, structure, and safety checks
- **Gazebo Simulation**: 6-DOF robotic arm with pick-and-place scenario
- **Web Interface**: User-friendly platform for testing

Built as part of the MavenAI Robotics Internship Assignment.

---

## 🎯 Features

### Code Checker
- ✅ Validates package structure (`package.xml`, `setup.py`, `CMakeLists.txt`)
- ✅ Python syntax checking with flake8
- ✅ C++ syntax validation with g++
- ✅ Detects ROS2 components (nodes, publishers, subscribers, services)
- ✅ Safety checks (joint limits, rate limiting, infinite loops)
- ✅ Generates detailed text and JSON reports

### Simulation Runner
- ✅ Launches Gazebo Fortress with 3D visualization
- ✅ Loads UR5 6-DOF robotic arm
- ✅ Creates test scene (ground plane, cube, target position)
- ✅ Captures simulation states and screenshots
- ✅ Records joint motions and simulation logs

### Web Interface
- ✅ Drag-and-drop file upload
- ✅ Real-time validation feedback
- ✅ Color-coded status indicators
- ✅ Simulation controls and duration settings
- ✅ Visual results with screenshots
- ✅ Detailed log viewer

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Fortress (Ignition Gazebo)
- Python 3.10+

### Installation
```bash
# Clone the repository
git clone https://github.com/SyniX-H/MavenAI-Project.git
cd MavenAI-Project

# Install system dependencies
sudo apt update
sudo apt install -y ros-humble-desktop-full
sudo apt install -y ignition-fortress
sudo apt install -y python3-pip python3-colcon-common-extensions

# Install Python dependencies
pip3 install flask flask-cors flake8

# Source ROS2
source /opt/ros/humble/setup.bash
```

### Running the Tool

1. **Start the Flask backend:**
```bash
python3 app.py
```

2. **Open web browser:**
```
http://localhost:5000
```

3. **Upload and test:**
   - Upload a ROS2 package (ZIP file)
   - Click "Run Validation Check"
   - Click "Launch Simulation" if validation passes

## 📁 Project Structure
```
MavenAI-Project/
├── backend/
│   ├── checker/
│   │   └── ros2_checker.py       # Code validation logic
│   └── simulator/
│       └── gazebo_runner.py      # Gazebo simulation runner
├── templates/
│   └── index.html                # Web interface
├── test_packages/
│   ├── correct_package.zip       # Valid test package
│   └── faulty_package.zip        # Invalid test package
├── app.py                        # Flask backend server
├── package_builder.py            # Test package generator
└── README.md
```

## 🧪 Test Packages

Two test packages are included:

### Correct Package (`correct_package.zip`)
- ✅ Valid ROS2 package structure
- ✅ Clean Python syntax
- ✅ Proper ROS2 components (publisher, subscriber, timer)
- ✅ Safe joint values and rate limiting
- **Expected result:** PASS

### Faulty Package (`faulty_package.zip`)
- ❌ Missing `package.xml`
- ❌ Python syntax errors (indentation)
- ❌ Unsafe joint values (999.0)
- ❌ Infinite loop without rate limiting
- ❌ Undefined variables
- **Expected result:** FAIL with detailed error messages

### Creating Test Packages
```bash
python3 package_builder.py
```

This generates both test packages in `test_packages/`.

## 📊 Usage Examples

### Example 1: Validating a Correct Package
```bash
# Upload: correct_package.zip
# Result:
============================================================
ROS2 PACKAGE VALIDATION REPORT
============================================================
Status: PASS

INFO:
  ℹ Found package.xml
  ℹ Found setup.py (Python package)
  ℹ ✓ pick_place_node.py passed syntax check
  ℹ Detected components: {'nodes': ['pick_place_node.py'], 
     'publishers': ['pick_place_node.py']}
```

### Example 2: Validating a Faulty Package
```bash
# Upload: faulty_package.zip
# Result:
============================================================
ROS2 PACKAGE VALIDATION REPORT
============================================================
Status: FAIL

ERRORS:
  ✗ Missing package.xml file
  ✗ Syntax errors in bad_node.py:
    - IndentationError at line 10
  
WARNINGS:
  ⚠ Infinite loop without rate limiting detected
  ⚠ Potentially unsafe joint value: 999.0
```

### Example 3: Running Simulation
```bash
# After validation passes:
# Click "Launch Simulation"
# Duration: 30 seconds

# Results:
{
  "status": "completed",
  "duration": 30,
  "logs": [
    "World file created with robot arm",
    "Gazebo launched successfully",
    "Initial screenshot captured",
    "Captured 30 state snapshots",
    "Final screenshot captured"
  ],
  "screenshots": ["start.png", "end.png"]
}
```

## 🔧 API Endpoints

### REST API Documentation

#### `POST /api/upload`
Upload a ROS2 package (ZIP file)

**Response:**
```json
{
  "success": true,
  "filename": "package.zip",
  "filepath": "uploads/package.zip"
}
```

#### `POST /api/check`
Validate uploaded package

**Request:**
```json
{
  "filepath": "uploads/package.zip"
}
```

**Response:**
```json
{
  "success": true,
  "json_report": {
    "status": "PASS",
    "errors": [],
    "warnings": [],
    "info": [...]
  }
}
```

#### `POST /api/simulate`
Run Gazebo simulation

**Request:**
```json
{
  "package_path": "/tmp/ros2_uploads/package",
  "duration": 30
}
```

**Response:**
```json
{
  "success": true,
  "report": {
    "status": "completed",
    "logs": [...],
    "screenshots": ["start.png", "end.png"]
  }
}
```

#### `GET /api/status`
Health check

**Response:**
```json
{
  "status": "running",
  "version": "1.0.0"
}
```

## 🧪 Testing

### Manual Testing
```bash
# Test code checker
python3 -c "from backend.checker.ros2_checker import ROS2CodeChecker; \
            checker = ROS2CodeChecker(); \
            print('✓ Code Checker loaded')"

# Test simulation runner
python3 -c "from backend.simulator.gazebo_runner import GazeboSimRunner; \
            runner = GazeboSimRunner(); \
            print('✓ Simulation Runner loaded')"

# Test Flask server
curl http://localhost:5000/api/status
```
