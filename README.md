# PyBullet Differential Drive Robot Simulation

A comprehensive PyBullet-based simulation environment for a differential drive robot with sensor streams, navigation capabilities, and RL training support.

## Features

- ✅ **3D Physics Simulation**: Full PyBullet physics with realistic robot dynamics
- ✅ **Differential Drive Robot**: Custom URDF model with wheels, base, and sensors
- ✅ **Real-time Sensor Streams**: Lidar, RGB camera, depth camera, IMU, and odometry
- ✅ **Navigation Capabilities**: Goal-based navigation with obstacle avoidance
- ✅ **Sensor Data Logging**: Record and analyze sensor data for training
- ✅ **RL-Ready**: Structured for Gymnasium environment integration

## Quick Start

### Prerequisites

- Python 3.8-3.11 (Python 3.12+ may require conda)
- Conda (recommended) or pip with virtual environment

### Installation

#### Option 1: Using Conda (Recommended)

```bash
# Clone or navigate to the project
cd pybullet_sim

# Create conda environment
conda create -n pybullet_env python=3.11
conda activate pybullet_env

# Install PyBullet via conda
conda install -c conda-forge pybullet

# Install other dependencies
pip install -r requirements.txt
```

#### Option 2: Using pip (may have issues on macOS/Apple Silicon)

```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
```

**Note**: If pip installation fails, see [INSTALL.md](INSTALL.md) for troubleshooting.

### Verify Installation

```bash
python scripts/00_check_setup.py
```

## Project Structure

```
pybullet_sim/
├── urdf/                          # Robot URDF files
│   └── diff_drive_robot.urdf     # Differential drive robot model
├── scripts/                       # Python scripts
│   ├── diff_drive_robot.py       # Robot controller class with sensors
│   ├── 00_check_setup.py        # Setup verification
│   ├── 01_test_pybullet.py      # Basic robot movement test
│   ├── 02_simple_navigation.py   # Simple goal navigation
│   ├── 03_sensor_streams.py      # Real-time sensor visualization
│   ├── 04_sensor_logger.py       # Sensor data recording
│   └── diagnose_pybullet.py      # Installation diagnostics
├── models/                        # Trained RL models (created during training)
├── logs/                          # Logs and sensor data
│   └── sensor_data/              # Recorded sensor data
├── configs/                       # Configuration files
├── assets/                        # Additional assets
├── requirements.txt               # Python dependencies
├── README.md                      # This file
├── SETUP.md                       # Detailed setup instructions
└── INSTALL.md                     # Installation troubleshooting
```

## Usage

### 1. Basic Robot Movement Test

```bash
python scripts/01_test_pybullet.py
```

Tests basic robot movement: forward, backward, and turning motions.

### 2. Simple Navigation

```bash
python scripts/02_simple_navigation.py
```

Robot navigates to a goal position using a simple proportional controller.

### 3. Real-time Sensor Streams

```bash
python scripts/03_sensor_streams.py
```

Displays live sensor data in a matplotlib dashboard:
- **Lidar Scan**: 2D top-down view with color-coded rays
- **RGB Camera**: Live feed from robot's perspective
- **Depth Camera**: Depth map visualization
- **IMU Data**: Accelerometer and gyroscope readings
- **Odometry**: Wheel velocities and robot velocity
- **Trajectory**: Path visualization

**Note**: The matplotlib window may stay on top. You can minimize it or use Alt+Tab to switch windows.

### 4. Sensor Data Logging

```bash
python scripts/04_sensor_logger.py
```

Records all sensor data to JSON files and saves camera images in `logs/sensor_data/`.

## Robot Specifications

- **Type**: Differential drive robot
- **Wheel radius**: 0.05 m
- **Wheel base**: 0.22 m
- **Max linear velocity**: 0.5 m/s
- **Max angular velocity**: 2.0 rad/s

### Sensors

- **Lidar**: 36-ray 2D lidar, 5m range
- **RGB Camera**: 320x240 resolution, 60° FOV
- **Depth Camera**: Same resolution, 0.01-10m range
- **IMU**: Simulated accelerometer and gyroscope (with noise)
- **Odometry**: Wheel encoder simulation

## Scripts Overview

| Script | Description |
|--------|-------------|
| `00_check_setup.py` | Verifies all dependencies are installed |
| `01_test_pybullet.py` | Basic robot movement demonstration |
| `02_simple_navigation.py` | Simple goal-based navigation |
| `03_sensor_streams.py` | Real-time sensor visualization dashboard |
| `04_sensor_logger.py` | Record sensor data for analysis |
| `diagnose_pybullet.py` | Diagnose installation issues |

## API Usage

### Using the Robot Class

```python
from scripts.diff_drive_robot import DiffDriveRobot
import pybullet as p

# Initialize PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# Create robot
robot = DiffDriveRobot("urdf/diff_drive_robot.urdf", start_pos=[0, 0, 0.1])

# Control robot
robot.set_velocity(v=0.3, w=0.0)  # Move forward

# Get sensor data
pose = robot.get_pose()                    # [x, y, theta]
lidar = robot.get_lidar_scan()             # Array of distances
rgb, depth = robot.get_camera_image()      # Camera images
imu = robot.get_imu_data()                 # IMU readings
odom = robot.get_odometry()                # Odometry data
```

## Troubleshooting

### Installation Issues

See [INSTALL.md](INSTALL.md) for detailed troubleshooting:
- PyBullet build failures
- "No matching distribution" errors
- Apple Silicon (M1/M2/M3) issues
- Python version compatibility

### Common Issues

**Matplotlib window stays on top:**
- This is a known matplotlib behavior
- Use Alt+Tab (Cmd+Tab on Mac) to switch windows
- Or minimize the matplotlib window

**PyBullet GUI doesn't open:**
- Ensure you're using `p.GUI` mode (not `p.DIRECT`)
- On macOS, may need XQuartz for older systems
- Try: `pip install --upgrade pybullet`

**Import errors:**
- Ensure conda/virtual environment is activated
- Verify installation: `python scripts/00_check_setup.py`

## Next Steps

1. **Environment Setup**: Create warehouse/street environments with obstacles
2. **Advanced Navigation**: Implement path planning and obstacle avoidance
3. **RL Training**: Set up Gymnasium environment for reinforcement learning
4. **Sim-to-Real**: Prepare for real robot deployment

## Contributing

Feel free to submit issues, fork the repository, and create pull requests.

## License

See [LICENSE](LICENSE) file for details.

## Acknowledgments

- PyBullet physics engine
- Stable-Baselines3 for RL algorithms
- Gymnasium for RL environment interface
