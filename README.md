# MARY - Mobile Autonomous Rain sYstem

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-blue)](https://docs.ros.org/en/foxy/)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-orange)](https://px4.io/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

**MARY** is an autonomous umbrella drone that follows a person and provides rain protection. Developed as part of the ROB498 Robotics Capstone at the University of Toronto.

```
    _______________
   /               \
  /                 \
 /_________☔________\
          |
       ___🚁___
      /       \
     |  MARY  |
      \_______/
          |
          👤
       [Person]
```

## Features

- **Autonomous Person Following** - Uses YOLOv8 for real-time person detection and tracking
- **Altitude Control** - TeraRanger Evo 60m ToF maintains safe height above person
- **Visual-Inertial Odometry** - Intel RealSense T265 for GPS-denied localization
- **Person Switching** - Can switch between tracked individuals on command
- **Safety Systems** - Geofencing, low battery handling, emergency stop

## Hardware Setup

| Component | Model | Purpose |
|-----------|-------|---------|
| **Flight Controller** | Cube Orange+ | PX4 autopilot |
| **Companion Computer** | Jetson Nano | Perception & control |
| **ESCs** | Spedix ES30 HV30A (x4) | Motor control |
| **Tracking Camera** | Intel RealSense T265 | Visual-inertial odometry |
| **RGB Camera** | Sony IMX219 | Person detection |
| **ToF Sensor** | TeraRanger Evo 60m | Altitude measurement |
| **RC Transmitter** | FrSky Taranis X9 Lite S | Manual override |
| **RC Receiver** | FrSky Archer R4 | RC link |

### Wiring Diagram

```
                    ┌─────────────────┐
                    │   Cube Orange+  │
                    │    (PX4 FCU)    │
                    └────────┬────────┘
                             │ USB/Serial
                             │
┌────────────┐      ┌────────▼────────┐      ┌────────────┐
│  T265      │ USB  │                 │ USB  │ TeraRanger │
│  Camera    ├──────┤   Jetson Nano   ├──────┤  Evo 60m   │
└────────────┘      │                 │      └────────────┘
                    │                 │
┌────────────┐ CSI  │                 │
│  IMX219    ├──────┤                 │
│  Camera    │      └─────────────────┘
└────────────┘
```

## Software Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        MARY System                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────┐     ┌─────────────────┐              │
│  │   PERCEPTION    │     │    CONTROL      │              │
│  ├─────────────────┤     ├─────────────────┤              │
│  │ • person_tracker│     │ • follower_node │              │
│  │ • t265_pose     │────▶│ • altitude_ctrl │              │
│  │ • tof_processor │     │ • mission_mgr   │              │
│  └─────────────────┘     │ • comm_node     │              │
│                          └────────┬────────┘              │
│                                   │                       │
│  ┌─────────────────┐              │                       │
│  │    HARDWARE     │              │                       │
│  ├─────────────────┤              │                       │
│  │ • MAVROS        │◀─────────────┘                       │
│  │ • Sensor drivers│                                      │
│  └────────┬────────┘                                      │
│           │                                               │
└───────────┼───────────────────────────────────────────────┘
            │
            ▼
    ┌───────────────┐
    │  Cube Orange+ │
    │    (PX4)      │
    └───────────────┘
```

## Repository Structure

```
ROB498-SkyNet/
├── mary_ws/                          # ROS2 Workspace
│   └── src/
│       ├── mary_perception/          # Perception nodes
│       │   └── mary_perception/
│       │       ├── person_tracker_node.py   # YOLO person detection
│       │       ├── tof_processor_node.py    # ToF altitude processing
│       │       └── t265_pose_node.py        # T265 pose processing
│       │
│       ├── mary_control/             # Control nodes
│       │   └── mary_control/
│       │       ├── follower_node.py         # Person following
│       │       ├── altitude_controller_node.py  # PID altitude
│       │       ├── mission_manager_node.py  # State machine
│       │       └── comm_node.py             # MAVROS interface
│       │
│       ├── mary_hardware/            # Hardware config
│       │   ├── config/
│       │   │   ├── px4_config.yaml          # PX4 parameters
│       │   │   └── px4_pluginlists.yaml     # MAVROS plugins
│       │   └── launch/
│       │       ├── mavros.launch.py         # MAVROS launch
│       │       └── sensors.launch.py        # Sensor launch
│       │
│       ├── mary_msgs/                # Custom messages
│       │   ├── msg/
│       │   │   ├── PersonDetection.msg
│       │   │   ├── TrackingStatus.msg
│       │   │   └── MissionStatus.msg
│       │   └── srv/
│       │       ├── SetTarget.srv
│       │       └── SwitchPerson.srv
│       │
│       └── mary_bringup/             # System launch
│           ├── config/
│           │   └── mary_params.yaml         # All parameters
│           └── launch/
│               └── mary_full.launch.py      # Full system launch
│
├── scripts/                          # Utility scripts
├── models/                           # ML models (YOLO weights)
├── docs/                             # Documentation
└── test/                             # Test files
```

## Installation

### Prerequisites

- Ubuntu 20.04
- ROS2 Foxy
- Python 3.8+
- CUDA (for Jetson Nano GPU acceleration)

### 1. Install ROS2 Foxy

```bash
# Add ROS2 repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-foxy-desktop ros-dev-tools
```

### 2. Install Dependencies

```bash
# MAVROS
sudo apt install ros-foxy-mavros ros-foxy-mavros-extras
sudo /opt/ros/foxy/lib/mavros/install_geographiclib_datasets.sh

# RealSense
sudo apt install ros-foxy-realsense2-camera

# OpenCV and vision
sudo apt install python3-opencv

# YOLO (Ultralytics)
pip3 install ultralytics

# TF transformations (required for T265 pose processing)
pip3 install tf_transformations transforms3d

# Other Python dependencies
pip3 install numpy scipy
```

### 3. Build the Workspace

```bash
# Clone repository
cd ~/
git clone https://github.com/YOUR_USERNAME/ROB498-SkyNet.git
cd ROB498-SkyNet

# Source ROS2
source /opt/ros/foxy/setup.bash

# Build
cd mary_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### 4. PX4 Configuration

1. Connect Cube Orange+ to QGroundControl
2. Set these parameters:
   ```
   EKF2_AID_MASK = 24        # Use vision position and velocity
   EKF2_HGT_MODE = 2         # Use vision for height
   EKF2_EV_DELAY = 50        # Vision delay compensation (ms)
   MAV_ODOM_LP = 1           # Enable vision pose input
   ```

### 5. Download YOLO Model

```bash
cd ~/ROB498-SkyNet/models
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

## Usage

### Quick Start

```bash
# Terminal 1: Launch full system
source ~/ROB498-SkyNet/mary_ws/install/setup.bash
ros2 launch mary_bringup mary_full.launch.py

# Terminal 2: Start mission
ros2 service call /mary/mission/start std_srvs/srv/Trigger

# Terminal 3: Monitor status
ros2 topic echo /mary/mission/status
```

### Manual Control

```bash
# Arm
ros2 service call /mary/comm/arm std_srvs/srv/Trigger

# Switch to OFFBOARD mode
ros2 service call /mary/comm/offboard std_srvs/srv/Trigger

# Emergency stop
ros2 service call /mary/mission/emergency_stop std_srvs/srv/Trigger
```

### Adjust Parameters

```bash
# Change target altitude (2.0 meters above person)
ros2 service call /mary/control/set_target mary_msgs/srv/SetTarget "{target_altitude: 2.0, max_speed: 2.0}"

# Switch to new person
ros2 service call /mary/mission/switch_person std_srvs/srv/Trigger
```

## ROS2 Topics

### Perception
| Topic | Type | Description |
|-------|------|-------------|
| `/mary/perception/person_position` | `PointStamped` | Person position in camera frame |
| `/mary/perception/person_detected` | `Bool` | Detection status |
| `/mary/perception/altitude` | `Float32` | Filtered altitude (ToF) |
| `/mary/localization/pose` | `PoseStamped` | Drone pose (T265) |

### Control
| Topic | Type | Description |
|-------|------|-------------|
| `/mary/mission/state` | `String` | Current mission state |
| `/mary/mission/status` | `String` | Human-readable status |
| `/mavros/setpoint_velocity/cmd_vel` | `TwistStamped` | Velocity commands |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/mary/mission/start` | `Trigger` | Start mission |
| `/mary/mission/stop` | `Trigger` | Land and stop |
| `/mary/mission/switch_person` | `Trigger` | Switch to new person |
| `/mary/mission/emergency_stop` | `Trigger` | Emergency stop |

## Safety Features

1. **Altitude Limits**: Min 1.0m, Max 5.0m
2. **Speed Limits**: Horizontal 2.0 m/s, Vertical 1.0 m/s
3. **Battery Monitoring**: Warning at 14.0V, Land at 13.2V
4. **Detection Timeout**: Hovers if person lost for >2 seconds
5. **Manual Override**: RC transmitter always has control

## Troubleshooting

### MAVROS not connecting
```bash
# Check USB connection
ls /dev/ttyACM*

# Try different baud rate
ros2 launch mary_hardware mavros.launch.py fcu_url:=/dev/ttyACM0:57600
```

### T265 not detected
```bash
# Check RealSense
rs-enumerate-devices

# Restart udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Person not detected
- Ensure adequate lighting
- Check camera is not obstructed
- Lower confidence threshold in params

## Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## License

MIT License - see [LICENSE](LICENSE) for details.

## Authors

- **Your Name** - University of Toronto - ROB498 Capstone

## Acknowledgments

- ROB498 Teaching Team
- PX4 Autopilot Community
- ROS2 Community
