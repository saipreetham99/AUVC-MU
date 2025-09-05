# Autonomous Underwater Vehicle Control System

A hybrid MuJoCo-Unity simulation environment for autonomous underwater vehicle development, featuring real-time computer vision and reinforcement learning-based control systems.

## Overview

This project implements a comprehensive AUV control system combining physics-based simulation, computer vision, and autonomous navigation. The system uses a BlueRobotics BlueROV2 platform with custom software stack for underwater object detection and tracking.


![MuJoCo-Unity Simulation](https://raw.githubusercontent.com/saipreetham99/AUVC-MU/main/Assets/sim.png)
*Hybrid MuJoCo-Unity simulation environment with physics-based underwater dynamics*




## Key Features

### Hybrid Simulation Environment
- **MuJoCo-Unity Integration**: Physics engine providing high-fidelity underwater dynamics
- **Buoyancy Physics**: Realistic water density, gravity compensation, and submersion effects
- **Real-time Visualization**: Unity-based 3D environment for testing and validation
- **Domain Adaptation**: Synthetic dataset generation for sim-to-real transfer

### Computer Vision Pipeline  
- **YOLOv8-OBB Architecture**: Custom object detection optimized for underwater environments
- **11ms Inference Latency**: Real-time performance with custom anchor optimization
- **Bounding Box Tracking**: Automated target detection and following capabilities
- **April Tag Integration**: Target identification and pose estimation

![YOLO Object Detection](https://raw.githubusercontent.com/saipreetham99/AUVC-MU/main/Assets/yolo.jpeg)

*Real-time underwater vehicle detection using YOLOv8 with 0.86 confidence*

### Autonomous Control System
- **State Machine Architecture**: Three-state control (Searching, Centering/Advancing, Circling)
- **PID Controllers**: Depth and lateral position control with buoyancy compensation
- **Sensor Fusion**: IMU data integration for orientation and navigation
- **Socket-based Communication**: UDP server for real-time telemetry and control


![Telemetry](https://raw.githubusercontent.com/saipreetham99/AUVC-MU/main/Assets/control.jpeg)



*Real-time IMU monitoring with PID-based correction for yaw, pitch, and roll.

## Architecture

### Hardware Components
- **Platform**: BlueRobotics BlueROV2
- **Propulsion**: 6 motors at 45-degree angles for omnidirectional movement
- **Electronics**: Raspberry Pi with Navigator board
- **Sensors**: IMU, depth sensor, camera system
- **Power**: Battery module in waterproof housing
## Technical Implementation

### MuJoCo Physics Engine (C#)
The core simulation implements underwater physics through the `MjScene.cs` system:
- Water surface height adjustment
- Viscosity and timestep control  
- Buoyancy force calculations per corner site
- Applied force distribution across 6 thrusters

### Socket Communication
UDP server handles real-time commands:
- `GET_MODEL_INFO`: Simulation parameters
- `GET_SENSORDATA`: IMU and timing data
- `APPLY_CTRL`: Thruster force application
- `STEP_SIM`: Physics step execution
- `RESET`: System state reset
- `GET_TARGET_BBOX`: Bounding box coordinates

### Control Algorithms
- **PID Control**: Depth and position stabilization
- **Force Mapping**: Site-specific thrust vector calculation
- **State Management**: Autonomous behavior switching
- **Error Compensation**: Real-time parameter tuning

## Installation & Setup

```bash
# Clone repository
git clone https://github.com/username/auv-control-system
cd auv-control-system

# Install Python dependencies
pip install -r requirements.txt

# Install Unity packages (MuJoCo integration)
# Open project in Unity Editor



