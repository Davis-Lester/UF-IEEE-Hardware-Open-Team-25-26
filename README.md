# Hardware_Team_Robot (ROS 2 Jazzy)

This repository contains the ROS 2 **Jazzy Jalisco** control software for a Raspberry Pi 4-based competitive robot. It features a custom Mecanum drive controller, hardware-accelerated drivers for sensors (MPU6050, GY-31), IR communication, and an autonomous action server.

## 📂 Project Structure

The project uses a modular architecture where hardware drivers are separated from the ROS nodes that utilize them.

```text
GITHUB/
├── action/
│   └── Drive.action          # Custom Action: Goal (ticks/angle), Result (success)
├── launch/
│   └── robot_launch.py       # Starts Chassis, IR, and Camera nodes
├── include/Hardware_Team_Robot/
│   ├── Sensors/              # Hardware Driver Headers
│   │   ├── GY31.h            # Color Sensor (TCS3200)
│   │   └── MPU6050.h         # IMU Driver
│   ├── Subsystems/           # (Reserved for complex logic modules)
│   ├── auton_routine.h       # Autonomous Client Header
│   ├── camera_node.h         # Camera Node Header
│   ├── chassis_node.h        # Main PID Controller Header
│   ├── ir_node.h             # IR Communication Header
│   └── odometry.h            # Odometry Math Class
└── src/
    ├── Sensors/              # Hardware Driver Implementations
    │   ├── GY31.cpp
    │   └── MPU6050.cpp
    ├── auton_routine.cpp     # Autonomous Action Client
    ├── camera_node.cpp       # Camera Node (Skeleton)
    ├── chassis_node.cpp      # Main Controller (PID, Motors, Encoders)
    ├── ir_node.cpp           # IR Node (NEC Protocol via DMA)
    └── Subsystems/           # (Reserved)
```

## ⚡ Features

* **Mecanum Kinematics:** Full omnidirectional control (Forward, Strafe, Turn) using 4x DC Motors.
* **Custom PID Control:** Closed-loop velocity and position control running directly on the Pi.
* **Hardware Acceleration:** Utilizes `pigpio` DMA waveforms for precise IR signal generation and Motor PWM, bypassing Linux OS jitter.
* **Sensor Fusion:** MPU6050 Gyroscope integration for accurate turning.
* **Action Server:** Autonomous routines utilize ROS 2 Actions to execute blocking commands (e.g., "Drive 1 meter") with feedback.

## 🛠️ Hardware Specifications

* **Compute:** Raspberry Pi 4 Model B (Ubuntu 24.04 / Debian Bookworm)
* **Motors:** 4x Metal Gearmotors (19:1 Ratio) with 64 CPR Encoders (~1216 ticks/rev).
* **IMU:** MPU6050 (I2C).
* **Color Sensor:** GY-31 / TCS3200 (Frequency Scaling).
* **IR:** 38kHz NEC Protocol Transmitter/Receiver.

## 🚀 Installation & Setup

### 1. Prerequisites
Ensure you are running a 64-bit OS compatible with **ROS 2 Jazzy**.

```bash
# 1. Install pigpio (Required for Hardware PWM/DMA)
sudo apt-get update
sudo apt-get install pigpio

# 2. Install ROS 2 Build Tools
sudo apt install python3-colcon-common-extensions
```

### 2. Build the Workpspace
Clone this repository into your ROS 2 workspace `src` directory.
```bash
cd ~/ros2_ws/src
git clone <your-repo-url> Hardware_Team_Robot
cd ~/ros2_ws

# Build the package
colcon build --symlink-install

# Source the overlay
source install/setup.bash
```

### 3. Running the Robot
Because `pigpio` interacts directly with hardware registers, the daemon must be running.
**Option A: The Standard Way** 

```bash
# Start the pigpio daemon
sudo pigpiod

# Launch the robot stack
ros2 launch Hardware_Team_Robot robot_launch.py
```

**Option B: Debugging Individual Nodes**
```bash
# Example: Run just the chassis node
ros2 run Hardware_Team_Robot chassis_node
```

## 🚧 Roadmap & Required Updates

This section outlines the immediate next steps required to fully enable Odometry and Camera functionality.

### 1. Implementing Odometry (`odometry.h`)
The `Odometry` class exists but is not yet fully integrated into the `chassis_node` loop.

* **Goal:** Publish `nav_msgs/Odometry` messages to the `/odom` topic so the robot knows where it is on the field (X, Y, Theta).
* **Implementation Steps:**
    1.  Open `src/chassis_node.cpp`.
    2.  Instantiate the `Odometry` object as a member variable.
    3.  In the main control loop (`execute`), calculate the *change* in encoder ticks for all 4 wheels since the last loop.
    4.  Call `odometry_.update(fl_delta, fr_delta, rl_delta, rr_delta, current_gyro_heading)`.
    5.  Create a ROS Publisher for `nav_msgs/msg/Odometry`.
    6.  Publish the calculated X and Y coordinates.

### 2. Enabling the Camera (`camera_node.cpp`)
The `src/camera_node.cpp` file is currently a skeleton.

* **Goal:** Capture video frames and detect game objects (balls/tags).
* **Implementation Steps:**
    1.  **Add Dependencies:** Add `opencv` and `cv_bridge` to your `package.xml` and `CMakeLists.txt`.
    2.  **Capture Loop:** Inside `camera_node.cpp`, use `cv::VideoCapture(0)` to open the Pi Camera.
    3.  **ROS Publishing:** Convert the OpenCV frame to a ROS message using `cv_bridge`.
    4.  **Publish:** Send the image to the `/camera/image_raw` topic.

### 3. Pin Configuration
Pin mappings are defined in `include/Hardware_Team_Robot/chassis_node.h`. **Verify these match your physical wiring before powering on.**

* **Encoders:** Configured for 3.3V Logic.
* **Motors:** PWM Frequency is handled by `pigpio`.
* **I2C Sensors:** MPU6050 must be on I2C Bus 1 (GPIO 2/3).

## 🤝 Contributing

1.  Create a new branch for your feature (`git checkout -b feature/AmazingSensor`).
2.  Commit your changes.
3.  Push to the branch and open a Pull Request.