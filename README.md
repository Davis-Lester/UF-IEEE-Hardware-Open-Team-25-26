# Hardware_Team_Robot (ROS 2 Jazzy)

This repository contains the ROS 2 **Jazzy Jalisco** control software for a Raspberry Pi 4-based competitive robot. It features a custom Mecanum drive controller, hardware-accelerated drivers for sensors (MPU6050, GY-31, HC-SR04), IR communication, and an autonomous action server.

## 📂 Project Structure

The project uses a modular architecture where hardware drivers are separated from the ROS nodes that utilize them.

```text
GITHUB/
├── action/
│   └── Drive.action              # Custom Action: Goal (mode, target_value, max_speed)
├── launch/
│   └── robot_launch.py           # Starts Chassis, IR, and Camera nodes
├── include/hardware_team_robot/
│   ├── sensors/                  # Hardware Driver Headers
│   │   ├── GY31.h                # Color Sensor (TCS3200)
│   │   ├── MPU6050.h             # IMU Driver
│   │   └── UltrasonicDistance.h  # Asynchronous Ping/Echo Driver
│   ├── Subsystems/               # (Reserved for complex logic modules)
│   ├── auton_routine.h           # Autonomous Client Header
│   ├── camera_node.h             # Camera Node Header
│   ├── chassis_node.h            # Main PID Controller & Threaded Odometry Header
│   ├── encoder_driver.h          # Hardware Interrupt Encoder Driver
│   ├── ir_node.h                 # IR Communication Header
│   ├── mecanum_odometry.h        # Kinematics & Pose Math
│   ├── navigation_controller.h   # Position-based PID Navigation
│   └── odometry.h                # Legacy Odometry Math
└── src/
    ├── sensors/                  # Hardware Driver Implementations
    │   ├── GY31.cpp
    │   ├── MPU6050.cpp
    │   └── UltrasonicDistance.cpp
    ├── Subsystems/               # (Reserved)
    ├── auton_routine.cpp         # Autonomous Action Client
    ├── camera_node.cpp           # Camera Node (Skeleton)
    ├── chassis_node.cpp          # Main Controller (PID, Motors, Odometry Loop)
    ├── encoder_driver.cpp        # Thread-safe Encoder Counting
    ├── ir_node.cpp               # IR Node (NEC Protocol via DMA)
    ├── mecanum_odometry.cpp      # Odometry State & Math
    ├── navigation_controller.cpp # Coordinate-to-Motor Output
    ├── odometry_test.cpp         # Simulation & Validation Harness
    └── odometry.cpp
```

## ⚡ Features


* **Mecanum Kinematics:** Full omnidirectional control (Forward, Strafe, Turn) using 4x DC Motors.
* **Threaded Sensor Fusion Odometry:** Runs a 100Hz background thread combining high-speed encoder ticks, IMU gyro integration, and 10Hz Ultrasonic wall-distance fusion to actively correct drift.
* **Custom PID Control:** Closed-loop velocity and position control running directly on the Pi.
* **Hardware Acceleration:** Utilizes `pigpio` DMA waveforms for precise IR signal generation and Motor PWM, bypassing Linux OS jitter. Hardware interrupts handle all encoder and ultrasonic timing.
* **Standardized Action Server:** Autonomous routines utilize a unified `target_value` parameter via ROS 2 Actions to execute blocking commands with real-time error feedback.

## 🛠️ Hardware Specifications

* **Compute:** Raspberry Pi 4 Model B (Ubuntu 24.04 / Debian Bookworm)
* **Motors:** 4x Metal Gearmotors (19:1 Ratio) with 64 CPR Encoders (~1216 ticks/rev).
* **IMU:** MPU6050 (I2C).
* **Ultrasonic Sensors:** 2x HC-SR04 (Hardware Interrupt Trigger/Echo).
* **Color Sensor:** GY-31 / TCS3200 (Frequency Scaling).
* **IR:** 38kHz NEC Protocol Transmitter/Receiver.

## 🚀 Installation & Setup

### 1. Prerequisites
Ensure you are running a 64-bit OS compatible with **ROS 2 Jazzy**.

```bash
# 1. Install pigpio (Required for Hardware PWM/DMA/Interrupts)
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
ros2 launch hardware_team_robot robot_launch.py
```

**Option B: Debugging Individual Nodes**
```bash
# Example: Run just the chassis node
ros2 run hardware_team_robot chassis_node
```

## 🚧 Roadmap & Required Updates

This section outlines the immediate next steps required to fully enable Odometry-based Navigation, Hardware Migrations, and Camera functionality.

### 1. Integrate the `NavigationController`
The odometry engine is now running in the background and fusing ultrasonic data. The next step is to upgrade from "Tick-Based" driving to "Coordinate-Based" driving.
* **Goal:** Allow the `auton_routine` to command coordinates (e.g., `moveToPose(24.0, 48.0, 90.0)`) instead of raw ticks.
* **Action:** Hook the `NavigationController` class up to the `Drive` action server inside `chassis_node.cpp` to replace the legacy PID loops.

### 2. Hardware Migration: I2C Color Sensor
Due to GPIO exhaustion, the GY-31 frequency-based color sensor is being deprecated.
* **Goal:** Free up 5 GPIO pins and improve lighting consistency.
* **Action:** Swap the GY-31 for a VEML7700. Write a new I2C driver in `src/sensors/` and publish a standard `std_msgs/msg/ColorRGBA` to the ROS network.

### 3. Enabling the Camera (`camera_node.cpp`)
The `src/camera_node.cpp` file is currently a skeleton.
* **Goal:** Capture video frames and detect game objects (balls/tags).
* **Action:** Add `opencv` and `cv_bridge` to your build system. Use OpenCV to capture the Pi Camera stream and publish it to `/camera/image_raw`.

### 4. Pin Configuration Verification
Pin mappings are strictly defined as `constexpr` in `include/hardware_team_robot/chassis_node.h`. **Verify these match your physical wiring before powering on to prevent hardware damage.**
* **Encoders:** Configured for 3.3V Logic.
* **Motors:** PWM Frequency is handled by `pigpio`.
* **I2C Sensors:** MPU6050 must be on I2C Bus 1 (GPIO 2/3).

## 🤝 Contributing

1.  Create a new branch for your feature (`git checkout -b feature/AmazingSensor`).
2.  Commit your changes.
3.  Push to the branch and open a Pull Request.
