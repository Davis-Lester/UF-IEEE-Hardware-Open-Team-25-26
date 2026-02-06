cmake_minimum_required(VERSION 3.8)
project(Hardware_Team_Robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# 1. Update Subsystem Library Sources
# Group the subsystem files together
set(SUBSYSTEM_SOURCES
    src/subsystems/GY31.cpp
    src/subsystems/MPU6050.cpp
    src/subsystems/Odometry.cpp
)

# 2. Update Nodes to include these sources
add_executable(chassis_node 
    src/chassis_node.cpp 
    ${SUBSYSTEM_SOURCES}
)
ament_target_dependencies(chassis_node rclcpp rclcpp_action)
rosidl_target_interfaces(chassis_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(chassis_node pigpio)

# Do the same for other nodes if they use subsystems
add_executable(ir_node src/ir_node.cpp) # IR node likely doesn't need odometry/IMU
ament_target_dependencies(ir_node rclcpp std_msgs)
target_link_libraries(ir_node pigpio)