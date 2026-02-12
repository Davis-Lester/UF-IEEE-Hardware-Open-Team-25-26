cmake_minimum_required(VERSION 3.8)
project(hardware_team_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_library(PIGPIO_LIB pigpio REQUIRED)


# Create library for sensors
add_library(sensor_lib SHARED 
    #src/sensors/GY31.cpp THIS FILE NEEDS TO BE FIXED BEFORE ITS ADDED
    src/sensors/MPU6050.cpp
)

target_include_directories(sensor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sensor_lib rclcpp std_msgs sensor_msgs nav_msgs)


# WILL HAVE TO ADD ODOMETRY AND AUTON NODES LATER

# Create nodes
# Chassis Node
add_executable(chassis_node 
    src/chassis_node.cpp 
)
target_link_libraries(chassis_node sensor_lib ${PIGPIO_LIB})
ament_target_dependencies(chassis_node rclcpp rclcpp_action)

# IR Node
add_executable(ir_node src/ir_node.cpp) # IR node likely doesn't need odometry/IMU
target_link_libraries(ir_node ${PIGPIO_LIB})
ament_target_dependencies(ir_node rclcpp std_msgs)


install(TARGETS
  sensor_lib
  chassis_node
  ir_node
  DESTINATION lib/${PROJECT_NAME}
)

# This ensures your headers are copied to the install space
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()