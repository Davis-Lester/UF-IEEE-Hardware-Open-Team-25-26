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
    # src/sensors/GY31.cpp #NEEDS TO BE FIXED
    src/sensors/MPU6050.cpp
    src/odometry.cpp
    src/navigation_controller.cpp
    src/mecanum_odometry.cpp
    src/encoder_driver.cpp
)

target_include_directories(sensor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sensor_lib rclcpp std_msgs sensor_msgs nav_msgs)


# define Drive action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Drive.action"
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(auton_node src/auton_routine.cpp)
target_link_libraries(auton_node sensor_lib ${PIGPIO_LIB} "${cpp_typesupport_target}")
add_dependencies(auton_node ${PROJECT_NAME}__rosidl_generator_cpp)
ament_target_dependencies(auton_node rclcpp rclcpp_action std_msgs)

# Create nodes
# Chassis Node needs to be fixed
# add_executable(chassis_node src/chassis_node.cpp)
# target_link_libraries(chassis_node sensor_lib ${PIGPIO_LIB} "${cpp_typesupport_target}")
# add_dependencies(chassis_node ${PROJECT_NAME}__rosidl_generator_cpp)
# ament_target_dependencies(chassis_node rclcpp rclcpp_action std_msgs)

# IR Node
add_executable(ir_node src/ir_node.cpp) # IR node likely doesn't need odometry/IMU
target_link_libraries(ir_node ${PIGPIO_LIB})
ament_target_dependencies(ir_node rclcpp std_msgs)

# Odometry test node
add_executable(odom_test_node src/odometry_test.cpp)
target_link_libraries(odom_test_node sensor_lib ${PIGPIO_LIB})
ament_target_dependencies(odom_test_node rclcpp std_msgs)

install(TARGETS
  sensor_lib
  auton_node
  #chassis_node
  odom_test_node
  ir_node
  DESTINATION lib/${PROJECT_NAME}
)

# This ensures your headers are copied to the install space
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()