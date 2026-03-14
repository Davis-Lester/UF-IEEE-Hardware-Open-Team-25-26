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
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_library(PIGPIO_LIB pigpio REQUIRED)
find_library(I2C_LIB i2c REQUIRED)

# define Drive action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Drive.action"
  "action/FindColor.action"
  DEPENDENCIES action_msgs
)

# Create library for sensors
add_library(sensor_lib SHARED 
    src/sensors/MPU6050.cpp
    src/odometry.cpp
    src/navigation_controller.cpp
    src/tank_odometry.cpp
    src/encoder_driver.cpp
    src/sensors/pca9685_driver.cpp
    src/camera_lift_node.cpp
)
target_include_directories(sensor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sensor_lib rclcpp rclcpp_action std_msgs sensor_msgs nav_msgs geometry_msgs)
add_dependencies(sensor_lib ${PROJECT_NAME}__rosidl_generator_cpp)
target_link_libraries(sensor_lib ${I2C_LIB} ${PIGPIO_LIB})

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
add_dependencies(sensor_lib ${cpp_typesupport_target})

# Auton Node
add_executable(auton_node src/auton_routine.cpp)
target_link_libraries(auton_node sensor_lib "${cpp_typesupport_target}")
add_dependencies(auton_node ${PROJECT_NAME}__rosidl_generator_cpp)
ament_target_dependencies(auton_node rclcpp rclcpp_action std_msgs geometry_msgs)

#Chassis Node needs to be fixed
add_executable(chassis_node src/chassis_node.cpp)
target_link_libraries(chassis_node sensor_lib ${PIGPIO_LIB} "${cpp_typesupport_target}")
add_dependencies(chassis_node ${PROJECT_NAME}__rosidl_generator_cpp)
ament_target_dependencies(chassis_node rclcpp rclcpp_action std_msgs geometry_msgs)

#Camera Node
add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node sensor_lib ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
add_dependencies(camera_node ${PROJECT_NAME}__rosidl_generator_cpp)
ament_target_dependencies(camera_node rclcpp rclcpp_action std_msgs cv_bridge sensor_msgs geometry_msgs)


# Intake Control Node (no pigpio or PCA9685 dependencies)
add_executable(intake_node src/intake_node.cpp)
target_link_libraries(intake_node sensor_lib "${cpp_typesupport_target}")
add_dependencies(intake_node ${PROJECT_NAME}__rosidl_generator_cpp)
ament_target_dependencies(intake_node rclcpp std_msgs)

# Keypad Servo node
# add_executable(keypad_servo_node src/keypad_servo_node.cpp)
# ament_target_dependencies(keypad_servo_node rclcpp std_msgs)
# target_link_libraries(keypad_servo_node sensor_lib ${PIGPIO_LIB})

install(TARGETS
  auton_node
  chassis_node
  camera_node
  # ir_node
  # keypad_servo_node
  intake_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS sensor_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# This ensures your headers are copied to the install space
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()