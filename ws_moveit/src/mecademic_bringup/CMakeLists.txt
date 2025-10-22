cmake_minimum_required(VERSION 3.8)
project(mecademic_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Services generieren ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTool.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_python_install_package(${PROJECT_NAME})

# Füge den C++-Node hinzu
add_executable(servo_jog_node src/mecademic_bringup/mecademic_bringup/servo_jog_node.cpp)

ament_target_dependencies(servo_jog_node
  rclcpp
  moveit_servo
  geometry_msgs
  trajectory_msgs
)

install(TARGETS
  servo_jog_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
