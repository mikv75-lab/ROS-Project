cmake_minimum_required(VERSION 3.8)
project(mecademic_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Für C++ ROS2 Unterstützung
find_package(moveit_servo REQUIRED)  # Abhängigkeit für moveit_servo
find_package(geometry_msgs REQUIRED)  # Falls du mit Twist-Nachrichten arbeitest
find_package(trajectory_msgs REQUIRED)  # Für JointTrajectory-Nachrichten

# --- Services generieren ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTool.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_python_install_package(${PROJECT_NAME})

# Füge den C++-Node hinzu
add_executable(servo_jog_node src/servo_jog_node.cpp)

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

# Installiere Python-Skripte
install(PROGRAMS
  mecademic_bringup/tool_manager.py
  mecademic_bringup/poses_manager.py
  mecademic_bringup/workspace_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
