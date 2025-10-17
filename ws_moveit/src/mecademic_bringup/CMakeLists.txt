cmake_minimum_required(VERSION 3.8)
project(mecademic_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Services generieren ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTool.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_python_install_package(${PROJECT_NAME})

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
