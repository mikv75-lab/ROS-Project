# embedded_rviz/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)  # 3.16+ ist auf Rolling safe
project(embedded_rviz LANGUAGES CXX)

# ament / ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

# Qt (Rolling RViz nutzt weiterhin Qt5)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Für ament-Extras (IN_LIST in if()) – vermeidet deine Policy-Warnung
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Bibliothek mit unserem Host
add_library(embedded_rviz_host SHARED
  src/host.cpp
)

target_include_directories(embedded_rviz_host
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(embedded_rviz_host
  rclcpp
  rviz_common
  rviz_rendering
)

target_link_libraries(embedded_rviz_host
  Qt5::Widgets
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
)

# Export für Downstream-Pakete (vermeidet "unknown export" Fehler)
install(TARGETS embedded_rviz_host
  EXPORT embedded_rviz_hostTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# ament-Exports
ament_export_targets(embedded_rviz_hostTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rviz_common rviz_rendering)

ament_package()
