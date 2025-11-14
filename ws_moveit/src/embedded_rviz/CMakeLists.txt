cmake_minimum_required(VERSION 3.16)
project(embedded_rviz VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

# Wichtig: Python3 vor pybind11 (liefert python3_add_library)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

# PyBind11: Vendor wenn verfügbar, sonst System
find_package(pybind11_vendor QUIET)
if(pybind11_vendor_FOUND)
  find_package(pybind11 CONFIG REQUIRED)
else()
  find_package(pybind11 REQUIRED)
endif()

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Host-Bibliothek ----
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

# ---- PyBind11-Modul ----
pybind11_add_module(embedded_rviz_py MODULE
  src/pybind.cpp
)
target_link_libraries(embedded_rviz_py PRIVATE
  embedded_rviz_host
  Qt5::Widgets
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
)
install(TARGETS embedded_rviz_py
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# ---- Install/Export ----
install(TARGETS embedded_rviz_host
  EXPORT embedded_rviz_hostTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# PyQt-Starter (ausführbar)
install(PROGRAMS
  scripts/pyqt_embed_rviz
  scripts/pyqt_embed_rviz_ui
  DESTINATION lib/${PROJECT_NAME}
)

# Ressourcen (UI)
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME})


ament_export_targets(embedded_rviz_hostTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rviz_common rviz_rendering)
ament_export_include_directories(include)

ament_package()

