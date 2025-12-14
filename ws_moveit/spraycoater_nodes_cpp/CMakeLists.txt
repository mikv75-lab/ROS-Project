cmake_minimum_required(VERSION 3.16)
project(spraycoater_nodes_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(servo src/servo.cpp)

target_include_directories(servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Liefert include+link Flags (intern plain-signature)
ament_target_dependencies(servo
  rclcpp
  std_msgs
  geometry_msgs
  control_msgs
)

# Zusätzliche Libs mit **plain**-Signatur (kein PUBLIC/PRIVATE!)
target_link_libraries(servo
  yaml-cpp
)

install(TARGETS servo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs geometry_msgs control_msgs yaml-cpp)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
