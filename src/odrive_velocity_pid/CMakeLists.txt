cmake_minimum_required(VERSION 3.8)
project(odrive_velocity_pid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(velocity_pid_node src/velocity_pid_node.cpp)
target_include_directories(velocity_pid_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(velocity_pid_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS velocity_pid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export the include directory so other packages can use pid_controller.hpp
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

ament_package()
