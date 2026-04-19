cmake_minimum_required(VERSION 3.14)
project(chrono_flap_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ── Project Chrono ─────────────────────────────────────────────────────────────────────────────
# Adjust Chrono_DIR if ChronoConfig.cmake is not on the default CMake search path, e.g.:
#   cmake -DChronoConfig_DIR=/path/to/chrono/lib/cmake/Chrono ..
find_package(Chrono REQUIRED)

# ── Eigen3 ─────────────────────────────────────────────────────────────────────────────────────
# Chrono headers internally include "Eigen/Dense". On systems where Eigen is installed under
# an eigen3/ prefix (e.g. /usr/include/eigen3/Eigen/Dense), CHRONO_INCLUDE_DIRS alone is not
# sufficient; we must add the Eigen3 include directory explicitly.
find_package(Eigen3 REQUIRED)

add_executable(chrono_flap_node src/chrono_flap_node.cpp)

target_compile_definitions(chrono_flap_node PRIVATE ${CHRONO_CXX_FLAGS_LIST})

target_include_directories(chrono_flap_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CHRONO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(chrono_flap_node
  ${CHRONO_LIBRARIES}
)

ament_target_dependencies(chrono_flap_node
  rclcpp
  std_msgs
)

install(TARGETS chrono_flap_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
