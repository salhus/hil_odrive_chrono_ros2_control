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
find_package(Eigen3 REQUIRED)
find_package(Chrono REQUIRED)

# ── Collect libraries ──────────────────────────────────────────────────────────
# chrono/ChConfig.h (included in the .cpp) defines CHRONO_VSG / CHRONO_IRRLICHT
# based on how Chrono was built.
#
# Chrono's CMake config does NOT create imported targets (Chrono_vsg etc.).
# It populates CHRONO_LIB_NAMES when components are requested.  Since we use
# a bare find_package(Chrono REQUIRED), we locate the vis libraries manually.

list(APPEND LIBS Chrono_core)

# Check if Chrono was built with VSG support
if(Chrono_VSG_AVAILABLE)
  find_library(CHRONO_VSG_LIB Chrono_vsg HINTS ${CHRONO_LIBRARY_DIRS} /usr/local/lib)
  if(CHRONO_VSG_LIB)
    message(STATUS "chrono_flap_sim: Found Chrono_vsg at ${CHRONO_VSG_LIB}")
    list(APPEND LIBS ${CHRONO_VSG_LIB})
  else()
    message(STATUS "chrono_flap_sim: Chrono_VSG_AVAILABLE but library not found — running headless")
  endif()
elseif(Chrono_IRRLICHT_AVAILABLE)
  find_library(CHRONO_IRRLICHT_LIB Chrono_irrlicht HINTS ${CHRONO_LIBRARY_DIRS} /usr/local/lib)
  if(CHRONO_IRRLICHT_LIB)
    message(STATUS "chrono_flap_sim: Found Chrono_irrlicht at ${CHRONO_IRRLICHT_LIB}")
    list(APPEND LIBS ${CHRONO_IRRLICHT_LIB})
  else()
    message(STATUS "chrono_flap_sim: Chrono_IRRLICHT_AVAILABLE but library not found — running headless")
  endif()
else()
  message(STATUS "chrono_flap_sim: No Chrono visualization module — running headless")
endif()

add_executable(chrono_flap_node src/chrono_flap_node.cpp)

target_include_directories(chrono_flap_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CHRONO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(chrono_flap_node ${LIBS})

ament_target_dependencies(chrono_flap_node
  rclcpp
  std_msgs
)

install(TARGETS chrono_flap_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()