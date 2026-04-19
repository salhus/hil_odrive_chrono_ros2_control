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

# ── Collect visualization libraries ────────────────────────────────────────────
# chrono/ChConfig.h (included in the .cpp) defines CHRONO_VSG / CHRONO_IRRLICHT
# based on how Chrono was built.  We mirror the pattern from Chrono's own demo
# CMakeLists: append Chrono_vsg or Chrono_irrlicht to the LIBS list.
list(APPEND LIBS Chrono_core)

if(CH_ENABLE_MODULE_VSG OR TARGET Chrono_vsg)
  message(STATUS "chrono_flap_sim: Chrono VSG module available — enabling VSG visualization")
  list(APPEND LIBS "Chrono_vsg")
elseif(CH_ENABLE_MODULE_IRRLICHT OR TARGET Chrono_irrlicht)
  message(STATUS "chrono_flap_sim: Chrono Irrlicht module available — enabling Irrlicht visualization")
  list(APPEND LIBS "Chrono_irrlicht")
else()
  message(STATUS "chrono_flap_sim: No Chrono visualization module found — running headless")
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