cmake_minimum_required(VERSION 3.14)
project(chrono_flap_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Chrono was built with C++20; must match to avoid ABI mismatches.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Workaround: GCC 13 has a broken std::format implementation that causes
# compile errors when <format> is transitively included in C++20 mode.
# Setting __cpp_lib_format=0 prevents the broken codepaths from being
# instantiated. Neither Chrono nor this node uses std::format directly.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "14")
  add_compile_definitions(__cpp_lib_format=0)
  message(STATUS "chrono_flap_sim: GCC <14 detected, disabling std::format to work around bug")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Chrono REQUIRED)

# Collect libraries
list(APPEND LIBS Chrono_core)

set(CHRONO_VIS_BACKEND "auto" CACHE STRING
  "Visualization backend: auto (VSG only), vsg, none. \
Use irrlicht only if you have a C++17-compatible build; Irrlicht 1.8 is incompatible with C++20.")
string(TOLOWER "${CHRONO_VIS_BACKEND}" _vis_backend)
message(STATUS "chrono_flap_sim: CHRONO_VIS_BACKEND = ${_vis_backend}")

set(_use_vsg FALSE)
set(_use_irrlicht FALSE)

if(_vis_backend STREQUAL "none")
  message(STATUS "chrono_flap_sim: Visualization disabled (CHRONO_VIS_BACKEND=none) - running headless")
elseif(_vis_backend STREQUAL "vsg" OR (_vis_backend STREQUAL "auto" AND Chrono_VSG_AVAILABLE))
  if(Chrono_VSG_AVAILABLE)
    find_library(CHRONO_VSG_LIB Chrono_vsg HINTS /usr/local/lib)
    if(CHRONO_VSG_LIB)
      message(STATUS "chrono_flap_sim: Found Chrono_vsg at ${CHRONO_VSG_LIB}")
      list(APPEND LIBS ${CHRONO_VSG_LIB})
      set(_use_vsg TRUE)

      # Find VSG include dir from CMAKE_PREFIX_PATH.
      # Do NOT link vsg::vsg imported target - it may re-enable format definitions.
      set(_VSG_INCLUDE_DIR "")
      foreach(_prefix ${CMAKE_PREFIX_PATH})
        if(EXISTS "${_prefix}/include/vsg/all.h")
          set(_VSG_INCLUDE_DIR "${_prefix}/include")
          break()
        endif()
      endforeach()

      if(_VSG_INCLUDE_DIR)
        message(STATUS "chrono_flap_sim: Found VSG headers at ${_VSG_INCLUDE_DIR}")
        set(VSG_EXTRA_INCLUDES ${_VSG_INCLUDE_DIR})
      else()
        message(WARNING "chrono_flap_sim: Chrono_vsg found but vsg/all.h not found. Set CMAKE_PREFIX_PATH to the vsg install prefix.")
      endif()

      find_library(_VSGXCHANGE_LIB vsgXchange HINTS /usr/local/lib)
      if(_VSGXCHANGE_LIB)
        list(APPEND LIBS ${_VSGXCHANGE_LIB})
      endif()
    else()
      message(STATUS "chrono_flap_sim: Chrono_VSG_AVAILABLE but Chrono_vsg library not found - running headless")
    endif()
  else()
    message(WARNING "chrono_flap_sim: CHRONO_VIS_BACKEND=vsg requested but Chrono_VSG_AVAILABLE is false - running headless")
  endif()
elseif(_vis_backend STREQUAL "irrlicht")
  # NOTE: Irrlicht 1.8 is incompatible with C++20. Only use this if you have a C++17 build.
  if(Chrono_IRRLICHT_AVAILABLE)
    find_library(CHRONO_IRRLICHT_LIB Chrono_irrlicht HINTS /usr/local/lib)
    if(CHRONO_IRRLICHT_LIB)
      message(STATUS "chrono_flap_sim: Found Chrono_irrlicht at ${CHRONO_IRRLICHT_LIB}")
      list(APPEND LIBS ${CHRONO_IRRLICHT_LIB})
      set(_use_irrlicht TRUE)
    endif()
  else()
    message(WARNING "chrono_flap_sim: CHRONO_VIS_BACKEND=irrlicht requested but Chrono_IRRLICHT_AVAILABLE is false - running headless")
  endif()
else()
  if(NOT _vis_backend STREQUAL "auto")
    message(WARNING "chrono_flap_sim: Unrecognized CHRONO_VIS_BACKEND='${_vis_backend}'. Valid values: auto, vsg, irrlicht, none. Falling back to headless.")
  endif()
  message(STATUS "chrono_flap_sim: No visualization module - running headless")
endif()

add_executable(chrono_flap_node src/chrono_flap_node.cpp)

# Use our own compile-time flags so visualization guards are decoupled from
# whatever CHRONO_VSG / CHRONO_IRRLICHT Chrono's cmake config injects.
if(_use_vsg)
  target_compile_definitions(chrono_flap_node PRIVATE CHRONO_FLAP_USE_VSG)
  message(STATUS "chrono_flap_sim: Compiling with CHRONO_FLAP_USE_VSG")
endif()
if(_use_irrlicht)
  target_compile_definitions(chrono_flap_node PRIVATE CHRONO_FLAP_USE_IRRLICHT)
  message(STATUS "chrono_flap_sim: Compiling with CHRONO_FLAP_USE_IRRLICHT")
endif()

target_include_directories(chrono_flap_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CHRONO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${VSG_EXTRA_INCLUDES}
)

target_link_libraries(chrono_flap_node ${LIBS})

ament_target_dependencies(chrono_flap_node
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS chrono_flap_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()