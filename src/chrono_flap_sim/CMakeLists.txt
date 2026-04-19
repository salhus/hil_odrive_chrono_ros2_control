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
# chrono/ChConfig.h (included in the .cpp) defines CHRONO_VSG / CHRONO_IRRLICHT.
# Chrono's CMake config does NOT create imported targets for vis modules.
# We locate the vis libraries manually via find_library().

list(APPEND LIBS Chrono_core)

# Check if Chrono was built with VSG support
if(Chrono_VSG_AVAILABLE)
  find_library(CHRONO_VSG_LIB Chrono_vsg HINTS ${CHRONO_LIBRARY_DIRS} /usr/local/lib)
  if(CHRONO_VSG_LIB)
    message(STATUS "chrono_flap_sim: Found Chrono_vsg at ${CHRONO_VSG_LIB}")
    list(APPEND LIBS ${CHRONO_VSG_LIB})

    # VSG headers (vsg/all.h) are needed by chrono_vsg headers.
    # find_package(vsg) needs CMAKE_PREFIX_PATH to point to the vsg install.
    # ament/colcon may override CMAKE_PREFIX_PATH, so we also scan common paths.
    find_package(vsg QUIET)
    if(NOT vsg_FOUND)
      # Scan CMAKE_PREFIX_PATH manually for vsg cmake config
      foreach(_prefix ${CMAKE_PREFIX_PATH})
        if(EXISTS "${_prefix}/lib/cmake/vsg/vsgConfig.cmake")
          set(vsg_DIR "${_prefix}/lib/cmake/vsg")
          find_package(vsg QUIET)
          break()
        endif()
        if(EXISTS "${_prefix}/include/vsg/all.h")
          set(_VSG_INCLUDE_FALLBACK "${_prefix}/include")
          break()
        endif()
      endforeach()
    endif()

    if(TARGET vsg::vsg)
      message(STATUS "chrono_flap_sim: Found vsg::vsg target — linking for VSG include dirs")
      list(APPEND LIBS vsg::vsg)
    elseif(_VSG_INCLUDE_FALLBACK)
      message(STATUS "chrono_flap_sim: Using fallback VSG include dir: ${_VSG_INCLUDE_FALLBACK}")
      set(VSG_EXTRA_INCLUDES ${_VSG_INCLUDE_FALLBACK})
    else()
      message(WARNING "chrono_flap_sim: Chrono_vsg found but vsg headers not found. "
        "Set CMAKE_PREFIX_PATH to the vsg install prefix, e.g.:
        "  colcon build --cmake-args -DCMAKE_PREFIX_PATH=/path/to/vsg")
    endif()

    # Also try vsgXchange and vsgImGui
    find_package(vsgXchange QUIET)
    if(TARGET vsgXchange::vsgXchange)
      list(APPEND LIBS vsgXchange::vsgXchange)
    endif()
    find_package(vsgImGui QUIET)
    if(TARGET vsgImGui::vsgImGui)
      list(APPEND LIBS vsgImGui::vsgImGui)
    endif()
  else()
    message(STATUS "chrono_flap_sim: Chrono_VSG_AVAILABLE but library not found — running headless")
  endif()
elseif(Chrono_IRRLICHT_AVAILABLE)
  find_library(CHRONO_IRRLICHT_LIB Chrono_irrlicht HINTS ${CHRONO_LIBRARY_DIRS} /usr/local/lib)
  if(CHRONO_IRRLICHT_LIB)
    message(STATUS "chrono_flap_sim: Found Chrono_irrlicht at ${CHRONO_IRRLICHT_LIB}")
    list(APPEND LIBS ${CHRONO_IRRLICHT_LIB})
  endif()
else()
  message(STATUS "chrono_flap_sim: No Chrono visualization module — running headless")
endif()

add_executable(chrono_flap_node src/chrono_flap_node.cpp)

target_include_directories(chrono_flap_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CHRONO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${VSG_EXTRA_INCLUDES}
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