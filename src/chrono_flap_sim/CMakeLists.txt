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

add_executable(chrono_flap_node src/chrono_flap_node.cpp)

target_include_directories(chrono_flap_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CHRONO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(chrono_flap_node
  Chrono_core
)

ament_target_dependencies(chrono_flap_node
  rclcpp
  std_msgs
)

# ── Optional 3D visualization ──────────────────────────────────────────────────────────────────
# chrono/ChConfig.h (included by chrono_flap_node.cpp) is the authoritative source for
# CHRONO_VSG / CHRONO_IRRLICHT macros — do NOT set them via target_compile_definitions.
# We only need to link the visualization libraries and, for VSG, pull in vsg include paths.
if(TARGET Chrono_vsg)
  message(STATUS "chrono_flap_sim: Chrono VSG found — enabling VSG visualization")

  # VSG may be installed at a non-standard prefix (e.g. ~/Packages/vsg/).
  # Extract interface include dirs from Chrono_vsg to derive search-path hints
  # so that find_package(vsg) can locate the vsg cmake config files without
  # modifying the global CMAKE_PREFIX_PATH.
  get_target_property(_chrono_vsg_incdirs Chrono_vsg INTERFACE_INCLUDE_DIRECTORIES)
  set(_vsg_prefix_hints)
  if(_chrono_vsg_incdirs)
    foreach(_dir ${_chrono_vsg_incdirs})
      # Standard convention: include dir is <prefix>/include — one level up is <prefix>
      get_filename_component(_prefix "${_dir}/.." ABSOLUTE)
      list(APPEND _vsg_prefix_hints "${_prefix}")
    endforeach()
  endif()

  # Try to find the vsg:: imported targets with their include dirs.
  # Pass the derived hints via PATHS so the global CMAKE_PREFIX_PATH is not polluted.
  find_package(vsg QUIET PATHS ${_vsg_prefix_hints})
  find_package(vsgXchange QUIET PATHS ${_vsg_prefix_hints})
  find_package(vsgImGui QUIET PATHS ${_vsg_prefix_hints})

  target_link_libraries(chrono_flap_node Chrono_vsg)
  if(TARGET vsg::vsg)
    target_link_libraries(chrono_flap_node vsg::vsg)
  elseif(_chrono_vsg_incdirs)
    # find_package(vsg) failed (e.g. no cmake config in the hinted prefix) —
    # fall back to the include dirs already advertised by the Chrono_vsg target.
    message(STATUS "chrono_flap_sim: vsg::vsg target not found; using Chrono_vsg include dirs as fallback")
    target_include_directories(chrono_flap_node PRIVATE ${_chrono_vsg_incdirs})
  endif()
  if(TARGET vsgXchange::vsgXchange)
    target_link_libraries(chrono_flap_node vsgXchange::vsgXchange)
  endif()
  if(TARGET vsgImGui::vsgImGui)
    target_link_libraries(chrono_flap_node vsgImGui::vsgImGui)
  endif()
elseif(TARGET Chrono_irrlicht)
  message(STATUS "chrono_flap_sim: Chrono Irrlicht found — enabling Irrlicht visualization")
  target_link_libraries(chrono_flap_node Chrono_irrlicht)
else()
  message(STATUS "chrono_flap_sim: No Chrono visualization module found — running headless")
endif()

install(TARGETS chrono_flap_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
