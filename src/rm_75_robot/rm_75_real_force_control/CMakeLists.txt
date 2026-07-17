cmake_minimum_required(VERSION 3.0.2)
project(rm_75_real_force_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_srvs
)

catkin_package()

# This package intentionally uses the vendor C++ API directly instead of the
# Gazebo/MoveIt simulation path. Prefer the vendored minimal SDK in this repo so
# a fresh clone can build; fall back to the original local official package if a
# developer has not checked in / pulled the vendored SDK files.
set(DEFAULT_RM_BASE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../official/rm_base_cmake_example")
if(NOT EXISTS "${DEFAULT_RM_BASE_SDK_DIR}/linux-x86/include/rm_base.h"
   AND EXISTS "/home/ros/机械臂/rm_base_cmake_example/linux-x86/include/rm_base.h")
  set(DEFAULT_RM_BASE_SDK_DIR "/home/ros/机械臂/rm_base_cmake_example")
endif()

set(RM_BASE_SDK_DIR "${DEFAULT_RM_BASE_SDK_DIR}"
    CACHE PATH "Path to the RealMan RM_Base C++ SDK example directory")
set(RM_BASE_INCLUDE_DIR "${RM_BASE_SDK_DIR}/linux-x86/include")
set(RM_BASE_LIBRARY "${RM_BASE_SDK_DIR}/linux-x86/lib/libRM_Base.so.1.0.0")

if(NOT EXISTS "${RM_BASE_INCLUDE_DIR}/rm_base.h")
  message(FATAL_ERROR "Cannot find vendor header: ${RM_BASE_INCLUDE_DIR}/rm_base.h")
endif()

if(NOT EXISTS "${RM_BASE_LIBRARY}")
  message(FATAL_ERROR "Cannot find vendor library: ${RM_BASE_LIBRARY}")
endif()

add_library(rm_base_vendor SHARED IMPORTED)
set_target_properties(rm_base_vendor PROPERTIES
  IMPORTED_LOCATION "${RM_BASE_LIBRARY}"
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RM_BASE_INCLUDE_DIR}
)

add_executable(rm75_force_read_only_node
  src/rm75_force_read_only_node.cpp
)

add_executable(rm75_drag_teach_node
  src/rm75_drag_teach_node.cpp
)

target_link_libraries(rm75_force_read_only_node
  ${catkin_LIBRARIES}
  rm_base_vendor
)

target_link_libraries(rm75_drag_teach_node
  ${catkin_LIBRARIES}
  rm_base_vendor
)

# The vendor binary has SONAME libRM_Base.so.1, while the provided file is
# libRM_Base.so.1.0.0. Create local symlinks next to this node so runtime lookup
# works without modifying the official SDK directory.
add_custom_command(TARGET rm75_force_read_only_node POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:rm75_force_read_only_node>/vendor"
  COMMAND ${CMAKE_COMMAND} -E create_symlink
          "${RM_BASE_LIBRARY}"
          "$<TARGET_FILE_DIR:rm75_force_read_only_node>/vendor/libRM_Base.so.1.0.0"
  COMMAND ${CMAKE_COMMAND} -E create_symlink
          "libRM_Base.so.1.0.0"
          "$<TARGET_FILE_DIR:rm75_force_read_only_node>/vendor/libRM_Base.so.1"
)

add_custom_command(TARGET rm75_drag_teach_node POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:rm75_drag_teach_node>/vendor"
  COMMAND ${CMAKE_COMMAND} -E create_symlink
          "${RM_BASE_LIBRARY}"
          "$<TARGET_FILE_DIR:rm75_drag_teach_node>/vendor/libRM_Base.so.1.0.0"
  COMMAND ${CMAKE_COMMAND} -E create_symlink
          "libRM_Base.so.1.0.0"
          "$<TARGET_FILE_DIR:rm75_drag_teach_node>/vendor/libRM_Base.so.1"
)

# Preserve a local runtime path so rosrun/roslaunch can find the vendor library
# without asking the user to export LD_LIBRARY_PATH.
set_target_properties(rm75_force_read_only_node PROPERTIES
  BUILD_RPATH "\$ORIGIN/vendor"
  INSTALL_RPATH "\$ORIGIN/vendor"
)

set_target_properties(rm75_drag_teach_node PROPERTIES
  BUILD_RPATH "\$ORIGIN/vendor"
  INSTALL_RPATH "\$ORIGIN/vendor"
)
