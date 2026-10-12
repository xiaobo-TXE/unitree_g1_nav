cmake_minimum_required(VERSION 3.16)
project(super_lio VERSION 2.0 LANGUAGES CXX C)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE STRING "Suppress developer warnings" FORCE)

# C++17, not 20, and this is a deployment constraint rather than a preference.
# The board carries Eigen 3.3.x in /usr/local/include/eigen3, and Eigen before
# 3.4 does not survive C++20's rewritten comparison operators: the compiler
# rewrites `expr == Scalar(0)` into the reversed form, finds Eigen's
# operator== returning an array expression instead of bool, and LDLT.h fails
# to compile (drift_estimator.cpp, registrar.cpp). The dev machine hides this
# because Ubuntu 22.04 ships Eigen 3.4.0. Nothing in this codebase uses a
# C++20 feature, and ROS 2 itself builds with 17, so 17 is the portable floor.
# Do not raise this without checking the Eigen version on the robot.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread -fexceptions -flto=auto")
set(CMAKE_C_FLAGS_RELEASE "-O3 -Wall -fPIC")
add_definitions(-DROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
# Optional: only robots with a Livox need it. The CustomMsg branch is the one
# thing in this package that cannot be built without the driver's interfaces,
# so it is compiled out instead of failing the whole build on a Velodyne /
# Ouster / Hesai rig. package.xml still lists it as a <depend> so that colcon
# orders the build correctly when the driver does live in the same workspace.
find_package(livox_ros_driver2 QUIET)
if(livox_ros_driver2_FOUND)
  message(STATUS "livox_ros_driver2 found: CustomMsg (lidar_type=LIVOX) enabled")
else()
  message(WARNING "livox_ros_driver2 not found: lidar_type=LIVOX will be unavailable. "
                  "Source the Livox driver workspace and rebuild if this rig has a Livox.")
endif()
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
# ==================== 针对 Ubuntu 20.04 的 glog 修复 ====================
# 不用 find_package(Glog)：20.04 的 libgoogle-glog-dev 未提供可用的 CMake config。
# 按当前架构的 multiarch 目录查找，避免写死 aarch64/x86_64 路径。
set(Glog_INCLUDE_DIRS /usr/include)
find_library(GLOG_TARGET
  NAMES glog
  PATHS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE} /usr/lib /usr/local/lib
)
if(NOT GLOG_TARGET)
  message(FATAL_ERROR "libglog not found. Install it with: sudo apt install libgoogle-glog-dev")
endif()
# =======================================================================

find_package(basic REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CloudPose.msg
  msg/CloudPose2.msg
  msg/LocStatus.msg
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# rosidl_get_typesupport_target is only available from Galactic onward.
# On Foxy, use rosidl_target_interfaces instead (applied later per target).

include_directories(
  include
  3rdparty
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
)

file(GLOB_RECURSE LIO_LIB_SRC "src/lio/*.cpp" "src/ros/*.cpp" "src/loc/*.cpp")
file(GLOB_RECURSE LIO_LIB_HEADERS "include/lio/*.hpp" "include/lio/*.h"
                                  "include/loc/*.hpp" "include/loc/*.h")

add_library(lio ${LIO_LIB_SRC} ${LIO_LIB_HEADERS})

ament_target_dependencies(lio PUBLIC
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_ros
  tf2_ros
  basic
)

if(livox_ros_driver2_FOUND)
  ament_target_dependencies(lio PUBLIC livox_ros_driver2)
  # PUBLIC: ROSWrapper.h guards its CustomMsg members on this, and every node
  # target includes that header.
  target_compile_definitions(lio PUBLIC HAVE_LIVOX_ROS_DRIVER2)
endif()

add_dependencies(lio ${PROJECT_NAME}__rosidl_typesupport_cpp)

target_link_libraries(lio PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  TBB::tbb
  ${GLOG_TARGET}
  pthread
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

add_executable(super_lio_node src/apps/super_lio_node.cpp)
add_dependencies(super_lio_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_link_libraries(super_lio_node PRIVATE lio)

add_executable(relocation_node src/apps/relocation_node.cpp)
add_dependencies(relocation_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_link_libraries(relocation_node PRIVATE lio)

add_executable(map_localization_node src/apps/map_localization_node.cpp)
add_dependencies(map_localization_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_link_libraries(map_localization_node PRIVATE lio)

# Offline map builder: PCL + Eigen only, deliberately no ROS and no `lio`, so it
# can be run against a session without a sourced workspace.
add_executable(loc_map_builder src/apps/loc_map_builder.cpp)
target_link_libraries(loc_map_builder PRIVATE
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

install(TARGETS
  lio
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  super_lio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  relocation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  map_localization_node
  loc_map_builder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  3rdparty/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Unit tests for the localization modules. They link `lio` but touch none of
# the ROS entry points: every loc module takes its inputs by value and returns
# its results, so no node or bag is needed.
# The test/ tree is not tracked in this repository (see .gitignore), so every
# target below is guarded on its source actually being present. Without the
# guards, a fresh clone configured with -DBUILD_TESTING=ON fails at CMake time
# on a missing file rather than simply building nothing.
if(BUILD_TESTING)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_loc.cpp)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_loc test/test_loc.cpp)
    if(TARGET test_loc)
      add_dependencies(test_loc ${PROJECT_NAME}__rosidl_typesupport_cpp)
      target_link_libraries(test_loc lio)
    endif()
  endif()

  # Manual test drivers, not gtest: they need the real map and the real session
  # PCDs, so they are run by hand rather than by `colcon test`.
  # See docs/maploc_manual.md section 8 for how to drive them.
  foreach(t loc_replay loc_probe_reg init_probe)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/tools/${t}.cpp)
      add_executable(${t} test/tools/${t}.cpp)
      add_dependencies(${t} ${PROJECT_NAME}__rosidl_typesupport_cpp)
      target_link_libraries(${t} PRIVATE lio)
      install(TARGETS ${t} DESTINATION lib/${PROJECT_NAME})
    endif()
  endforeach()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()