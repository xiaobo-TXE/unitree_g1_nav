cmake_minimum_required(VERSION 3.16)
project(basic VERSION 2.0 LANGUAGES CXX C)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

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

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)


file(GLOB_RECURSE BASIC_SOURCES "src/*.cpp")
file(GLOB_RECURSE BASIC_HEADERS "include/*.h" "include/*.hpp")

add_library(${PROJECT_NAME} SHARED
  ${BASIC_SOURCES}
  ${BASIC_HEADERS}
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  gflags
  ${GLOG_TARGET}
  pthread
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 PCL)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()