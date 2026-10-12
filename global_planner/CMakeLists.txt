cmake_minimum_required(VERSION 3.5)
project(global_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_library(global_planner_core
  src/terrain_map.cpp
  src/weighted_astar.cpp
)
target_include_directories(global_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(global_planner_core ${PCL_LIBRARIES})
target_compile_definitions(global_planner_core PRIVATE ${PCL_DEFINITIONS})

add_executable(global_planner_node
  src/global_planner_node.cpp
  src/main.cpp
)
target_include_directories(global_planner_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(global_planner_node global_planner_core ${PCL_LIBRARIES})
ament_target_dependencies(global_planner_node
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions
)

install(TARGETS global_planner_core global_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_global_planner test/test_global_planner.cpp)
  if(TARGET test_global_planner)
    target_include_directories(test_global_planner PRIVATE include ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_global_planner global_planner_core ${PCL_LIBRARIES})
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(global_planner_core)
ament_package()
