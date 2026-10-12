cmake_minimum_required(VERSION 3.14)
project(rviz_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(OpenGL REQUIRED)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  src/goal_3d_tool.cpp
  src/pose_3d_tool.cpp
  include/rviz_tools/goal_3d_tool.hpp
  include/rviz_tools/pose_3d_tool.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  pluginlib
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  OpenGL::GL
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
