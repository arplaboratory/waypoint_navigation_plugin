## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds the
## TeleopPanel, ImuDisplay, and PlantFlagTool tutorials.

cmake_minimum_required(VERSION 3.8)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(mav_manager_srv REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

set(dependencies
  rclcpp  
  Qt5
  rviz2
  rviz_common
  rosbag2_cpp
  rviz_default_plugins
  rviz_rendering
  rviz_ogre_vendor
  rosbag2
  pluginlib
  std_srvs
  mav_manager_srv
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/waypoint_nav_frame_ros2.cpp
  src/waypoint_nav_tool_ros2.cpp
)

set(${PROJECT_NAME}_HDRS
    src/waypoint_nav_frame_ros2.hpp
    src/waypoint_nav_tool_ros2.hpp
    src/gnuplot.h
)

set(${PROJECT_NAME}_UIS
    ui/WaypointNavigation.ui
)

message(STATUS "Generate header for ui with rviz2_QT_VERSION: ${rviz2_QT_VERSION}")
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
foreach(header "${${PROJECT_NAME}_HDRS}")
  qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${header})
endforeach()
## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SRC_FILES}``. We also add the needed dependencies.
add_library(${PROJECT_NAME} SHARED ${SRC_FILES} 
${${PROJECT_NAME}_UIS_H}
${${PROJECT_NAME}_MOCS}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  )

## Here we export the plugins and meshes so they can be found by RViz at runtime.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
register_rviz_ogre_media_exports(DIRECTORIES "media")

## END_TUTORIAL

ament_export_dependencies(
  geometry_msgs
  rclcpp
  rviz_common
  rviz_rendering
  sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons)

ament_package()
