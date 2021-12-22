cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_path)
add_definitions(-Wall -std=c++11 -Werror=return-type ${PCL_DEFINITIONS})
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
#set(CMAKE_BUILD_TYPE Release)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs visualization_msgs)
find_package(Eigen REQUIRED)


generate_messages(
  DEPENDENCIES geometry_msgs std_msgs   visualization_msgs)

catkin_package(CATKIN_DEPENDS roscpp   std_msgs
  geometry_msgs
  visualization_msgs)

include_directories(
${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
#  LIBRARIES imu_3dm_gx3
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(quadrotor_path  src/quadrotor_path.cpp)
target_link_libraries(quadrotor_path ${catkin_LIBRARIES})

# install
#set_target_properties(MVSampleVISLAMEagle PROPERTIES COMPILE_FLAGS "-std=c++11" LINK_FLAGS "--sysroot=${CMAKE_SYSROOT} "INSTALL_RPATH ${PROJECT_INSTALL_DIRECTORY})
#install(TARGETS ${MVSAMPLEVISLAMEAGLE_TARGET_NAME} DESTINATION ${PROJECT_INSTALL_DIRECTORY})
