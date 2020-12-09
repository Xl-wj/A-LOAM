cmake_minimum_required(VERSION 2.8.3)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O2")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -O2")

#PointMatcher
set(pointmatcher_dir /home/xl/projects/ws_aloam/src/A-LOAM/cmake/PointMatcher.cmake)
include(${pointmatcher_dir})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

add_executable(aimageProjection src/imageProjection.cpp)
target_link_libraries(aimageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(ascanRegistration src/scanRegistration.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserOdometry src/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(kittiHelper src/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(transformFusion src/transformFusion.cpp)
target_link_libraries(transformFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(lidarMapLocalization src/lidarMapLocalization.cpp)
target_link_libraries(lidarMapLocalization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${libpointmatcher_LIBRARIES})

