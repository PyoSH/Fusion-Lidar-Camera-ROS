cmake_minimum_required(VERSION 3.0.2)
project(fusion_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  image_transport
  cv_bridge
  message_generation
  geometry_msgs
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  msgVelodyne.msg
  ParkPoint.msg
PointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES velodyne_filter
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/home/kroad/catkin_ws/src/stopline_2021")


add_executable(fusion_LiDAR src/fusion_LiDAR.cpp)
add_dependencies(fusion_LiDAR ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fusion_LiDAR ${catkin_LIBRARIES})

#add_executable(fusion_cam src/fusion_cam.cpp)
#target_link_libraries(fusion_cam ${catkin_LIBRARIES})
#target_link_libraries(fusion_cam ${OpenCV_LIBS})