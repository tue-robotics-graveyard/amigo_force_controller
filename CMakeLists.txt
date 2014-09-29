cmake_minimum_required(VERSION 2.8.3)
project(amigo_force_controller)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_roscomm
  rtt_rosclock
  rtt_rosdeployment
  rtt_rosnode
  rtt_ros_msgs
  std_msgs
  )

include_directories(
	include
	${catkin_INCLUDE_DIRS} 
	${USE_OROCOS_INCLUDE_DIRS})

orocos_install_headers(DIRECTORY 
	include)

# Get all the headers
file(GLOB_RECURSE HEADER_FILES include/*.hpp)

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros rtt_roscomm rtt_rosclock rtt_rosdeployment rtt_rosnode rtt_ros_msgs std_msgs
)

orocos_component(CurrentFFW src/CurrentFFW.cpp ${HEADER_FILES})
target_link_libraries(CurrentFFW ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
