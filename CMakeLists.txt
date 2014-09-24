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

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros rtt_roscomm rtt_rosclock rtt_rosdeployment rtt_rosnode rtt_ros_msgs std_msgs
)

install(FILES
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#orocos_component(COMPONENT src/COMPONENT.cpp)
#target_link_libraries(COMPONENT ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})