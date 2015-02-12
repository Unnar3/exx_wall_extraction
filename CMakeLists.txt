set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(exx_plane_extraction)

find_package(catkin REQUIRED COMPONENTS
  	sensor_msgs
	cv_bridge
	roscpp
	std_msgs
	image_transport
	message_generation
	pcl_conversions
  	pcl_ros
)

#add_message_files(FILES
#	distPose.msg
#)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS message_runtime s8_common_node s8_msgs
)
include_directories(${catkin_INCLUDE_DIRS})


add_executable(plane_extraction src/plane_extraction.cpp)
target_link_libraries(plane_extraction ${catkin_LIBRARIES})