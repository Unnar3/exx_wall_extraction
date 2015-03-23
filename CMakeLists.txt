cmake_minimum_required(VERSION 2.8.3)
project(exx_plane_extraction)

find_package(catkin REQUIRED COMPONENTS
  	sensor_msgs
	roscpp
	std_msgs
	image_transport
	message_generation
	exx_common_node
	pcl_conversions
  	pcl_ros
)

#add_message_files(FILES
#	distPose.msg
#)

#generate_messages(DEPENDENCIES
#	std_msgs
#)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS message_runtime exx_common_node
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(plane_test src/plane_test.cpp include/compression.cpp)
target_link_libraries(plane_test ${catkin_LIBRARIES})
add_executable(plane_compression src/plane_compression.cpp)
target_link_libraries(plane_compression ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkGraphics.so)