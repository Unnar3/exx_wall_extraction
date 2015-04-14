cmake_minimum_required(VERSION 2.8.3)
project(exx_plane_extraction)

set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -fpermissive")

find_package(catkin REQUIRED COMPONENTS
  	sensor_msgs
	roscpp
	std_msgs
	image_transport
	message_generation
	pcl_conversions
  	pcl_ros
  	exx_compression
  	ransac_primitives
  	metaroom_xml_parser
  	qt_build
)

rosbuild_prepare_qt4(QtCore QtXml)

#add_message_files(FILES
#	distPose.msg
#)

#generate_messages(DEPENDENCIES
#	std_msgs
#)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS message_runtime exx_compression qt_build libqt4-dev
)
include_directories(moment_of_inertia include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(plane_test src/plane_test.cpp)
target_link_libraries(plane_test ${catkin_LIBRARIES} ${QT_LIBRARIES} compression features libvtkCommon.so libvtkFiltering.so libvtkGraphics.so)
#add_executable(plane_compression src/plane_compression.cpp)
#target_link_libraries(plane_compression ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkGraphics.so)