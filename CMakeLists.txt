cmake_minimum_required(VERSION 2.8.3)
project(xm_people)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DGTEST_USE_OWN_TR1_TUPLE=1")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DGTEST_USE_OWN_TR1_TUPLE=1")
endif (CMAKE_SYSTEM_NAME MATCHES "Darwin")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  xm_msgs
  geometry_msgs
  image_transport
  kdl_conversions
  openni2_camera
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  tf
  message_generation
)

#find_package(opencv REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


generate_messages(
    DEPENDENCIES geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES xm_people
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport kdl_conversions openni2_camera pcl_conversions pcl_ros roscpp roslib tf message_runtime
  DEPENDS system_lib opencv
)

set(OPENNI2_WRAPPER /opt/ros/indigo/include/openni2_camera/)

find_path(OpenNI2_INCLUDEDIR
	  NAMES OpenNI.h
	  HINTS /usr/include/openni2)
find_library(OpenNI2_LIBRARIES
	     NAMES OpenNI2 DummyDevice OniFile PS1090
	     HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
	     PATH_SUFFIXES lib) 
message(STATUS ${OpenNI2_LIBRARIES})

	#manual nite libraries
	SET ( NITE2_INC $ENV{NITE2_INCLUDE})
	SET ( NITE2_LIB $ENV{NITE2_REDIST64})

	if (NITE2_INC AND NITE2_LIB)
		message(STATUS "NITE found at ${NITE2_LIB}")
		set_property(
	  		SOURCE openniServer.cpp
	   	PROPERTY COMPILE_DEFINITIONS WITH_NITE2=1
	   	)
	else()
		message("NITE2 library not found")
	endif()
	include_directories(${NITE2_INC}  ${NITE2_LIB})

include_directories(${OpenNI2_INCLUDEDIR})
include_directories(${OPENNI2_WRAPPER})
   #PCL  
    find_package(PCL 1.8 REQUIRED)  
    include_directories(${PCL_INCLUDE_DIRS})  
    link_directories(${PCL_LIBRARY_DIRS})  
    add_definitions(${PCL_DEFINITIONS})  

add_executable(xtion_tracker src/xtion_tracker_node.cpp)
target_link_libraries(xtion_tracker OpenNI2 ${OpenNI2_LIBRARIES} ${NITE2_LIB}/libNiTE2.so ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}      ${catkin_LIBRARIES})

