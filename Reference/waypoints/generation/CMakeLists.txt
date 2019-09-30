set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8)
project( ceilingCam )
find_package( OpenCV REQUIRED )
message(STATUS "OpenCV library status:")
message(STATUS "	version: ${OpenCV_VERSION}")
message(STATUS "	libraries: ${OpenCV_LIBS}")
message(STATUS "	include path: ${OpenCV_INCLUDE_DIRS}")
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(
	PathFindingTest
	Main.cpp
	PixelState.h
	State.h
	SocketConnection.h
	SocketConnection.cpp
	System.h
	TransitionRegion.h
	TransitionPoint.h
	TransitionRegion.cpp
	RobotDetection.cpp
	RobotDetection.h
	PixelState.cpp
	CeilingObjectDetection.cpp
	CeilingObjectDetection.h
	ImageProcessing.cpp
	ImageProcessing.h
	Planner.cpp
	Planner.h
	HashTable.cpp
	HashTable.h
)
target_link_libraries( PathFindingTest ${OpenCV_LIBS} )
