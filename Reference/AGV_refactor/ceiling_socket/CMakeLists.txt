set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 3.1)
project( DISCMAHN )
find_package( OpenCV REQUIRED )
message(STATUS "OpenCV library status:")
message(STATUS "	version: ${OpenCV_VERSION}")
message(STATUS "	libraries: ${OpenCV_LIBS}")
message(STATUS "	include path: ${OpenCV_INCLUDE_DIRS}")
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(
	DISCMAHN
#	Main.cpp
	MainOrig.cpp
		DataStructures/PixelState.h
		DataStructures/State.h
		DataStructures/System.h
		DataStructures/TransitionRegion.h
		DataStructures/TransitionRegion.cpp
		DataStructures/PixelState.cpp
		Communication/SocketConnection.h
		Communication/SocketConnection.cpp
#		Imaging/RobotDetection.cpp
#		Imaging/RobotDetection.h
        Imaging/CeilingObjectDetection.cpp
        Imaging/CeilingObjectDetection.h
        Imaging/ImageProcessing.cpp
        Imaging/ImageProcessing.h
		RobotControl/Planner.cpp
		RobotControl/Planner.h
        Imaging/Aruco.cpp Imaging/Aruco.h
		Imaging/MeanShift.h
#		TestMain.cpp
)
target_link_libraries( DISCMAHN ${OpenCV_LIBS} )
