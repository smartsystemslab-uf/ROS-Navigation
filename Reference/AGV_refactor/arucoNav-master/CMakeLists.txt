set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8)
project( arucoTest )
find_package( OpenCV REQUIRED )
message(STATUS "OpenCV library status:")
message(STATUS "	version: ${OpenCV_VERSION}")
message(STATUS "	libraries: ${OpenCV_LIBS}")
message(STATUS "	include path: ${OpenCV_INCLUDE_DIRS}")
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( 
	arucoTest
	arucoDetect.cpp 
)
target_link_libraries( arucoTest ${OpenCV_LIBS} )
