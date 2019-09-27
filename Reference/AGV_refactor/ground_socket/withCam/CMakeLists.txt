 cmake_minimum_required(VERSION 2.8)
 project( main )
 find_package( OpenCV REQUIRED )
 find_package( WiringPi REQUIRED)
 
 include_directories( ${OpenCV_INCLUDE_DIRS} ${WIRINGPI_INCLUDE_DIRS} )
 add_executable( main main.cpp  )
# add_definitions( -lwiringPi)
 target_link_libraries( main pthread )
 target_link_libraries( main ${WIRINGPI_LIBRARIES} )
 target_link_libraries( main ${OpenCV_LIBS} )
 
