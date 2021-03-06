#!/usr/bin/expect -f

#camera IP address
set camIP 192.168.1.114
#camera pw
set camPW raspberry
#camera directory
set camDir ~/Desktop/refuckulate/ceiling_socket/
#destination directory
set desDir appCamera1

send "running rsync on $camIP \n"

while true {
	#connect via rsync
	spawn rsync -azvh --exclude=*.cpp --exclude=*.h --exclude=CMakeCache.txt --exclude CMakeLists.txt --exclude=*.cmake --exclude=CMake* --exclude=Makefile --exclude=image*.jpg --exclude=detectedRobots.jpg --exclude=robotInCorner.jpg --exclude=PathFindingTest -e ssh pi@$camIP:$camDir rsync/$desDir
	###
	expect {
		-re ".*es.*.*" {
			exp_send "yes\r"
			exp_continue
		}
		-re ".*sword.*" {
			exp_send "$camPW\r"
		}
	}
	interact
	sleep 1
}
