#!/usr/bin/expect -f

#camera IP address
set camIP 192.168.1.141
#camera pw
set camPW raspberry
#camera directory
set camDir ~/Desktop/AGV_demo/ceiling_socket/
#destination directory
set desDir socketCam3

send "running rsync on $camIP \n"

while true {
	#connect via rsync
	spawn rsync -azvh --exclude=*.cpp --exclude=*.h --exclude=*.txt --exclude=*.cmake --exclude=CMake* --exclude=Makefile --exclude=image*.jpg --exclude=detectedRobots.jpg --exclude=robotInCorner.jpg --exclude=PathFindingTest -e ssh pi@$camIP:$camDir rsync/$desDir
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
