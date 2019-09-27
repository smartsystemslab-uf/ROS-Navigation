#!/bin/bash
set -e -x
echo "loading camera drive"
#sudo modprobe bcm2835-v4l2

echo "setting Pins"

g++ -o twoCamsOneRobot piSerialMultiCamTest.cpp -lwiringPi

gpio -v

gpio mode 1 out
gpio write 1 0
gpio mode 23 out
gpio write 23 0
gpio mode 24 out
gpio write 24 0

gpio readall

