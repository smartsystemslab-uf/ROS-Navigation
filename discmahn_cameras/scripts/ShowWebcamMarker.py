#!/usr/bin/env python

from visualization_msgs.msg import Marker
import rospy
import math

topic = 'camera_markers'
publisher = rospy.Publisher(topic, Marker, queue_size=0)

rospy.init_node('CameraMarker')

while not rospy.is_shutdown():

   marker = Marker()
   marker.header.frame_id = "/webcam"
   marker.type = marker.SPHERE
   marker.action = marker.ADD
   marker.scale.x = 0.2
   marker.scale.y = 0.2
   marker.scale.z = 0.2
   marker.color.a = 1.0
   marker.color.r = 1.0
   marker.color.g = 0.0
   marker.color.b = 0.0
   marker.pose.orientation.w = 1.0
   marker.pose.position.x = 0
   marker.pose.position.y = 0
   marker.pose.position.z = 0

   # Publish the MarkerArray
   publisher.publish(marker)


   rospy.sleep(0.01)
