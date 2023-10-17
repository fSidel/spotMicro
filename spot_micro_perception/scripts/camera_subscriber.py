#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2 

"""
# This node subscribes to the camera publisher. The receiving messages are ROS images data types
# and we use cv_bridge to switch back and forth between ROS and OpenCV images.
"""

subscriberNodeName='camera_sensor_subscriber'
topicName='video_topic'


def callbackFunction(message):
	"""
	This function is called every time a message arrives. The received message 
	is a ros image data type and it will be converted into a cv image.
	Afterwards we print the image to the screen.
	"""
	
	bridgeObject=CvBridge()
	rospy.loginfo("received a video message/frame")
	convertedFrameBackToCV=bridgeObject.imgmsg_to_cv2(message)
	
	cv2.imshow("camera",convertedFrameBackToCV)
	cv2.waitKey(1)
	

rospy.init_node(subscriberNodeName, anonymous=True)
rospy.Subscriber(topicName,Image, callbackFunction)
rospy.spin()
cv2.destroyAllWindows()