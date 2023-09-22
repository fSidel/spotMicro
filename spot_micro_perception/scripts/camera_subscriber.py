#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2 

"""
# we will send messages in form of images consequently, we need to import Image 
# cv_bridge is a package that consists of a library for converting OpenCV images (of type cv::Mat)
# into a ROS image message and for converting ROS image message back to OpenCV images
# That is, it serves as a bridge between OpenCV and ROS
# here we import OpenCV
"""

subscriberNodeName='camera_sensor_subscriber'
topicName='video_topic'


def callbackFunction(message):
	"""
	This function is called every time a message arrives. This message is a
	cv image data type and it will be converted into a ros image.
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
