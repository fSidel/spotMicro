#!/usr/bin/env python

import rospy  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

"""
This Node will be used to publish the images as an image data type in
our ROS Topic. This data needs to be converted into a OpenCV Image (type cv::Mat)
because it is the type expected by our image recognition algorithms. To archieve
this we will use CvBridge to conveniently convert between the two types.
"""

publisherNodeName='camera_sensor_publisher'
topicName='video_topic'

rospy.init_node(publisherNodeName,anonymous=True)
publisher=rospy.Publisher(topicName,Image, queue_size=60)
rate = rospy.Rate(30)

videoCaptureObject=cv2.VideoCapture(0)
bridgeObject=CvBridge()


while not rospy.is_shutdown():
	"""
	frameStatus: boolean
	capturedFrame: cv image
	imageToTransmit: ros image
	"""

	frameStatus, capturedFrame = videoCaptureObject.read()
	
	if frameStatus == True:
		rospy.loginfo('Video frame captured and published')
		imageToTransmit=bridgeObject.cv2_to_imgmsg(capturedFrame)
		publisher.publish(imageToTransmit)
	rate.sleep()
