#!/usr/bin/env python

"""
Class for sending coordinates of detected objects to spot micro walk and angle node
"""
import rospy  
import sys
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class SpotMicroObjectDetection():  
    nodeName = "object_detection_publisher"
    topicName = "detection_topic" 
    subcribedTopic = "video_topic"


    def __init__(self):
        rospy.init_node(self.nodeName, 
                        anonymous=True)

        self.bridgeObject = CvBridge()

    
    def detectionPublish(self):
        pass


    def cameraCallback(self):
        pass


    def run(self):
        rospy.Subscriber(self.subscribedTopic, 
                         Image, 
                         self.cameraCallback)
        rospy.spin()


if __name__ == "__main__":
    smod = SpotMicroObjectDetection()
    smod.run()