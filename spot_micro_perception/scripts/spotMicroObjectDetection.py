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


    def loadModel(self):
        pass
    
    
    def detectionPublish(self):
        pass


    def cameraCallback(self, message):
        rospy.loginfo("received a video message/frame")
        convertedFrameBackToCV=self.bridgeObject.imgmsg_to_cv2(message)
        cv2.imshow("camera",convertedFrameBackToCV)
        cv2.waitKey(1)


    def run(self):
        rospy.Subscriber(self.subcribedTopic, 
                         Image, 
                         self.cameraCallback)
        rospy.spin()

        self.weights = rospy.get_param('~model_weights')
        self.cfg = rospy.get_param('~model_cfg')

        rospy.loginfo(self.weights)
        rospy.loginfo(self.cfg)


if __name__ == "__main__":
    smod = SpotMicroObjectDetection()
    smod.run()