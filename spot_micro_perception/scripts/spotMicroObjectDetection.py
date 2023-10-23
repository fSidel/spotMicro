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


MISSING_FILES_ERROR = """
Paths Not Found! 
Check if the weights, cfg and labels loaded are correct.
If the default files are missing follow instructions 
in weights_download_instruction.
"""


class SpotMicroObjectDetection():  
    nodeName = "object_detection_publisher"
    topicName = "detection_topic" 
    subcribedTopic = "video_topic"


    def __init__(self):
        rospy.init_node(self.nodeName, 
                        anonymous=True)

        self.bridgeObject = CvBridge()


    def loadModel(self):
        self.weights = rospy.get_param('~model_weights')
        self.cfg = rospy.get_param('~model_cfg')
        self.labels = rospy.get_param('~model_labels')

        if not (os.stat(self.weights).st_size > 0 and os.stat(self.cfg).st_size > 0 and os.stat(self.labels).st_size > 0):
            rospy.loginfo(MISSING_FILES_ERROR)

        rospy.loginfo("load weights path: " + self.weights)
        rospy.loginfo("load neural net config path: " + self.cfg)
        rospy.loginfo("load model labels path: " + self.labels)

        with open(self.labels) as labels:
            self.classes = labels.read().splitlines()



    def detectionPublish(self):
        pass


    def cameraCallback(self, message):
        rospy.loginfo("received a video message/frame")
        convertedFrameBackToCV=self.bridgeObject.imgmsg_to_cv2(message)
        cv2.imshow("camera",convertedFrameBackToCV)
        cv2.waitKey(1)


    def run(self):
        self.loadModel()
        rospy.Subscriber(self.subcribedTopic, 
                         Image, 
                         self.cameraCallback)
        rospy.spin()




if __name__ == "__main__":
    smod = SpotMicroObjectDetection()
    smod.run()