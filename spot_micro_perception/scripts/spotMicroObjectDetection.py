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


    def loadLabels(self):
        self.labels = rospy.get_param("/detection_publisher/model_labels")

        with open(self.labels) as labels:
            self.classes = labels.read().splitlines()
        rospy.loginfo("load model labels path: " + self.labels)

        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))


    def loadModel(self):
        self.weights = rospy.get_param("/detection_publisher/model_weights")
        self.cfg = rospy.get_param("/detection_publisher/model_cfg")

        self.network = cv2.dnn.readNetFromDarknet(self.cfg, self.weights)
        rospy.loginfo("loaded Darknet")
        
        self.layer_names = self.network.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.network.getUnconnectedOutLayers()]   


    def detectionPublish(self):
        pass


    def cameraCallback(self, message):
        rospy.loginfo("received a video message/frame")
        convertedFrameBackToCV=self.bridgeObject.imgmsg_to_cv2(message)
        cv2.imshow("camera",convertedFrameBackToCV)
        cv2.waitKey(1)


    def run(self):
        self.loadLabels()
        self.loadModel()
        rospy.Subscriber(self.subcribedTopic, 
                         Image, 
                         self.cameraCallback)
        rospy.spin()




if __name__ == "__main__":
    smod = SpotMicroObjectDetection()
    smod.run()