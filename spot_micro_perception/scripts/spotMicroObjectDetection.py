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


    def loadLabels(self):
        self.labels = rospy.get_param('~model_labels')

        with open(self.labels) as labels:
            self.classes = labels.read().splitlines()
        rospy.loginfo("load model labels path: " + self.labels)

        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))


    def loadModel(self):
        self.weights = rospy.get_param('~model_weights')
        self.cfg = rospy.get_param('~model_cfg')

        if not (os.stat(self.weights).st_size > 0 
                and os.stat(self.cfg).st_size > 0 
                and os.stat(self.labels).st_size > 0):
            rospy.loginfo(MISSING_FILES_ERROR)

        rospy.loginfo("load weights path: " + self.weights)
        rospy.loginfo("load neural net config path: " + self.cfg)

        #TODO: Add more models instead of relying only on DarkNet 
        # if (self.weights.endswith(".weights") 
        #     and self.cfg.endswith(".cfg")):
        #     # FUNCTION WITH cv2.dnn.readNetFromDarknet()
        # elif (self.weights.endswith(".caffemodel") 
        #     and self.cfg.endswith(".prototxt")):
        #     # FUNCTION WITH cv2.dnn.readNetFromCaffe()
        # There is no support for PyTorch, the dnn in OpenCV
        # as of 3.3.1 only supports Torch7 models and 
        # no one really trains those anymore. In OpenCV >=4.5
        # the dnn module can convert PyTorch models into ONNX formats
        # and those are natively supported by OpenCV with the
        # function cv2.dnn.readNetFromONNX()
        # .....
    



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