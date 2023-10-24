#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2

"""
This class listens the coordinates topic of detected objects and the video
capture topic to render a real time video of the detected objects with 
boundaries drawn on top of the detection.
This class slows down the already slow detection process so only initialize
the node for debug purposes when on a virtualized development environment
"""


class OpenCVDetectionDebug():  
    nodeName = "detection_debug" 
    detectionSub = "detection_topic"
    videoSub = "video_topic"


    def __init__(self):
        rospy.init_node(self.nodeName, 
                        anonymous=True)
        
        self.bridgeObject = CvBridge()
        

    def detectionCallback(self, message):
        rospy.loginfo(message)
        (self.id, self.centerX, self.centerY) = message.data


    def cameraCallback(self, message):   
        rospy.loginfo("received a video message/frame")
        self.convertedFrameBackToCV=self.bridgeObject.imgmsg_to_cv2(message)
        cv2.imshow("camera", self.convertedFrameBackToCV)
        cv2.waitKey(1)


    def run(self):
        rospy.Subscriber(OpenCVDetectionDebug.detectionSub,
                         Int32MultiArray,
                         self.detectionCallback)
        rospy.Subscriber(OpenCVDetectionDebug.videoSub,
                         Image,
                         self.cameraCallback)
        rospy.spin()


if __name__ == "__main__":
    ocvdb = OpenCVDetectionDebug()
    ocvdb.run()
        