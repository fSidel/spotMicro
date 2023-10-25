#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np 
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

        with open(rospy.get_param('/detection_publisher/model_labels')) as labels:
            self.classes = labels.read().splitlines()

        self.colors = np.random.uniform(0, 255, size=(len(rospy.get_param('/detection_publisher/model_labels')), 3))
        

    def detectionCallback(self, message):
        # Retrieves coordinates of the boundary box from detection node
        (self.id, 
         self.centerX, 
         self.centerY, 
         self.boundary_width, 
         self.boundary_height) = message.data
        
        self.drawBoxes()


    def cameraCallback(self, message):   
        # Retrieves image from camera node, converts it in a cv2 data type
        # and gets information about the size of the image
        rospy.loginfo("image acquired by debug node, processing cv output of detections...")
        
        self.image=self.bridgeObject.imgmsg_to_cv2(message)
        (self.image_height, 
         self.image_width, 
         channels) = self.image.shape
        


    def drawBoxes(self):
        ll_x = int(self.centerX - self.boundary_width / 2)
        ll_y = int(self.centerY - self.boundary_height / 2)

        cv2.drawMarker(self.image,
                       (self.centerX, self.centerY),
                       self.colors[self.id],
                       markerType=cv2.MARKER_CROSS,
                       markerSize=10
                       )

        cv2.rectangle(self.image, 
                      (ll_x, ll_y), 
                      (self.boundary_width, self.boundary_height), 
                      self.colors[self.id])
        
        cv2.putText(self.image, 
                    self.classes[self.id], 
                    (ll_x, ll_y - 10), 
                    cv2.FONT_HERSHEY_PLAIN, 
                    fontScale=1, 
                    color=self.colors[self.id])
        
        cv2.imshow("detections", self.image)
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
        