#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from spot_micro_perception.msg import DetectionsInFrame
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np 
import cv2
import json



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
    debugSub = "debug_topic"


    def __init__(self):
        rospy.init_node(self.nodeName, 
                        anonymous=True)
        
        self.bridgeObject = CvBridge()

        rospy.Subscriber(OpenCVDetectionDebug.debugSub,
                         DetectionsInFrame,
                         self.debugCallback)

        with open(rospy.get_param('/detection_publisher/model_labels')) as labels:
            self.classes = labels.read().splitlines()

        self.colors = np.random.uniform(0, 255, size=(len(rospy.get_param('/detection_publisher/model_labels')), 3))
        

    def debugCallback(self, message):
        # Retrieves the capture of the image along with the coordinates 
        # of each detection in the frame.
        
        (ros_image,
         detections_string) = message.frame, message.detections

        image = self.image=self.bridgeObject.imgmsg_to_cv2(ros_image)
        detections = json.loads(detections_string)
        
        rospy.loginfo(detections_string)

        for detection in detections:
            (id,
             centerX,
             centerY) = (detection.get("id"), detection.get("x"), detection.get("y"))
            
            cv2.drawMarker(image,
                           (centerX, centerY),
                           color=self.colors[id],
                           markerType=cv2.MARKER_CROSS,
                           markerSize=10,
                           thickness=3
                        )
        
        cv2.imshow("detections", self.image)
        cv2.waitKey(1)

                    

    def run(self):
        rospy.spin()



if __name__ == "__main__":
    ocvdb = OpenCVDetectionDebug()
    ocvdb.run()
        