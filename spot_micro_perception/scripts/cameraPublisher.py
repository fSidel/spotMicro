#!/usr/bin/env python

import rospy  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher():
    nodeName = "camera_sensor_publisher"
    topicName = "video_topic"


    def __init__(self):
        rospy.init_node(self.nodeName,
                        anonymous=True)
        
        self.capture_pub = rospy.Publisher(self.topicName, Image, queue_size=60)
        self.rate = rospy.Rate(10)

        self.videoCaptureObject = cv2.VideoCapture(0)
        self.bridgeObject = CvBridge()
        
    
    def videoCapture(self):
        frameStatus, capturedFrame = self.videoCaptureObject.read()

        if frameStatus == True:
            rospy.loginfo("Video frame captured and published")
            imageToPublish = self.bridgeObject.cv2_to_imgmsg(capturedFrame)
            self.capture_pub.publish(imageToPublish)

        self.rate.sleep()

    
    def run(self):
        while not rospy.is_shutdown():
            self.videoCapture()


if __name__ == "__main__":
    cp = CameraPublisher()
    cp.run()