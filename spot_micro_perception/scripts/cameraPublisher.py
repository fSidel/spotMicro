#!/usr/bin/env python

import rospy  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher():
    node_name = "camera_sensor_publisher"
    video_publication = "video_topic"


    def __init__(self):
        rospy.init_node(self.node_name,
                        anonymous=True)
        
        self.capture_publisher = rospy.Publisher(self.video_publication, Image, queue_size=60)
        self.rate = rospy.Rate(24)

        self.video_capture_object = cv2.VideoCapture(0)
        self.bridge_object = CvBridge()
        
    
    def video_capture(self):
        capture_success, cv2_image = self.video_capture_object.read()

        if capture_success == True:
            rospy.loginfo("Video frame captured and published")
            ros_image = self.bridge_object.cv2_to_imgmsg(cv2_image)
            self.capture_publisher.publish(ros_image)

        self.rate.sleep()

    
    def run(self):
        while not rospy.is_shutdown():
            self.video_capture()


if __name__ == "__main__":
    cp = CameraPublisher()
    cp.run()