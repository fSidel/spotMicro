#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2 


"""
This node subscribes to the camera publisher. The receiving messages 
are ROS images data typesand we use cv_bridge to switch 
back and forth between ROS and OpenCV images.
"""

class CameraSubscriber():
    node_name = "camera_sensor_subscriber"
    video_subscription = "video_topic"


    def __init__(self):
        rospy.init_node(self.node_name,
                        anonymous=True)
        
        self.bridge_object = CvBridge()

    
    def camera_callback(self, message):
        """
	    This function is called every time a message arrives. The received message 
	    is a ros image data type and it will be converted into a cv image.
	    Afterwards we print the image to the screen.
	    """
             
        rospy.loginfo("received a video message/frame")
        image=self.bridge_object.imgmsg_to_cv2(message)
        cv2.imshow("camera",image)
        cv2.waitKey(1)


    def run(self):
        rospy.Subscriber(self.video_subscription,
                         Image,
                         self.camera_callback)
        rospy.spin()


if __name__ == "__main__":
    cs = CameraSubscriber()
    cs.run()