#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray

"""
This class listens the coordinates topic of detected objects and the video
capture topic to render a real time video of the detected objects with 
boundaries drawn on top of the detection.
This class slows down the already slow detection process so only initialize
the node for debug purposes when on a virtualized development environment
"""


class OpenCVDetectionDebug():  
    nodeName = "detection_debug" 
    subcribedTopic = "detection_topic"


    def __init__(self):
        rospy.init_node(self.nodeName, 
                        anonymous=True)
        

    def detectionCallback(self, message):
        rospy.loginfo(message)


    def run(self):
        rospy.Subscriber(self.subscribedTopic,
                         Int32MultiArray,
                         self.detectionCallback)
        rospy.spin()


if __name__ == "__main__":
    ocvdb = OpenCVDetectionDebug()
    ocvdb.run()
        