#!/usr/bin/env python

"""
Class for sending coordinates of detected objects to spot micro walk and angle node
"""
import rospy  
import roslaunch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
import os

from utilities.NetworkRunner import NetworkRunner
from spot_micro_perception.msg import Detections
from spot_micro_perception.msg import DetectionsInFrame



class SpotMicroDetectionManager():
    nodeName = "detection_manager"
    packageName = 'spot_micro_perception'
    darknetNode = 'spotMicroDarknetDetection.py'
    detection_publication = "detection_topic"
    debug_publication = "debug_topic" 
    video_subscription = "video_topic"

    def __init__(self):
        """
        Node initialization. This node does not subscribe
        or publish to any topic. It's just an intermediary
        to the node that  will actually publish the detections.
        """
        rospy.init_node(self.nodeName, 
                        anonymous=True)
        
        # Retrieve parameters from arguments in CLI
        # if none are specified, the defaults from
        # the launch file perception.launch will
        # be used
        self.model = rospy.get_param('~network_model')
        self.config = rospy.get_param('~network_config')
        self.labels = rospy.get_param('~network_labels')
        self.width = rospy.get_param('/detection_publisher/model_width')
        self.height = rospy.get_param('/detection_publisher/model_height')
        rospy.loginfo("load labels path: " + self.labels)
        

        self.bridge_object = CvBridge()

        self.detection_publisher = rospy.Publisher(SpotMicroDetectionManager.detection_publication,
                                                     Detections,
                                                     queue_size=60)
        
        self.debug_publisher = rospy.Publisher(SpotMicroDetectionManager.debug_publication,
                                               DetectionsInFrame,
                                               queue_size=60)
        
        self.rate = rospy.Rate(60)
        

    def run_network(self):
        """
        This function decides what node should be launched
        to actually detect the objects. Each model given to
        the manager may have been trained with a different
        framework and thus may need to be deployed differently.
        After identifying the correct framework, launch the 
        node needed to manage the model and pass the parameters
        of the paths. 
        """

        # Check if paths to weights and configuration
        # of the model actually exist
        if not (os.stat(self.model).st_size > 0 
                and os.stat(self.config).st_size > 0 
                and os.stat(self.labels).st_size > 0):
            rospy.loginfo("No Network Found!")

        rospy.loginfo("load model path: " + self.model)
        rospy.loginfo("load model config path: " + self.config)

        self.network = NetworkRunner()
        self.network.run_network(self.model, self.config)


    def camera_callback(self, message):
        image = self.bridge_object.imgmsg_to_cv2(message)

        detections_in_frame = self.network.detect_on_image(image, self.width, self.height)
        rospy.loginfo("detections made: " + str(detections_in_frame))
        

        ros_image = self.bridge_object.cv2_to_imgmsg(image)
        detection_string = json.dumps(detections_in_frame)

        self.detection_publisher.publish(Detections(detection_string))
        self.debug_publisher.publish(DetectionsInFrame(ros_image,
                                                       detection_string))


        
    def run(self):
        rospy.Subscriber(self.video_subscription, 
                         Image, 
                         self.camera_callback)
        self.run_network()
        rospy.spin()
    


if __name__ == "__main__":
    smom = SpotMicroDetectionManager()
    
    smom.run()