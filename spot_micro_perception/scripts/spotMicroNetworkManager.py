#!/usr/bin/env python

"""
Class for sending coordinates of detected objects to spot micro walk and angle node
"""
import rospy  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json

from utilities.ModelConfigReader import ModelConfigReader
from utilities.NetworkRunner import NetworkRunner
from spot_micro_perception.msg import Detections


class SpotMicroDetectionManager():
    nodeName = "detection_manager"
    packageName = 'spot_micro_perception'
    detection_publication = "detection_topic"
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
        self.configs_folder = rospy.get_param('/network_manager/configs_folder')
        self.config_file = rospy.get_param('/network_manager/config_file')
        self.display_detections = rospy.get_param('/network_manager/display_detections')

        rospy.loginfo("Loading Network with the following config: " + self.config_file)

        self.bridge_object = CvBridge()

        self.detection_publisher = rospy.Publisher(SpotMicroDetectionManager.detection_publication,
                                                     Detections,
                                                     queue_size=60)
        
        
        self.rate = rospy.Rate(60)


    def retrieve_setup_data(self):
        rospy.loginfo("Retrieving Configuration...")

        self.network_config = ModelConfigReader(self.configs_folder, 
                                                self.config_file)

        rospy.loginfo(self.network_config.current_config)
        

    def setup_camera(self):
        #TODO: Implement a Service to communicate with the Pi Camera Node
        # to change the resolution of the video capture to better
        # implement the resize of an image (right now it's software based with OpenCV,
        # but the Pi Cam can resize in place with the hardware)
        pass        


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

        rospy.loginfo("load model path: " + self.network_config.model)
        rospy.loginfo("load model config path: " + self.network_config.modelcfg)

        self.network = NetworkRunner()
        self.network.run_network(self.network_config.model, self.network_config.modelcfg)

        if self.display_detections is True:
            self.network.run_detection_display(self.network_config.labels)


    def camera_callback(self, message):
        image = self.bridge_object.imgmsg_to_cv2(message)

        detections_in_frame = self.network.detect_on_image(image, self.network_config.width, self.network_config.height)
        rospy.loginfo("detections made: " + str(detections_in_frame))

        if self.display_detections is True:
            self.network.show_detections(image, detections_in_frame)
        
        detection_string = json.dumps(detections_in_frame)
        self.detection_publisher.publish(Detections(detection_string))


        
    def run(self):
        try:
            rospy.Subscriber(self.video_subscription, 
                            Image, 
                            self.camera_callback)
            self.retrieve_setup_data()
            self.run_network()
            rospy.spin()
        except IOError:
            rospy.signal_shutdown("Shutting Down for IOError")
    


if __name__ == "__main__":
    smom = SpotMicroDetectionManager()
    
    smom.run()