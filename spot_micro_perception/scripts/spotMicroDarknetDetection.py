#!/usr/bin/env python

"""
Class for sending coordinates of detected objects to spot micro walk and angle node
"""
import rospy  
from sensor_msgs.msg import Image
from spot_micro_perception.msg import Detections
from spot_micro_perception.msg import DetectionsInFrame
from cv_bridge import CvBridge
import numpy as np
import cv2
import json


class SpotMicroObjectDetection():  
    node_name = "object_detection_publisher"
    detection_publication = "detection_topic"
    debug_publication = "debug_topic" 
    video_subscription = "video_topic"


    def __init__(self):
        rospy.init_node(self.node_name, 
                        anonymous=True)

        self.bridge_object = CvBridge()

        self.detection_publisher = rospy.Publisher(SpotMicroObjectDetection.detection_publication,
                                                     Detections,
                                                     queue_size=60)
        
        self.debug_publisher = rospy.Publisher(SpotMicroObjectDetection.debug_publication,
                                               DetectionsInFrame,
                                               queue_size=60)
        
        self.rate = rospy.Rate(60)


        # Loading parameters and darknet network
        self.weights = rospy.get_param("/detection_publisher/model_weights")
        self.cfg = rospy.get_param("/detection_publisher/model_cfg")
        self.width = rospy.get_param('/detection_publisher/model_width')
        self.height = rospy.get_param('/detection_publisher/model_height')
        self.confidence_threshold = rospy.get_param('/detection_publisher/confidence_threshold')
        self.network = cv2.dnn.readNetFromDarknet(self.cfg, self.weights)
        rospy.loginfo("Loaded Darknet Neural Network...")


        # Provides names of the layers in the network. We must do this because YOLO
        # makes predictions for three different scales of the image we have captured.
        # Thus we must iterate every layer, find which layer in the network is
        # an output layer and save these layers for later when we will filter
        # out the detections from the background.
        self.layer_names = self.network.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] 
                              for i in self.network.getUnconnectedOutLayers()]


    def camera_callback(self, message):
        image=self.bridge_object.imgmsg_to_cv2(message)

        (image_height, 
         image_width, 
         channels) = image.shape
        
        blob = cv2.dnn.blobFromImage(image, 
                                     1/255.0, 
                                     (self.width, self.height), 
                                     swapRB=True)
        
        self.network.setInput(blob)
        outputs = self.network.forward(self.output_layers)

        detections_in_frame = []

        for out in outputs:
            for detection in out:

                scores = detection[5:]
                id = np.argmax(scores)
                confidence = scores[id]

                if(confidence > self.confidence_threshold):
                    # Scaling back the size of the boundary, because the predictions are made on a 
                    # scaled down image. this operation may result in floating values,since there
                    # are no fractional pixels we truncate and turn the float into an int.
                    
                    (center_x, 
                     center_y) = (detection[0:2] * np.array([image_width, image_height])).astype("int")

                    detections_in_frame.append({"id": id,
                                                "x": center_x,
                                                "y": center_y})


        ros_image = self.bridge_object.cv2_to_imgmsg(image)
        detection_string = json.dumps(detections_in_frame)

        self.detection_publisher.publish(Detections(detection_string))
        self.debug_publisher.publish(DetectionsInFrame(ros_image,
                                                       detection_string))


    def run(self):
        rospy.Subscriber(self.video_subscription, 
                         Image, 
                         self.camera_callback)
        rospy.spin()




if __name__ == "__main__":
    smod = SpotMicroObjectDetection()
    smod.run()