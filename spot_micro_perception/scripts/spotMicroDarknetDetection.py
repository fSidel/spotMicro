#!/usr/bin/env python

"""
Class for sending coordinates of detected objects to spot micro walk and angle node
"""
import rospy  
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from spot_micro_perception.msg import DetectionsInFrame
from cv_bridge import CvBridge
import numpy as np
import cv2
import json


class SpotMicroObjectDetection():  
    nodeName = "object_detection_publisher"
    detectionPub = "detection_topic"
    debugPub = "debug_topic" 
    videoSub = "video_topic"


    def __init__(self):
        rospy.init_node(self.nodeName, 
                        anonymous=True)

        self.bridgeObject = CvBridge()
        
        self.debug_pub = rospy.Publisher(SpotMicroObjectDetection.debugPub,
                                         DetectionsInFrame,
                                         queue_size=60)
        
        rospy.Subscriber(self.videoSub, 
                         Image, 
                         self.cameraCallback)
        
        self.rate = rospy.Rate(5)


        # Loading parameters and darknet network
        self.weights = rospy.get_param("/detection_publisher/model_weights")
        self.cfg = rospy.get_param("/detection_publisher/model_cfg")
        self.width = rospy.get_param('/detection_publisher/model_width')
        self.height = rospy.get_param('/detection_publisher/model_height')
        self.confidenceThresh = 0.5
        self.network = cv2.dnn.readNetFromDarknet(self.cfg, self.weights)
        rospy.loginfo("loaded Darknet")


        # Provides names of the layers in the network. We must do this because YOLO
        # makes predictions for three different scales of the image we have captured.
        # Thus we must iterate every layer, find which layer in the network is
        # an output layer and save these layers for later when we will filter
        # out the detections from the background.
        self.layer_names = self.network.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] 
                              for i in self.network.getUnconnectedOutLayers()]



    def cameraCallback(self, message):
        image=self.bridgeObject.imgmsg_to_cv2(message)

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

                if(confidence > self.confidenceThresh):
                    # Scaling back the size of the boundary, because the predictions are made on a 
                    # scaled down image. this operation may result in floating values,since there
                    #  are no fractional pixels we truncate and turn the float into an int.
                    
                    (centerX, 
                     centerY) = (detection[0:2] * np.array([image_width, image_height])).astype("int")

                    detections_in_frame.append({"id": id,
                                                "x": centerX,
                                                "y": centerY})
      
        self.debug_pub.publish(DetectionsInFrame(self.bridgeObject.cv2_to_imgmsg(image),
                                                 json.dumps(detections_in_frame)))



    def run(self):
        rospy.spin()




if __name__ == "__main__":
    smod = SpotMicroObjectDetection()
    smod.run()