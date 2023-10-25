#!/usr/bin/env python

"""
Class for sending coordinates of detected objects to spot micro walk and angle node
"""
import rospy  
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2


class SpotMicroObjectDetection():  
    nodeName = "object_detection_publisher"
    topicName = "detection_topic" 
    subcribedTopic = "video_topic"
    confidenceThresh = 0.5


    def __init__(self):
        rospy.init_node(self.nodeName, 
                        anonymous=True)

        self.bridgeObject = CvBridge()
        
        self.detection_pub = rospy.Publisher(self.topicName, Int32MultiArray, queue_size=60)
        self.rate = rospy.Rate(5)


    def loadModel(self):
        # Loading parameters and darknet network
        self.weights = rospy.get_param("/detection_publisher/model_weights")
        self.cfg = rospy.get_param("/detection_publisher/model_cfg")
        self.width = rospy.get_param('/detection_publisher/model_width')
        self.height = rospy.get_param('/detection_publisher/model_height')

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

        for out in outputs:
            for detection in out:
                scores = detection[5:]
                id = np.argmax(scores)
                confidence = scores[id]

                if(confidence > SpotMicroObjectDetection.confidenceThresh):
                    # Scaling back the size of the boundary, because
                    # the predictions are made on a scaled down image.
                    # this operation may result in floating values,
                    # since there are no fractional pixels we truncate
                    # and turn the float into an int
                    
                    (centerX, 
                     centerY) = (detection[0:2] * np.array([image_width, image_height])).astype("int")

                    
                    arrayToPublish = Int32MultiArray()
                    arrayToPublish.data = [id, 
                                           centerX, 
                                           centerY]
                    
                    self.detection_pub.publish(arrayToPublish)

                    rospy.loginfo("received coordinates of detected object")
                    rospy.loginfo((id, centerX, centerY))


    def run(self):
        self.loadModel()
        rospy.Subscriber(self.subcribedTopic, 
                         Image, 
                         self.cameraCallback)
        rospy.spin()



if __name__ == "__main__":
    smod = SpotMicroObjectDetection()
    smod.run()