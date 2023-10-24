#!/usr/bin/env python

"""
Class for sending coordinates of detected objects to spot micro walk and angle node
"""
import rospy  
import sys
import os
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
        self.rate = rospy.Rate(2)

        
    def loadLabels(self):
        self.labels = rospy.get_param("/detection_publisher/model_labels")

        with open(self.labels) as labels:
            self.classes = labels.read().splitlines()
        rospy.loginfo("load model labels path: " + self.labels)

        #TODO: Decouple realtime detection in opencv showim from the node
        # Determines colors for the boundary boxes used
        # in the detection of every instance of a class
        # trained in the network.
        # self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))


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
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.network.getUnconnectedOutLayers()]   


    def cameraCallback(self, message):
        rospy.loginfo("received a video message/frame")
        image=self.bridgeObject.imgmsg_to_cv2(message)
        image_height, image_width, channels = image.shape
        # cv2.imshow("normal", image)
        # cv2.waitKey(1)
        
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (self.height, self.width), swapRB=True)
        self.network.setInput(blob)
        outputs = self.network.forward(self.output_layers)

        class_ids = []
        confidences = []
        coords = [] 

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
                    boundary = detection[0:4] * np.array([image_width, image_height, image_width, image_height])
                    (centerX, centerY, boundary_width, boundary_height) = boundary.astype("int")

                    #TODO: Add a node to display detected images when not in headless mode
                    # Determine the position of the lower left corner
                    # ll_x = int(centerX - boundary_width / 2)
                    # ll_y = int(centerY - boundary_height / 2)

                    # Update list of detections higher then our threshold
                    # coords.append([ll_x, ll_y, boundary_width, boundary_height])
                    # confidences.append(float(confidence))
                    # class_ids.append(id)

                    # cv2.rectangle(image, (ll_x, ll_y), (boundary_width, boundary_height), self.colors[id])
                    # cv2.putText(image, self.classes[id], (ll_x, ll_y - 10), cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=self.colors[id])
                    
                    rospy.loginfo("Detections found and coordinates published")
                    arrayToPublish = Int32MultiArray()
                    arrayToPublish.data = [id, centerX, centerY]
                    self.detection_pub.publish(arrayToPublish)

        # cv2.imshow("detections", image)
        # cv2.waitKey(1)


    def run(self):
        self.loadLabels()
        self.loadModel()
        rospy.Subscriber(self.subcribedTopic, 
                         Image, 
                         self.cameraCallback)
        rospy.spin()



if __name__ == "__main__":
    smod = SpotMicroObjectDetection()
    smod.run()