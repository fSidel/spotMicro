#!/usr/bin/python

"""
This class manages how to properly interact with a CNN
leveraging a Mobilenet backbone. An example of this would
be the MobilenetSSD neural network.
"""

# Network produces output blob with a shape 1x1xNx7 where N is a number of
# detections and an every detection is a vector of values
# [batchId, classId, confidence, left, top, right, bottom]

import cv2
import numpy as np

from utilities.NetworkFather import NetworkFather

class MobilenetNetwork(NetworkFather):
    def __init__(self):
        super(MobilenetNetwork, self).__init__()

    def load_network(self, caffemodel, prototxt):
        self.network = cv2.dnn.readNetFromDarknet(prototxt, caffemodel)
        return self.network

    def detect(self, image, input_height=None, input_width=None, input_scale=1.0, confidence_threshold=0.80):
        image_height = image.shape[0]
        image_width = image.shape[1]

        input_height = image_height if (input_height is None) else input_height
        input_width = image_width if (input_width is None) else input_width

        blob = cv2.dnn.blobFromImage(image,
                                     scalefactor=input_scale,
                                     size=(input_width, input_height),
                                     crop=False)

        output = self.network.forward()

        detections_in_frame = []

        for detection in output[0, 0]:
            confidence = detection[2]
            if confidence > confidence_threshold:
                corner_x = int(detection[3] * input_width)
                corner_y = int(detection[4] * input_height)
                box_width = int(detection[5] * input_width)
                box_height = int(detection[6] * input_height)
                id = int(detection[1]) - 1  # Skip background label

                center_x = round(corner_x + box_width / 2)
                center_y = round(corner_y + box_height / 2)

                detections_in_frame.append({"id": id,
                                            "x": center_x,
                                            "y": center_y})
                
        return detections_in_frame



                
        

