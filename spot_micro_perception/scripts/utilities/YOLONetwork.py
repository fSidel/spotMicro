#!/usr/bin/python

"""
This class manages how to properly interact with a CNN
leveraging a Darknet backbone. An example of this would
be the YOLO neural network.
"""

# Network produces output blob with a shape NxC where N is a number of
# detected objects and C is a number of classes + 4 where the first 4
# numbers are [center_x, center_y, width, height]

import cv2
import numpy as np

from utilities.NetworkFather import NetworkFather


class YOLONetwork(NetworkFather):
    def __init__(self):
        super(YOLONetwork, self).__init__()


    def load_network(self, weights, cfg):
        self.network = cv2.dnn.readNetFromDarknet(cfg, weights)
        self.layer_names = self.network.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] 
                              for i in self.network.getUnconnectedOutLayers()]
        
        return self.network
    

    def detect(self, image, input_height=None, input_width=None, input_scale=1.0, confidence_threshold=0.80):
        image_height = image.shape[0]
        image_width = image.shape[1]

        input_height = image_height if (input_height is None) else input_height
        input_width = image_width if (input_width is None) else input_width

        blob = cv2.dnn.blobFromImage(image,
                                     scalefactor=input_scale,
                                     size=(input_width, input_height),
                                     swapRB=True)
        
        self.network.setInput(blob)
        outputs = self.network.forward(self.output_layers)

        detections_in_frame = []

        for out in outputs:
            for detection in out:

                scores = detection[5:]
                id = np.argmax(scores)
                confidence = scores[id]

                if(confidence > confidence_threshold):
                    # Scaling back the size of the boundary, because the predictions are made on a 
                    # scaled down image. this operation may result in floating values,since there
                    # are no fractional pixels we truncate and turn the float into an int.
                    
                    (center_x, 
                     center_y) = (detection[0:2] * np.array([image_width, image_height])).astype("int")

                    detections_in_frame.append({"id": id,
                                                "x": center_x,
                                                "y": center_y})
        
        return detections_in_frame


        

        
        
