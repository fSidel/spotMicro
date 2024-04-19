import cv2
import numpy as np

class DetectionsDisplay():
    def __init__(self, labels):
        self.colors = np.random.uniform(0, 255, 
                                        size=(len(labels), 3))
    
    def show(self, image, detections):
        for detection in detections:
            (id,
             center_x,
             center_y) = (detection.get("id"), detection.get("x"), detection.get("y"))
            
            cv2.drawMarker(image,
                           (center_x, center_y),
                           color=self.colors[id],
                           markerType=cv2.MARKER_CROSS,
                           markerSize=10,
                           thickness=3
                        )
        
        cv2.imshow("detections", image)
        cv2.waitKey(1)