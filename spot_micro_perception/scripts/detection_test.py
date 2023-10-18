#!/usr/bin/env python

import cv2
import numpy as np
import rospkg
import json
import os


"""
RosPack returns the path to the package.
This is useful to navigate resources inside
the package and loading all the necessary 
configs to setup the neural network.
"""
rospack = rospkg.RosPack()
pkgdir = rospack.get_path('spot_micro_perception')


"""
Loads correct weights and configurations
to initialize the neural network.
"""
with open(pkgdir + '/configs/models_dictionary.json', 'r') as f:
   data = json.load(f)

model_name = 'yolov2'
model_setup = data[model_name]
model_width = model_setup['width']
model_height =  model_setup['height']
cfg = model_setup['cfg']
labels = model_setup['labels']
weight = model_setup['weight']

cfg_path = os.path.join(pkgdir, cfg)
weight_path = os.path.join(pkgdir, weight)
labels_path = os.path.join(pkgdir, labels)

network = cv2.dnn.readNetFromDarknet(cfg_path, weight_path)

confidence_threshold = 0.6


"""
Determines colors for the boundary boxes used
in the detection of every instance of a class
trained in the network.
"""
with open(labels_path) as labels:
   classes = labels.read().splitlines()

colors = np.random.uniform(0, 255, size=(len(classes), 3))


"""
Webcam capture loop.
"""
capture = cv2.VideoCapture(0)
while capture.isOpened():
   # Image Capture
   success, image = capture.read()
   image_height, image_width, channels = image.shape

   # Generating blob, passing it as input in the network
   # and retrieving the output
   blob = cv2.dnn.blobFromImage(cv2.resize(image, (model_height, model_width)))
   network.setInput(blob)
   detections = network.forward()
   


   break


cv2.waitKey(0)
cv2.destroyAllWindows()