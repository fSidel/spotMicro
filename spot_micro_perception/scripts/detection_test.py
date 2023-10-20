#!/usr/bin/env python

import cv2
import numpy as np
import rospkg
import json
import os



# RosPack returns the path to the package.
# This is useful to navigate resources inside
# the package and loading all the necessary 
# configs to setup the neural network.
rospack = rospkg.RosPack()
pkgdir = rospack.get_path('spot_micro_perception')


# Loads correct weights and configurations
# to initialize the neural network.
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

try:
    if os.stat(weight_path).st_size > 0:
       print("weight_path found")
except OSError:
    print("weight_path missing, follow the instructions in /configs/networks/weights_download_instructions.txt")
    exit()

network = cv2.dnn.readNetFromDarknet(cfg_path, weight_path)

confidence_threshold = 0.6
nms_threshold = 0.3



# Determines colors for the boundary boxes used
# in the detection of every instance of a class
# trained in the network.
with open(labels_path) as labels:
   classes = labels.read().splitlines()

colors = np.random.uniform(0, 255, size=(len(classes), 3))


# Provides names of the layers in the network. We must do this because YOLO
# makes predictions for three different scales of the image we have captured.
# Thus we must iterate every layer, find which layer in the network is
# an output layer and save these layers for later when we will filter
# out the detections from the background.
layer_names = network.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in network.getUnconnectedOutLayers()]


# Webcam capture loop.
capture = cv2.VideoCapture(0)
while capture.isOpened():
   # Image Capture
   # We save the data about height and width of
   # the image because we will use it later to scale up
   # the boxes of our detections, since the model does the
   # predictions on a different scale from our original capture
   success, image = capture.read()
   image_height, image_width, channels = image.shape

   # Generating blob, passing it as input in the network
   # and retrieving the output given the known output layers
   # of the model
   blob = cv2.dnn.blobFromImage(image, 1/255.0, (model_height, model_width), swapRB=True)
   network.setInput(blob)
   outputs = network.forward(output_layers)

   # YOLO networks produce an array 1 x (4+S) with S number of scores and 4 datapoints
   # about the coordinates of each detection, containing the center (x,y) of the
   # boundary box along with the height and the width of each box
   # Thus it is structured like this:
   # [centerX, centerY, width, height, scores...] 
   # Now we will filter our data with our confidence threshold
   class_ids = []
   confidences = []
   coords = [] 

   for out in outputs:
      for detection in out:
         scores = detection[5:]
         id = np.argmax(scores)
         confidence = scores[id]

         if(confidence > confidence_threshold):
            # Scaling back the size of the boundary, because
            # the predictions are made on a scaled down image.
            # this operation may result in floating values,
            # since there are no fractional pixels we truncate
            # and turn the float into an int
            boundary = detection[0:4] * np.array([image_width, image_height, image_width, image_height])
            (centerX, centerY, boundary_width, boundary_height) = boundary.astype("int")

            # Determine the position of the lower left corner
            ll_x = int(centerX - boundary_width / 2)
            ll_y = int(centerY - boundary_height / 2)

            # Update list of detections higher then our threshold
            coords.append([ll_x, ll_y, boundary_width, boundary_height])
            confidences.append(float(confidence))
            class_ids.append(id)

            cv2.rectangle(image, (ll_x, ll_y), (boundary_width, boundary_height), colors[id])
            cv2.putText(image, classes[id], (ll_x, ll_y - 10), cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=colors[id])


   print(coords, confidences, class_ids)
   #TODO: FIX NON MAXIMUM SUPPRESSION
   #boxes = cv2.dnn.NMSBoxes(coords, confidences, confidence_threshold, 0.2)
   cv2.imshow("detections", image)
   if cv2.waitKey(10) & 0xFF == ord('q'):
      break

   
capture.release()
cv2.destroyAllWindows()


# MobileNet networks produce an array 1 x 1 x n x 7 with n number of detections,
# each detection is stored in the [0, 0, idx, :] value of the array.
# The detection is an array structured like this :
# [batchId, classId, confidence, leftCoord, topCoord, rightCoord, bottomCoord]