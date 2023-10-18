#!/usr/bin/env python

import cv2
import numpy as np
import rospkg
import json
import os


rospack = rospkg.RosPack()
pkgdir = rospack.get_path('spot_micro_perception')

with open(pkgdir + '/configs/models_dictionary.json', 'r') as f:
   data = json.load(f)

model_name = 'yolov2'
model_setup = data[model_name]


model_width, model_height,  = model_setup['width'], model_setup['height']

cfg = model_setup['cfg']
cfg_path = os.path.join(pkgdir, cfg)

weight = model_setup['weight']
weight_path = os.path.join(pkgdir, weight)

network = cv2.dnn.readNetFromDarknet(cfg_path, weight_path)


labels = model_setup['labels']
labels_path = os.path.join(pkgdir, labels)

with open(labels_path) as voc:
   classes = voc.read().splitlines()

colors = np.random.uniform(0, 255, size=(len(classes), 3))


capture = cv2.VideoCapture(0)
while capture.isOpened():
   success, image = capture.read()
   image_height, image_width, channels = image.shape
   blob = cv2.dnn.blobFromImage(cv2.resize(image, (model_height, model_width)))
   network.setInput(blob)
   detections = network.forward()
   print(detections)
   break


cv2.waitKey(0)
cv2.destroyAllWindows()