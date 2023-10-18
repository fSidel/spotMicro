#!/usr/bin/env python

# from os.path import dirname, abspath
import cv2
import numpy as np
import rospkg
import json

rospack = rospkg.RosPack()
pkgdir = rospack.get_path('spot_micro_perception')


model = cv2.dnn.readNetFromDarknet(pkgdir + '/configs/networks/yolov2.cfg', pkgdir + '/configs/networks/yolov2.weights')


classes = []
with open(pkgdir + '/configs/labels/coco.txt') as voc:
   classes = voc.read().splitlines()

print(classes)


data = {}
with open(pkgdir + '/configs/camera_recording_size.json', 'r') as f:
   data = json.load(f)

print(data)
print(data['yolov2'])


colors = np.random.uniform(0, 255, size=(len(classes), 3))


capture = cv2.VideoCapture(0)
while capture.isOpened():
   success, image = capture.read()
   image_height, image_width, channels = image.shape
   print(success, image_height, image_width)
   break


cv2.waitKey(0)
cv2.destroyAllWindows()