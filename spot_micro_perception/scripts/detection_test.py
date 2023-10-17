#!/usr/bin/env python

# from os.path import dirname, abspath
import cv2
import numpy as np
import rospkg

rospack = rospkg.RosPack()
pkgdir = rospack.get_path('spot_micro_perception')

model = cv2.dnn.readNetFromDarknet(pkgdir + '/network_config/yolo-voc.cfg', pkgdir + '/network_config/yolo-voc.weights')

classes = []

with open(pkgdir + '/object_labels/voc.txt') as voc:
   classes = voc.read().splitlines()
