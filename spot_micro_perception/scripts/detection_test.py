#!/usr/bin/env python

from os.path import dirname, abspath
import cv2
import numpy as np


package = dirname(abspath(__file__))
model = cv2.dnn.readNetFromDarknet(package + '/network_config/yolo-voc.cfg', package + '/network_config/yolo-voc.weights')

classes = []


with open(package + '/object_labels/voc.txt') as voc:
   classes = voc.read().splitlines()
