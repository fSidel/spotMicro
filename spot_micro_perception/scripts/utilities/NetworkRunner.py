#!/usr/bin/python

"""
This class acts as a manager for managing a CNN given the 
weights and the configuration of said networks. Currently the 
only Networks that can be managed are those supported by 
the DNN module of OpenCV. 
"""

from utilities.YOLONetwork import YOLONetwork
from utilities.MobilenetNetwork import MobilenetNetwork
from utilities.ModelConfigReader import ModelConfigReader


class NetworkRunner():
    def __init__(self):
        pass

    def run_network(self, model, config):

        config_reader = ModelConfigReader()
        print(config_reader.get_config_file_path())

        if (model.endswith(".weights") and 
            config.endswith(".cfg")):
            self.network = YOLONetwork()
            self.network.load_network(model, config),

        elif (model.endswith(".caffe") and
              config.endswith(".caffemodel")):
            self.network = MobilenetNetwork()
            self.network.load_network(model, config)

        else:
            print("No support for this model.")

    def detect_on_image(self, image, input_height=None, input_width=None, input_scale=1/255.0, confidence_threshold=0.80):
        return self.network.detect(image, input_height, input_width, input_scale, confidence_threshold)

