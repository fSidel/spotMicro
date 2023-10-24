#!/usr/bin/env python

"""
Class for sending coordinates of detected objects to spot micro walk and angle node
"""
import rospy  
import roslaunch
import os

MISSING_FILES_ERROR = """
Paths Not Found! 
Check if the weights, cfg and labels loaded are correct.
If the default files are missing follow instructions 
in weights_download_instruction.
"""

class SpotMicroObjectManager():
    nodeName = "detection_manager"
    packageName = 'spot_micro_perception'
    darknetNode = 'spotMicroDarknetDetection.py'

    def __init__(self):
        """
        Node initialization. This node does not subscribe
        or publish to any topic. It's just an intermediary
        to the node that  will actually publish the detections.
        """
        rospy.init_node(self.nodeName, 
                        anonymous=True)
        
        self.launcher = roslaunch.scriptapi.ROSLaunch()
        self.launcher.start()


    def launchDetectionNode(self):
        """
        This function decides what node should be launched
        to actually detect the objects. Each model given to
        the manager may have been trained with a different
        framework and thus may need to be deployed differently.
        After identifying the correct framework, launch the 
        node needed to manage the model and pass the parameters
        of the paths. 
        """
        # Retrieve parameters from arguments in CLI
        # if none are specified, the defaults from
        # the launch file perception.launch will
        # be used
        self.weights = rospy.get_param('~model_weights')
        self.cfg = rospy.get_param('~model_cfg')
        self.labels = rospy.get_param('~model_labels')


        # Check if paths to weights and configuration
        # of the model actually exist
        if not (os.stat(self.weights).st_size > 0 
                and os.stat(self.cfg).st_size > 0 
                and os.stat(self.labels).st_size > 0):
            rospy.loginfo(MISSING_FILES_ERROR)

        rospy.loginfo("load weights path: " + self.weights)
        rospy.loginfo("load neural net config path: " + self.cfg)


        # There is no support for PyTorch, the dnn in OpenCV
        # as of 3.3.1 only supports Torch7 models and 
        # no one really trains those anymore. In OpenCV >=4.5
        # the dnn module can convert PyTorch models into ONNX formats
        # and those are natively supported by OpenCV with the
        # function cv2.dnn.readNetFromONNX(
        if (self.weights.endswith(".weights") 
            and self.cfg.endswith(".cfg")):
            self.YOLO = roslaunch.core.Node(package=SpotMicroObjectManager.packageName, 
                                            node_type=SpotMicroObjectManager.darknetNode,
                                            name="detection_node",
                                            output='screen')
            self.launcher.launch(self.YOLO)
        elif (self.weights.endswith(".caffemodel") 
            and self.cfg.endswith(".prototxt")):
            #TODO: NODE WITH cv2.dnn.readNetFromCaffe()
            pass
        else:
            rospy.loginfo("model not supported, can't launch detection node")
        


    def run(self):
        self.launchDetectionNode()
        rospy.spin()
    



if __name__ == "__main__":
    smom = SpotMicroObjectManager()
    smom.run()