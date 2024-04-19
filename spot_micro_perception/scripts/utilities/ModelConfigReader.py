import os
import yaml


MALFORMED_CONFIG_FILE = """
UNPROPERLY FORMATTED CONFIG FILE.
The file is not properly formatted and some
keys are missing. Check the file to see
if the following keys are set:
    model: 
    config: 
    labels: 
    width: 
    height: 
    confidence_threshold: 
"""

MISSING_CONFIG_FILE = """
NETWORK CONFIG FILE NOT FOUND.
The given configuration may not exist, check
if the path is wrong or if the file exists.
"""

MISSING_MODEL_FILE = """
MODEL FILE NOT FOUND.
The weights of the model have not been found,
thus it is impossible to load the model in memory.
Check if the path is wrong or if the file is missing.
(es. yolov2.weights, mobilenet.caffeemodel).
"""

MISSING_MODELCFG_FILE = """
MODEL CONFIG FILE NOT FOUND.
The configuration of the model has not been found,
thus it is impossible to load the model in memory.
Check if the path is wrong or if the file is missing.
(es. yolov2.cfg, mobilenet.prototxt).
"""

MISSING_LABELS_FILE = """
MODEL CONFIG FILE NOT FOUND.
The labels supported by the model have not been found,
and the detections cannot be labeled. Check if the path 
is wrong or if the file is missing.
(es. coco.txt, voc.txt).
"""


class ModelConfigReader:
    def __init__(self, base_folder, config_file):
        """
        The ModelConfigReader class manages the configuration of the
        default model to load in the detection node of Spotty.
        By default the class expects a config file in 
        ./spot_micro_perception/configs/defaults.yaml, however
        it is possible to override this and provide an alternative 
        configuration for testing purposes.
        """

        self.base_folder = base_folder
        
        self.current_config = {}

        self.path_to_config = os.path.join(base_folder, config_file)

        self._validate_yaml_config_path(self.path_to_config)

        self._extract_config_values(self.path_to_config)
        
        

    def _validate_yaml_config_path(self, path):
        """
            Add description
        """
        if not (os.path.isfile(path) and \
              (path.endswith('.yaml') or path.endswith('.yml'))):
            raise IOError(MISSING_CONFIG_FILE)
        
        self.path_to_config = path


    def _validate_model_path(self, path):
        """
        """
        if not os.path.exists(path):
            raise IOError(MISSING_MODEL_FILE)


    def _validate_modelcfg_path(self, path):
        """
        """
        if not os.path.exists(path):
            raise IOError(MISSING_MODELCFG_FILE)
        
    def _validate_labels_path(self, path):
        """
        """
        if not os.path.exists(path):
            raise IOError(MISSING_LABELS_FILE)

    def _extract_config_values(self, path):
        """
            Add description
        """

        KEYS_OF_INTEREST = ['model', 
                            'modelcfg', 
                            'labels', 
                            'width', 
                            'height', 
                            'confidence_threshold']

        with open(self.path_to_config, 'r') as file:
            data = yaml.safe_load(file)

        for key in KEYS_OF_INTEREST:
            if key in data:
                self.current_config[key] = data[key]
            else:
                raise KeyError(MALFORMED_CONFIG_FILE)
        
        self.current_config['model'] = os.path.join(self.base_folder, self.current_config['model'])
        self.current_config['modelcfg'] = os.path.join(self.base_folder, self.current_config['modelcfg'])
        self.current_config['labels'] = os.path.join(self.base_folder, self.current_config['labels'])

        self.model = self.current_config['model']
        self.modelcfg = self.current_config['modelcfg']
        self.labels = self.current_config['labels']
        self.width = self.current_config['width']
        self.height = self.current_config['height']
        self.confidence_threshold = self.current_config['confidence_threshold']

        self._validate_model_path(self.model)
        self._validate_modelcfg_path(self.modelcfg)
        self._validate_labels_path(self.labels)

        with open(self.labels) as labels:
            self.labels = labels.read().splitlines()
                
        
        