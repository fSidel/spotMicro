import os
import yaml


MISSING_FILES_ERROR = """
Paths Not Found! 
Check if the weights, cfg and labels loaded are correct.
If the default files are missing follow instructions 
in weights_download_instruction.
"""


class ModelConfigReader:
    def __init__(self, alternative_config_path = None):
        """
        The ModelConfigReader class manages the configuration of the
        default model to load in the detection node of Spotty.
        By default the class expects a config file in 
        ./spot_micro_perception/configs/defaults.yaml, however
        it is possible to override this and provide an alternative 
        configuration for testing purposes.
        """

        self.default_configs = os.path.curdir

    def get_config_file_path(self):
        return self.default_configs