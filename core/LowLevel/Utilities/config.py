# core/LowLevel/Utilities/config.py
import yaml

class ConfigProfile:
    def __init__(self, filename= 'config.yaml'):
        with open(filename, "r") as f:
            self._profile = yaml.safe_load(f)

    @property
    def profile(self):
        return self._profile


CONFIG = ConfigProfile().profile
