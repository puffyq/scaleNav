import os

try:
    from ruamel.yaml import YAML
except ImportError:
    YAML = None
    import yaml


# Global Configuration Management
class Config:
    def __init__(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(base_dir, "traj_opt.yaml")
        with open(config_path, "r", encoding="utf-8") as config_file:
            if YAML is not None:
                self._data = YAML().load(config_file)
            else:
                self._data = yaml.safe_load(config_file)
        self._data["train"] = True
        self._data["goal_length"] = 2.0 * self._data['radio_range']
        self._data["sgm_time"] = 2 * self._data["radio_range"] / self._data["vel_max_train"]
        self._data["traj_num"] = self._data['horizon_num'] * self._data['vertical_num'] * self._data["radio_num"]

    def __getitem__(self, key):
        return self._data[key]

    def __setitem__(self, key, value):
        self._data[key] = value


cfg = Config()
