import yaml

DEFAULT_CONFIG_FILE = "default_config.json"
CONFIG_FILE = "config.json"


class ConfigLoader:
    config: list

    def load_config(self):
        with open("config.yaml", "r") as file:
            self.config = yaml.safe_load(file)
