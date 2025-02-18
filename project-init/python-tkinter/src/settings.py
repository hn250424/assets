import json
from typing import Dict, Any
from path import SETTINGS_JSON_PATH


class One:
    def __init__(self, oneElement: int):
        self.oneElement = oneElement


class Two:
    def __init__(self, twoElement: int, id: str = None):
        self.twoElement = twoElement
        self.id = id


class Settings:

    def __init__(self, one: Dict[str, One], two: Dict[str, Two]):
        self.one = one
        self.two = two

    def to_dict(self) -> Dict[str, Dict[str, Dict[str, Any]]]:
        return {
            'one': {
                key: {
                    'oneElement': self.one[key].oneElement,
                } for key in self.one
            },
            'two': {
                key: {
                    'id': self.two[key].id,
                    'twoElement': self.two[key].twoElement,
                } for key in self.two
            },
        }

    
def get_settings() -> Settings:
    with open(SETTINGS_JSON_PATH, 'r') as file:
        data = json.load(file)

        one = {
            key: One(**value) for key, value in data.get('one', {}).items()  
        }

        two = {
            key: Two(**value) for key, value in data.get('two', {}).items()  
        }

        return Settings(one, two)


def set_settings(settings: Settings):
    data = settings.to_dict()
    with open(SETTINGS_JSON_PATH, 'w') as file:
        json.dump(data, file, indent=4)