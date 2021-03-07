import requests

from camera.utils import debug


@debug
def speak_phrase(phrase):
    try:
        requests.get("http://127.0.0.1:8000/test/", timeout=0.0001)
    except requests.exceptions.ReadTimeout:
        pass
