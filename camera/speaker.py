import requests

from camera.states import States
from camera.utils import debug


phrases = {
    States.SEE_TARGET: "attack",
    States.SEE_WAll: "wall",
    States.BLOCKED: "i am blocked",
    States.SEE_FLOOR: "go"
}


class SoundPlayer:
    @staticmethod
    @debug
    def speak(state):
        text = phrases[state]
        try:
            requests.get("http://192.168.88.192:8080/say?text=" + text, timeout=0.01)
        except Exception as e:
            print(e)
