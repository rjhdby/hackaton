import requests

from camera.states import States
from camera.utils import debug


phrases = {
    States.SEE_TARGET: "attack",
    States.SEE_WAll: "wall",
    States.BLOCKED: "i am blocked",
    States.SEE_FLOOR: "go"
}

init_phrase = "ready to destroy"


class SoundPlayer:

    @debug
    @staticmethod
    def init():
        SoundPlayer.say(init_phrase)

    @staticmethod
    @debug
    def speak(state):
        text = phrases[state]
        SoundPlayer.say(text)

    @staticmethod
    @debug
    def say(text):
        try:
            requests.get("http://192.168.88.192:8080/say?text=" + text, timeout=0.01)
        except Exception as e:
            print(e)
