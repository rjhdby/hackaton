import RPi.GPIO as GPIO
import time

from camera.setup import distance_threshold
from camera.utils import debug


class Distance:
    TRIG = 16  # raspberry pin
    ECHO = 18  # raspberry pin

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.TRIG, GPIO.OUT, initial=0)
        GPIO.setup(self.ECHO, GPIO.IN)

    # cleanup after usage
    def cleanup(self):
        GPIO.cleanup()

    @debug
    def measure(self):
        GPIO.output(self.TRIG, 1)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, 0)

        start = time.time()
        stop = time.time()

        while GPIO.input(self.ECHO) == 0:
            pass
            start = time.time()

        while GPIO.input(self.ECHO) == 1:
            pass
            stop = time.time()

        distance = int((stop - start) * 17000)  # sm

        return distance

    # Между измерениями нужно подождать немного, если использовать другие методы, то нужно решить в месте использования
    @debug
    def measure_with_delay(self, delay=0.1):
        time.sleep(delay)

        return self.measure()

    @debug
    def is_distance_low_threshold(self, threshold=distance_threshold):
        distance = self.measure()
        lower = distance < threshold
        print(f"Found low distance: {lower}")
        return lower
