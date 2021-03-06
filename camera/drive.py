from Raspi_MotorHAT import Raspi_MotorHAT
from Raspi_MotorHAT import Raspi_PWM_Servo_Driver
from camera.setup import *
import time
import random

from camera.utils import debug


class Drive:
    motor = Raspi_MotorHAT(addr=0x6f, freq=250).getMotor(1)
    pwm1 = Raspi_PWM_Servo_Driver.PWM(0x70)

    def __init__(self):
        self.pwm1.setPWM(14, 0, steer_center)

    @debug
    def track(self, deviation):
        steer = steer_center + int(deviation * steer_to_cam_multiplier)
        if steer < steer_right:
            steer = steer_right
        if steer > steer_left:
            steer = steer_left

        print(f"drive.track: steer: {steer}")

        self.set_steer(steer)
        self.run_forward(attack_speed)

    def set_to_center(self):
        self.set_steer(steer_center)

    @debug
    def drive_forward_for_time(self, speed, stop_time=0.5):
        self.run_forward(speed)
        time.sleep(stop_time)
        self.stop()

    @debug
    def run_forward(self, speed):
        print("RUN!!!")
        self.motor.setSpeed(speed)
        self.motor.run(Raspi_MotorHAT.FORWARD)

    @debug
    def drive_backward_for_time(self, speed, stop_time=0.5):
        self.run_backward(speed)
        time.sleep(stop_time)
        self.stop()

    @debug
    def run_backward(self, speed):
        print("RUN BACK!!!")
        self.motor.setSpeed(speed)
        self.motor.run(Raspi_MotorHAT.BACKWARD)

    @debug
    def stop(self):
        self.motor.MC.setPin(self.motor.IN1pin, 1)
        self.motor.MC.setPin(self.motor.IN2pin, 1)
        self.set_steer(steer_center)

    @debug
    def set_random_steer(self):
        steer = random.sample([steer_center, steer_left, steer_right], k=1)[0]
        self.set_steer(steer)

    @debug
    def set_steer(self, steer):
        self.pwm1.setPWM(14, 0, steer)
