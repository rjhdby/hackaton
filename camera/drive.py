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

        speed = self.pick_random_speed(attack_speed)
        self.run_forward(speed)

    def set_to_center(self):
        self.set_steer(steer_center)

    @debug
    def pick_random_speed(self, diapason):
        speed = random.randint(*diapason)
        print(f"pick random speed {speed}")
        return speed

    @debug
    def drive_forward_for_time(self, speed, stop_time=0.5):
        self.run_forward(speed)
        if time != 0:
            time.sleep(stop_time)
            self.stop()

    @debug
    def run_forward(self, speed):
        print("RUN!!!")
        self.motor.setSpeed(speed)
        self.motor.run(Raspi_MotorHAT.FORWARD)

    @debug
    def turn_and_back(self, speed_diapason, steer=None):
        # выруливание с поворотом назад
        print(f"turn and back for speed_diapason : {speed_diapason} steer: {steer}")
        if steer is None:
            # выруливаем случайно
            self.set_random_steer()
        else:
            # выруливаем определенно
            self.set_steer(steer)
        # сдаем назад
        speed = self.pick_random_speed(speed_diapason)
        self.drive_backward_for_time(speed=speed, stop_time=wall_back_time)

    @debug
    def drive_backward_for_time(self, speed, stop_time=0.5):
        self.run_backward(speed)
        time.sleep(stop_time)
        self.stop()

    @debug
    def run_backward(self, speed):
        print(f"RUN BACK!!! speed {speed}")
        self.motor.setSpeed(speed)
        self.motor.run(Raspi_MotorHAT.BACKWARD)

    @debug
    def stop(self):
        self.motor.MC.setPin(self.motor.IN1pin, 1)
        self.motor.MC.setPin(self.motor.IN2pin, 1)
        self.set_steer(steer_center)

    @debug
    def set_random_steer(self):
        steer = random.sample([steer_left, steer_center, steer_right], k=1)[0]
        self.set_steer(steer)

    @debug
    def set_steer(self, steer):
        self.pwm1.setPWM(14, 0, steer)
