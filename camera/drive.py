from Raspi_MotorHAT import Raspi_MotorHAT
from Raspi_MotorHAT import Raspi_PWM_Servo_Driver
from camera.setup import *
import time


class Drive:
    motor = Raspi_MotorHAT(addr=0x6f, freq=250).getMotor(1)
    pwm1 = Raspi_PWM_Servo_Driver.PWM(0x70)

    def __init__(self):
        self.pwm1.setPWM(14, 0, steer_center)

    def track(self, deviation):
        steer = steer_center + int(deviation * steer_to_cam_multiplier)
        if abs(steer - steer_center) < steer_confidence:
            self.stop()
            return
        if steer < steer_right:
            steer = steer_right
        if steer > steer_left:
            steer = steer_left

        print(f"drive.track: steer: {steer}")

        self.set_steer(steer)
        self.run(50)

    def drive_for_time(self, speed, stop_time=0.5):
        start_time = time.time()
        self.run(speed)
        while time.time() < min(start_time + stop_time, 5):
            continue
        self.stop()

    def run(self, speed):
        print ("RUN!!!")
        self.motor.setSpeed(speed)
        self.motor.run(Raspi_MotorHAT.FORWARD)

    def stop(self):
        self.motor.MC.setPin(self.motor.IN1pin, 1)
        self.motor.MC.setPin(self.motor.IN2pin, 1)
        self.set_steer(steer_center)

    def set_steer(self, steer):
        self.pwm1.setPWM(14, 0, steer)
