from Raspi_MotorHAT import Raspi_PWM_Servo_Driver
from simple_pid import PID
import time
from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image
from IPython.display import clear_output
import numpy as np
import cv2
from matplotlib import pyplot as plt
from matplotlib.colors import hsv_to_rgb
import imutils
import random

from camera.drive import Drive
from camera.setup import *
from camera.state_predictor import StatePredictor
from camera.states import States
from camera.utils import debug
from camera.image_processing import *

pid_ver = PID(0.3, 0.02, 0.01, setpoint=0, output_limits=(-200, 200))
pid_hor = PID(0.3, 0.02, 0.01, setpoint=0, output_limits=(-200, 200))
pwm1 = Raspi_PWM_Servo_Driver.PWM(0x70)
pwm1.setPWMFreq(250)
pwm1.setPWM(14, 0, 0)

processor = Processor()
state_predictor = StatePredictor()


class Worker:
    cam_ver = (cam_up + cam_down) // 2
    cam_hor = cam_hor_center

    camera = PiCamera()

    image = np.empty((240 * 320 * 3,), dtype=np.uint8).reshape((240, 320, 3))

    drive = Drive()
    search = 0
    steer = steer_right

    roaming = 12

    def __init__(self):
        self._update_cam(0, 0)
        self.camera.resolution = (cam_hor_res, cam_ver_res)
        self.camera.framerate = 24
        self.camera.exposure_mode = 'sports'
        self.camera.start_preview()
        sleep(2)

    @debug
    def _update_cam(self, err_hor, err_ver, dt=None):
        self.cam_hor += int(pid_hor(-err_hor, dt))

        if self.cam_hor < cam_right:
            self.cam_hor = cam_right
            pid_hor.reset()
        if self.cam_hor > cam_left:
            self.cam_hor = cam_left
            pid_hor.reset()
        # print(f"_update_cam: hor: {self.cam_hor}, ver: {self.cam_ver}")
        pwm1.setPWM(0, 0, cam_down)
        pwm1.setPWM(1, 0, self.cam_hor)

    @debug
    def get_objects_info(self, hsv):
        target_mask = processor.get_mask(hsv, target_low_color, target_high_color)
        processor.save_image("target_mask", target_mask)
        target_info = processor.get_contours_circle_info(target_mask, self.image)
        print(f"target circle info x, y, r {target_info}")

        # floor_mask = processor.get_mask(hsv, floor_low_color, floor_high_color)
        # processor.save_image("floor_mask", floor_mask)
        # floor_info = processor.get_contours_circle_info(floor_mask, self.image)
        # print(f"floor circle info x, y, r {floor_info}")

        wall_mask = processor.get_mask(hsv, wall_low_color, wall_high_color)
        processor.save_image("wall_mask", wall_mask)
        wall_info = processor.get_contours_circle_info(wall_mask, self.image)
        print(f"wall circle info x, y, r {wall_info}")

        return target_info, wall_info

    @debug
    def start_hunting(self):
        for _ in self.camera.capture_continuous(self.image, format='rgb', use_video_port=True):
            clear_output(wait=True)

            hsv = processor.input_to_hsv(processor, self.image)

            info = self.get_objects_info(hsv)
            target_info, wall_info = info

            current_state = state_predictor.predict(target_info, wall_info)
            if target_info is not None:
                self.prev_target_radius = target_info.radius
            if wall_info is not None:
                self.prev_wall_radius = wall_info.radius
            print(f"CURRENT STATE {current_state}")

            if current_state == States.BLOCKED:
                # выруливаем пока случайно
                self.drive.set_random_steer()
                # сдаем назад
                self.drive.drive_backward_for_time(speed=wall_back_speed, stop_time=wall_back_time)
                continue

            if current_state == States.SEE_TARGET:
                self.attack(target_info)
                continue

            if self.search == 0:
                self.center_camera()
            if self.roaming > 10 or self.search > 1:
                self._search(8)
                self.roaming = 0
                continue

            self.roaming += 1

            if current_state == States.SEE_WAll:
                # выруливаем пока случайно
                # self.drive.set_random_steer()
                self.drive.set_steer(self.steer)
                # сдаем назад
                self.drive.drive_backward_for_time(speed=wall_back_speed, stop_time=wall_back_time)
                if random.random() < search_on_proba:
                    if self.steer == steer_right:
                        self.steer = steer_left
                    else:
                        self.steer = steer_right
                continue

            if current_state == States.SEE_FLOOR and self.search < 1:
                if random.random() < random_floor_back:
                    # но иногда сдаем назад из-за залипаний
                    # выруливаем пока случайно
                    self.drive.set_random_steer()
                    # сдаем назад
                    self.drive.drive_backward_for_time(speed=wall_back_speed, stop_time=wall_back_time)
                    continue

                # двигаться немного прямо
                self.drive.set_steer(steer_center)
                self.drive.drive_forward_for_time(speed=floor_go_speed, stop_time=floor_go_time)
                # self.drive.drive_forward_for_time(speed=floor_go_speed, stop_time=0)
                # торможение
                time.sleep(slowdown_time)
                # randomный поиск
                # if random.random() < search_on_proba:
                #     self._search(8)
                continue

            # искать но не должно сюда дойти по идее
            self._search(8)

    @debug
    def attack(self, target_info):
        print(f"do attack for {target_info}")
        angle_x_err = (target_info.x / cam_hor_res - 0.5) * cam_x_angle
        # angle_y_err = (target_info.y / cam_ver_res - 0.5) * cam_y_angle

        x_err = -(cam_left - cam_right) * angle_x_err // 180
        # y_err = (cam_down - cam_up) * angle_y_err // 100
        self._update_cam(x_err, 0)
        self._move(target_info.radius)

    def center_camera(self):
        pid_hor.reset()
        pwm1.setPWM(1, 0, cam_hor_center)

    @debug
    def _search(self, iterations):
        print ("SEARCH")
        if self.search == 0:
            self.search = iterations
        self.search -= 1
        if self.search == -1:
            self.search = 0
            self.drive.set_to_center()
            self.center_camera()
            return

        if self.cam_hor >= cam_hor_center:
            self.cam_hor += 200
        else:
            self.cam_hor -= 200

        if self.cam_hor > cam_left:
            self.cam_hor = cam_hor_center - 200
        if self.cam_hor < cam_right:
            self.cam_hor = cam_hor_center + 200

        self.center_camera()

    @debug
    def _move(self, target_radius):
        if target_radius != 0 and target_radius > cam_ver_res / 3:
            print ("STOP!!!")
            self.drive.stop()
            return
        self.drive.track(self.cam_hor - cam_hor_center)

    def shutdown(self):
        self.drive.stop()
