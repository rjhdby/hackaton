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
from camera.drive import Drive
from camera.setup import *

pid_ver = PID(0.3, 0.02, 0.01, setpoint=0, output_limits=(-200, 200))
pid_hor = PID(0.3, 0.02, 0.01, setpoint=0, output_limits=(-200, 200))
pwm1 = Raspi_PWM_Servo_Driver.PWM(0x70)
pwm1.setPWMFreq(250)
pwm1.setPWM(14, 0, 0)


class Worker:
    cam_ver = (cam_up + cam_down) // 2
    cam_hor = cam_hor_center

    camera = PiCamera()

    image = np.empty((240 * 320 * 3,), dtype=np.uint8).reshape((240, 320, 3))

    drive = Drive()

    def __init__(self):
        self._update_cam(0, 0)
        self.camera.resolution = (cam_hor_res, cam_ver_res)
        self.camera.framerate = 24
        self.camera.start_preview()
        sleep(2)

    def _update_cam(self, err_hor, err_ver, dt=None):
        self.cam_ver += int(pid_ver(-err_ver, dt))
        self.cam_hor += int(pid_hor(-err_hor, dt))
        if self.cam_ver < cam_up:
            self.cam_ver = cam_up
            pid_ver.reset()
        if self.cam_ver > cam_down:
            self.cam_ver = cam_down
            pid_ver.reset()
        if self.cam_hor < cam_right:
            self.cam_hor = cam_right
            pid_hor.reset()
        if self.cam_hor > cam_left:
            self.cam_hor = cam_left
            pid_hor.reset()
        print(f"_update_cam: hor: {self.cam_hor}, ver: {self.cam_ver}")
        pwm1.setPWM(0, 0, self.cam_ver)
        pwm1.setPWM(1, 0, self.cam_hor)

    def track_target(self):
        for _ in self.camera.capture_continuous(self.image, format='rgb', use_video_port=True):
            clear_output(wait=True)

            hsv = self._get_hsv()

            target_info = self._get_target_circle_info(hsv)
            if target_info is None:
                self.drive.stop()
                pid_ver.reset()
                pid_ver.reset()
                self._search()
                continue

            x, y, radius = target_info

            angle_x_err = (x / cam_hor_res - 0.5) * cam_x_angle
            angle_y_err = (y / cam_ver_res - 0.5) * cam_y_angle

            x_err = -(cam_left - cam_right) * angle_x_err // 180
            y_err = (cam_down - cam_up) * angle_y_err // 100
            self._update_cam(x_err, y_err)

            self._move()

    def _get_target_circle_info(self, hsv):
        mask = self._get_target_mask(hsv)
        # img = self.image // 4
        contours = self._get_contours(mask)

        if len(contours) == 0:
            return None

        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        return x, y, radius

    def _search(self):
        if self.cam_hor >= cam_hor_center:
            self.cam_hor += 100
        else:
            self.cam_hor -= 100

        if self.cam_hor > cam_left:
            self.cam_hor = cam_hor_center - 100
        if self.cam_hor < cam_right:
            self.cam_hor = cam_hor_center + 100

        pid_hor.reset()
        pwm1.setPWM(1, 0, self.cam_hor)

    def _move(self):
        if abs(self.cam_hor - cam_hor_center) < cam_hor_confidence:
            self.drive.stop()
            return
        self.drive.track(self.cam_hor - cam_hor_center)

    def _get_hsv(self):
        blurred = cv2.GaussianBlur(self.image, (5, 5), 0)
        return cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)

    @staticmethod
    def _get_test_mask(hsv):
        # lower mask (0-10)
        mask0 = cv2.inRange(hsv, lower_red0, upper_red0)
        # upper mask (170-180)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        mask = mask0 + mask1

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        return mask

    @staticmethod
    def _get_target_mask(hsv):
        lower_color = np.array(target_low_color)
        upper_color = np.array(target_high_color)

        mask = cv2.inRange(hsv, lower_color, upper_color)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        return mask

    @staticmethod
    def _get_contours(mask):
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return imutils.grab_contours(contours)
