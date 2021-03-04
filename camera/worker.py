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
        pwm1.setPWM(0, 0, self.cam_ver)
        pwm1.setPWM(1, 0, self.cam_hor)

    def track(self):
        for _ in self.camera.capture_continuous(self.image, format='rgb', use_video_port=True):
            clear_output(wait=True)

            hsv = self._get_hsv()
            mask = self._get_mask(hsv)
            # img = self.image // 4
            contours = self._get_contours(mask)

            if len(contours) == 0:
                pid_ver.reset()
                pid_ver.reset()
                continue

            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size

            # if radius > 5:
            #     # draw the circle and centroid on the frame,
            #     # then update the list of tracked points
            #     cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            #     cv2.circle(img, center, 5, (0, 0, 255), -1)

            angle_x_err = (x / cam_hor_res - 0.5) * cam_x_angle
            angle_y_err = (y / cam_ver_res - 0.5) * cam_y_angle

            x_err = -(cam_left - cam_right) * angle_x_err // 180
            y_err = (cam_down - cam_up) * angle_y_err // 100
            self._update_cam(x_err, y_err)

            self._move()

    def _move(self):
        if abs(self.cam_hor - cam_hor_center) < cam_hor_confidence:
            self.drive.stop()
            return
        self.drive.track(self.cam_hor)

    def _get_hsv(self):
        blurred = cv2.GaussianBlur(self.image, (5, 5), 0)
        return cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)

    @staticmethod
    def _get_mask(hsv):
        # lower mask (0-10)
        lower_red = np.array([0, 110, 110])
        upper_red = np.array([15, 255, 255])
        mask0 = cv2.inRange(hsv, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([165, 110, 110])
        upper_red = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask0 + mask1

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        return mask

    @staticmethod
    def _get_contours(mask):
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return imutils.grab_contours(contours)
