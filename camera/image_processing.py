import cv2
import imutils
import os
import time

from camera.setup import *

from pathlib import Path
from matplotlib import pyplot as plt
import numpy as np

debug_images_path = "./images"


class Processor:
    _last_debug_time = time.time()
    run_path = f"{debug_images_path}/run_{len(os.listdir(debug_images_path))}"
    Path(run_path).mkdir(parents=True, exist_ok=True)

    def get_contours_circle_info(self, mask, img=None):
        contours = self.get_contours(mask)

        if len(contours) == 0:
            return None

        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if debug_images and img is not None and time.time() - self._last_debug_time > debug_images_time:
            _last_debug_time = time.time()
            try:
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # only proceed if the radius meets a minimum size
                if radius > 2:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(img, (int(x), int(y)), int(radius),
                               (0, 255, 255), 2)
                    cv2.circle(img, center, 5, (0, 0, 255), -1)

                img_new = img.copy()
                img_new[:, :, 0] = mask
                img_new = cv2.cvtColor(img_new, cv2.COLOR_HSV2RGB)
                self.save_image("image+mask", img_new)

            except Exception as e:
                print(e)

        return x, y, radius

    def save_image(self, prefix, img):
        try:
            if debug_images:
                path = f"{self.run_path}/{prefix}"
                Path(path).mkdir(parents=True, exist_ok=True)
                cv2.imwrite(f"{path}/{len(os.listdir(path))}.jpg", img)
                plt.imshow(img)
                plt.show()
        except Exception as e:
            print(e)

    @staticmethod
    def input_to_hsv(img):
        img = img.copy()
        blurred = cv2.GaussianBlur(img, (3, 3), 0)
        # processor.save_image("input", blurred)
        img = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
        # processor.save_image("hsv_input", img)
        return img

    @staticmethod
    def get_test_mask(hsv):
        # lower mask (0-10)
        mask0 = cv2.inRange(hsv, lower_red0, upper_red0)
        # upper mask (170-180)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        mask = mask0 + mask1

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        return mask

    @staticmethod
    def get_mask(hsv, low_color, high_color):
        lower_color = np.array(low_color)
        upper_color = np.array(high_color)

        mask = cv2.inRange(hsv, lower_color, upper_color)

        # mask = cv2.erode(mask, None, iterations=2)
        # mask = cv2.dilate(mask, None, iterations=2)

        return mask

    @staticmethod
    def get_contours(mask):
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"found number of contours: {len(contours)}")
        return imutils.grab_contours(contours)
