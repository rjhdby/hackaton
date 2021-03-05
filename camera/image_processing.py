import cv2
import imutils

from camera.setup import *


def get_contours_circle_info(mask):
    contours = get_contours(mask)

    if len(contours) == 0:
        return None

    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    c = max(contours, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    return x, y, radius


def get_test_mask(hsv):
    # lower mask (0-10)
    mask0 = cv2.inRange(hsv, lower_red0, upper_red0)
    # upper mask (170-180)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

    mask = mask0 + mask1

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    return mask


def get_mask(hsv, low_color, high_color):
    lower_color = np.array(low_color)
    upper_color = np.array(high_color)

    mask = cv2.inRange(hsv, lower_color, upper_color)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    return mask


def get_contours(mask):
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"found number of contours: {len(contours)}")
    return imutils.grab_contours(contours)
