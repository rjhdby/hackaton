import cv2
import imutils
import os

from camera.setup import *

from pathlib import Path

debug_images_path = "./images"
Path(debug_images_path).mkdir(parents=True, exist_ok=True)


def get_contours_circle_info(mask, img=None):
    contours = get_contours(mask)

    if len(contours) == 0:
        return None

    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    c = max(contours, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)

    if debug_images and img is not None:
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

            img = img.copy()
            img[:, :, 0] = mask

            cv2.imwrite(f"{debug_images_path}/image+mask_{len(os.listdir(debug_images_path))}.jpg", img)
        except Exception as e:
            print(e)

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
