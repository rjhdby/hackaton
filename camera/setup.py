import numpy as np

debug_images = True
debug_images_time = 0.5

cam_up = 700
cam_down = 1500
cam_left = 2300
cam_right = 300

cam_x_angle = 170
cam_y_angle = 127

cam_hor_res = 320
cam_ver_res = 240

cam_diapason = cam_left - cam_right

cam_hor_center = 1400
cam_hor_confidence = cam_diapason // 10  # 10%

steer_left = 2600
steer_right = 1700

steer_center = 2250

steer_diapason = steer_left - steer_right

steer_to_cam_multiplier = steer_diapason / cam_diapason
steer_confidence = steer_diapason // 10

# COLORS

lower_red0 = np.array([0, 110, 110])
upper_red0 = np.array([15, 255, 255])
lower_red1 = np.array([165, 110, 110])
upper_red1 = np.array([180, 255, 255])

target_low_color = [50, 200, 50]
target_high_color = [240, 255, 240]

floor_low_color = [20, 20, 20]
floor_high_color = [150, 150, 150]

wall_low_color = [180, 50, 50]
wall_high_color = [255, 255, 255]
