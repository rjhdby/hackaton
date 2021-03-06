import numpy as np

debug_images = True
debug_images_time = 1

cam_up = 700
cam_down = 1500
cam_left = 2650
cam_right = 600

cam_x_angle = 170
cam_y_angle = 127

cam_hor_res = 320
cam_ver_res = 240

cam_diapason = cam_left - cam_right

cam_hor_center = 1730
cam_hor_confidence = cam_diapason // 10  # 10%

steer_left = 2700
steer_right = 1800

steer_center = 2400

steer_diapason = steer_left - steer_right

steer_to_cam_multiplier = steer_diapason / cam_diapason
steer_confidence = steer_diapason // 10

# COLORS

lower_red0 = np.array([0, 110, 110])
upper_red0 = np.array([15, 255, 255])
lower_red1 = np.array([165, 110, 110])
upper_red1 = np.array([180, 255, 255])

target_low_color = [50, 100, 70]  # hsv !
target_high_color = [75, 255, 255]

floor_low_color = [0, 4, 4]
floor_high_color = [255, 30, 30]

wall_low_color = [6, 180, 100]
wall_high_color = [12, 255, 255]

# AREA METRICS

low_floor_radius = 30

distance_threshold = 10
