cam_up = 700
cam_down = 1500
cam_left = 2300
cam_right = 300

cam_x_angle = 170
cam_y_angle = 127

cam_hor_res = 320
cam_ver_res = 240

cam_diapason = cam_left - cam_right

cam_hor_center = (cam_left + cam_right) // 2
cam_hor_confidence = cam_diapason // 10  # 10%

steer_left = 2600
steer_right = 2000

steer_center = (steer_left + steer_right) // 2

steer_diapason = steer_left - steer_right

steer_to_cam_multiplier = int(steer_diapason / cam_diapason)
steer_confidence = steer_diapason // 10
