from camera.distance import Distance
from camera.image_processing import ContourInfo
from camera.setup import *
from camera.states import States
from camera.utils import debug

import math

distance = Distance()


class StatePredictor:

    wall_x = 0
    wall_y = 0
    wall_r = 0

    target_x = 0
    target_y = 0
    target_r = 0

    @staticmethod
    @debug
    def _is_big_wall_area(wall_info):
        """маленькая площадь пола"""
        if wall_info is not None:
            if wall_info.radius > low_wall_radius:
                return True
        return False

    @staticmethod
    @debug
    def _wall_is_near(wall_info: ContourInfo):
        if wall_info is not None:
            return wall_info.y > wall_max_dist

    @debug
    def predict(self, target_info, floor_info, wall_info) -> States:

        if target_info is not None and wall_info is not None and target_info.radius < cam_ver_res / 3:
            max_change_target = max(abs(self.target_r - target_info.radius), abs(self.target_x-target_info.x), abs(self.target_y-target_info.x))
            max_change_wall = max(abs(self.wall_r - wall_info.radius), abs(self.wall_x-wall_info.x), abs(self.wall_y-wall_info.x))

            self.target_r = target_info.radius
            self.target_x = target_info.x
            self.target_y = target_info.y
            self.wall_r = wall_info.radius
            self.wall_x = wall_info.x
            self.wall_y = wall_info.y

            if max_change_target + max_change_wall < block_threshold:
                print("BLOCKED!!!")
                return States.BLOCKED

        if target_info is not None:
            print("TARGET!!!")
            return States.SEE_TARGET

        wall_near = StatePredictor._wall_is_near(wall_info)
        if wall_near:
            print("WALL!!!")
            return States.SEE_WAll

        print("FLOOR!!!")
        return States.SEE_FLOOR
