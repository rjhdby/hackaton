from camera.distance import Distance
from camera.image_processing import ContourInfo
from camera.setup import *
from camera.states import States
from camera.utils import debug

import math

distance = Distance()


class StatePredictor:

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

    @staticmethod
    @debug
    def predict(target_info, floor_info, wall_info, prev_target_radius, prev_wall_radius) -> States:
        target_change = abs(prev_target_radius - target_info.radius)
        wall_change = abs(prev_wall_radius - wall_info.radius)

        if target_change < block_threshold and wall_change < block_threshold:
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
