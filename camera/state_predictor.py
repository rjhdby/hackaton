from camera.distance import Distance
from camera.setup import low_wall_radius
from camera.states import States
from camera.utils import debug

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
    def predict(target_info, floor_info, wall_info) -> States:
        if target_info is not None:
            return States.SEE_TARGET

        big_wall = StatePredictor._is_big_wall_area(wall_info)
        if big_wall:
            return States.SEE_WAll

        return States.SEE_FLOOR
