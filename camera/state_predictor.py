from camera.setup import low_floor_radius
from camera.states import States
from camera.utils import debug


class StatePredictor:

    @staticmethod
    @debug
    def _is_low_floor_area(floor_info):
        """маленькая площадь пола"""
        if floor_info is not None:
            if floor_info.radius > low_floor_radius:
                return False
        return True

    @staticmethod
    @debug
    def predict(target_info, floor_info, wall_info) -> States:
        if target_info is not None:
            return States.SEE_TARGET

        low_floor = StatePredictor._is_low_floor_area(floor_info)
        if low_floor and wall_info is not None:
            return States.SEE_WAll

        if not low_floor:
            return States.SEE_FLOOR
