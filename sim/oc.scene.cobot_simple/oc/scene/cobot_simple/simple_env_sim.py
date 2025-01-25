import carb
from oc.utils.cobot import BaseSim  # type: ignore


class SimpleEnvSim(BaseSim):
    def __init__(self):
        super().__init__()

    def set_up_scene(self, scene):
        carb.log_info("Load Simple Env Sim")

        return

    def post_reset(self):
        pass
