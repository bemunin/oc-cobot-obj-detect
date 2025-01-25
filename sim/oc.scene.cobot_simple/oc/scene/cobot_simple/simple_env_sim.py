import carb
from oc.utils.cobot import BaseSim


class SimpleEnvSim(BaseSim):
    def __init__(self):
        super().__init__()

    def set_up_scene(self, scene):
        self._world.scene.add_ground_plane()
        carb.log_info("Add ground plane")
        return

    def post_reset(self):
        pass
