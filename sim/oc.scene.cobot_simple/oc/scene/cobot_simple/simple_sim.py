import numpy as np
from oc.utils.cobot import BaseSim
from oc.utils.cobot.robots.franka_manager import (
    FrankaManager,
)
from omni.isaac.core.scenes import Scene
from omni.isaac.menu import set_camera_view

from .tasks import EnvCreator


class SimpleSim(BaseSim):
    def __init__(self):
        super().__init__()

    def set_up_scene(self, scene: Scene):
        super().set_up_scene(scene)

        set_camera_view(
            eye=np.array([1.74, 1.68, 1.8]),
            target=np.zeros(3),
        )

        self._world.add_task(EnvCreator(name="env_task"))
        self._world.add_task(FrankaManager(name="franka_task"))

    def post_reset(self):
        return
