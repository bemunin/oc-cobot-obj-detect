from oc.utils.cobot import BaseSim
from oc.utils.cobot.robots.franka_manager import FrankaManager
from omni.isaac.core.scenes import Scene

from .tasks import EnvCreator


class SimpleEnvSim(BaseSim):
    def __init__(self):
        super().__init__()

    def set_up_scene(self, scene: Scene):
        self._world.add_task(EnvCreator(name="env_task"))
        self._world.add_task(FrankaManager(name="franka_task"))
        return
