from typing import Optional

import numpy as np
from omni.isaac.core.scenes import Scene
from omni.isaac.core.tasks import BaseTask
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController


class FrankaManager(BaseTask):
    def __init__(self, name: str, offset: Optional[np.ndarray] = None):
        super().__init__(name, offset)
        self._franka: Franka = None
        self._controller: PickPlaceController = None
        self._enable_ros = False

    def set_up_scene(self, scene: Scene):
        super().set_up_scene(scene)
        self._franka = Franka(prim_path="/World/Franka", name="franka")
        scene.add(self._franka)

    def post_reset(self):
        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )

    def pre_step(self, time_step_index: int, simulation_time: float):
        return

    def _execute(self):
        return
