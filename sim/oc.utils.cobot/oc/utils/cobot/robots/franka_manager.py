from omni.isaac.core.scenes import Scene
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController

from ..bases.creator_task import CreatorTask


class FrankaManager(CreatorTask):
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
