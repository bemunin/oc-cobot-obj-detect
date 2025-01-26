from dataclasses import asdict, dataclass
from typing import Optional

import numpy as np
from oc.utils.cobot import obj_utils
from omni.isaac.core.scenes import Scene
from omni.isaac.core.tasks import BaseTask
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController


@dataclass
class PickPlaceCmd:
    object_name: str
    place_at: np.ndarray


class FrankaManager(BaseTask):
    def __init__(self, name: str, offset: Optional[np.ndarray] = None):
        super().__init__(name, offset)
        self._franka: Franka = None
        self._pp_controller: PickPlaceController = None
        self._enable_ros = False
        self._cmds = []
        self._cmd_pointer = 0

    def set_up_scene(self, scene: Scene):
        super().set_up_scene(scene)
        self._franka = Franka(prim_path="/World/Franka", name="franka")

        scene.add(self._franka)

    def post_reset(self):
        self._franka.gripper.set_joint_positions(
            self._franka.gripper.joint_opened_positions
        )
        self._pp_controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )

    def pre_step(self, time_step_index: int, simulation_time: float):
        if not self._enable_ros:
            self._execute()

    # Helpers
    def _execute(self):
        if self._cmd_pointer >= len(self._cmds):
            return

        cmd = self._cmds[self._cmd_pointer]

        if type(cmd) is PickPlaceCmd:
            cmd = PickPlaceCmd(**asdict(cmd))
            target = self._scene.get_object(cmd.object_name)

            if target is None:
                return

            position, _ = target.get_world_pose()

            # adjust pick at the top of the object
            dimen = obj_utils.get_dimension(target.prim_path)
            height_m = dimen[-1]
            est_gripper_length = 0.04
            collision_offset = 0.01

            pick_at = position.copy()
            pick_pos_z = height_m - (est_gripper_length - collision_offset)
            pick_at[2] = pick_pos_z

            action = self._pp_controller.forward(
                picking_position=pick_at,
                placing_position=cmd.place_at,
                current_joint_positions=self._franka.get_joint_positions(),
            )
            self._franka.apply_action(action)

            if self._pp_controller.is_done():
                self._pp_controller.reset()
                self._cmd_pointer += 1

    def add_cmd(self, cmd: PickPlaceCmd):
        self._cmds.append(cmd)

    @property
    def enable_ros(self):
        return self._enable_ros

    @enable_ros.setter
    def enable_ros(self, value: bool):
        self._enable_ros = value
