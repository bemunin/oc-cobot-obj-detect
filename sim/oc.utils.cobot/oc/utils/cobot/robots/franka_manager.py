from omni.isaac.core.scenes import Scene

from ..bases.creator_task import CreatorTask


class FrankaManager(CreatorTask):
    def set_up_scene(self, scene: Scene):
        pass

    def post_reset(self):
        pass

    def pre_step(self, time_step_index: int, simulation_time: float):
        pass
