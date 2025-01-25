from ..bases.creator_task import CreatorTask


class FrankaManager(CreatorTask):
    def set_up_scene(self):
        pass

    def post_reset(self):
        pass

    def pre_step(self, time_step_index: int, simulation_time: float):
        pass
