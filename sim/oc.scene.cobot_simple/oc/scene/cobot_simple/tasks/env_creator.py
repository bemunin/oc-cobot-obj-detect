from oc.utils.cobot.bases.creator_task import CreatorTask
from omni.isaac.core.scenes import Scene


class EnvCreator(CreatorTask):
    def set_up_scene(self, scene: Scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()

        return

    def post_reset(self):
        return super().post_reset()
