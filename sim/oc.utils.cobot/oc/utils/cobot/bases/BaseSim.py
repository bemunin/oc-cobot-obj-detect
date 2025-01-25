import asyncio
import gc
from abc import abstractmethod

from omni.isaac.core import World
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.utils.stage import create_new_stage_async


class BaseSim(object):
    def __init__(self) -> None:
        self._world = None
        self._world_settings = {
            "physics_dt": 1.0 / 60.0,
            "stage_units_in_meters": 1.0,
            "rendering_dt": 1.0 / 60.0,
        }

    async def _load_world_async(self, start: bool, new_stage: bool, **kwargs):
        ## Get or create a new clean world
        if World.instance() is None or new_stage:
            self._world = World(**self._world_settings)
            await create_new_stage_async()
        else:
            self._world = World.instance()
            self._world.stop()
            self._world.clear()
            gc.collect()

        ## Initialize context, scene setup, and world reset/pause
        await self._world.initialize_simulation_context_async()

        self.set_up_scene(self._world.scene)

        await self._world.reset_async()
        await self._world.pause_async()

        self.post_reset()

        self._world.add_physics_callback("pre_step", self.pre_step)

        ## If there are any tasks, make sure they do per-step actions
        if len(self._world.get_current_tasks()) > 0:
            self._world.add_physics_callback("tasks_step", self._world.step_async)

        if start:
            await self._world.play_async()

    def load(self, start: bool = True, new_stage: bool = False, **kwargs):
        asyncio.ensure_future(self._load_world_async(start, new_stage, **kwargs))

    def get_world(self):
        return self._world

    @abstractmethod
    def set_up_scene(self, scene: Scene) -> None:
        """used to setup anything in the world, adding tasks happen here for instance.

        Args:
            scene (Scene): [description]
        """
        return

    @abstractmethod
    def post_reset(self) -> None:
        """entry point to setup anythings after set_up_scene and world reset
        this method ensures that all prim paths are retrievable
        """
        return

    @abstractmethod
    def pre_step(self, simulation_time: float) -> None:
        return
