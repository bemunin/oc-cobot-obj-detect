import carb
import omni.ext

from .simple_scene import SimpleScene


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._sim = SimpleScene()
        self._sim.load(start=True)

    def on_shutdown(self):
        self._sim = None
        carb.log_info("Extension shutdown!!!")
