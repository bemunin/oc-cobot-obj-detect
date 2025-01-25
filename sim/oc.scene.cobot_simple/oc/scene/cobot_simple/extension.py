import carb
import omni.ext

from .simple_env_sim import SimpleEnvSim


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._sim = SimpleEnvSim()
        self._sim.load(start=False)

    def on_shutdown(self):
        self._sim = None
        carb.log_info("Extension shutdown!!!")
