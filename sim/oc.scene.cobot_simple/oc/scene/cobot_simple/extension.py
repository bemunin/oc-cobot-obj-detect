import weakref

import carb
import omni.ext
from omni.isaac.ui.menu import make_menu_item_description
from omni.kit.menu.utils import add_menu_items, remove_menu_items

from .simple_sim import SimpleSim


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._name = "Cobot Simple"
        self._sim = SimpleSim()
        self._sim.load(start=True)
        self._window = None
        self._menu = None

        self._build_menu()

    def on_shutdown(self):
        remove_menu_items(self._menu, self._name)
        self._sim = None
        self._window = None
        self._menu = None
        carb.log_info("Extension shutdown!!!")

    def _build_menu(self):
        self._menu = [
            make_menu_item_description(
                self._ext_id,
                self._name,
                lambda a=weakref.proxy(self): a._handle_click_menu(),
            )
        ]
        add_menu_items(self._menu, "OmniCraft")

    def _build_ui(self):
        pass

    def _handle_click_menu(self):
        if self._window is None:
            return
        self._window.visible = not self._window.visible
