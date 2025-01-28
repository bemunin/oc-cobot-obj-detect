import functools
import gc

import carb
import numpy as np
import omni.ext
import omni.ui as ui
from oc.utils.cobot.robots.franka_manager import (
    DelayCmd,
    FrankaManager,
    MoveCmd,
    PickPlaceCmd,
    SetGripperCmd,
)
from omni.isaac.core import World
from omni.isaac.ui import ui_utils
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
        self._build_ui()

    def on_shutdown(self):
        remove_menu_items(self._menu, self._name)
        self._sim = None
        self._window = None
        # self._menu = None
        carb.log_info("Extension shutdown!!!")
        gc.collect()

    def _build_menu(self):
        callback = functools.partial(self._handle_click_menu)
        self._menu = [
            make_menu_item_description(
                self._ext_id,
                self._name,
                callback,
            )
        ]
        add_menu_items(self._menu, name="OmniCraft")

    def _build_ui(self):
        self._window = omni.ui.Window(
            self._name,
            width=300,
            height=0,
            visible=True,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )

        on_click_run_test = functools.partial(self._handle_click_run_test)
        on_change_enable_ros = functools.partial(self._handle_change_enable_ros)

        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                ui.Spacer(height=5)
                ui_utils.btn_builder(
                    label="Run Test", text="Start", on_clicked_fn=on_click_run_test
                )
                ui.Spacer(height=5)
                ui_utils.cb_builder(
                    label="ROS Connection",
                    default_val=False,
                    tooltip="click to enable ros connection",
                    on_clicked_fn=on_change_enable_ros,
                )
                ui.Spacer(height=5)

        return

    def _handle_click_menu(self):
        if self._window is None:
            return
        self._window.visible = not self._window.visible

    def _handle_click_run_test(self):
        world = World.instance()
        franka: FrankaManager = world.get_task("franka_task")

        if not franka:
            return

        sequences = [
            SetGripperCmd(state="open"),
            DelayCmd(time_sec=0.5),
            PickPlaceCmd(object_name="cylinder", place_at=np.array([0.4, -0.4, 0.08])),
            SetGripperCmd(state="open"),
            PickPlaceCmd(object_name="cube", place_at=np.array([0.4, 0.4, 0.04])),
            MoveCmd(state="standby"),
            SetGripperCmd(state="close"),
        ]
        for cmd in sequences:
            franka.add_cmd(cmd)

    def _handle_change_enable_ros(self, value: bool):
        world = World.instance()
        franka: FrankaManager = world.get_task("franka_task")
        if not franka:
            return

        franka.enable_ros = value
