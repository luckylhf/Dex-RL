# Copyright (c) 2021-2024, The RSL-RL Project Developers.
# All rights reserved.
# Original code is licensed under the BSD-3-Clause license.
#
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# Copyright (c) 2025-2026, The Legged Lab Project Developers.
# All rights reserved.
#
# Copyright (c) 2025-2026, The TienKung-Lab Project Developers.
# All rights reserved.
# Modifications are licensed under the BSD-3-Clause license.
#
# This file contains code derived from the RSL-RL, Isaac Lab, and Legged Lab Projects,
# with additional modifications by the TienKung-Lab Project,
# and is distributed under the BSD-3-Clause license.

"""Keyboard controller for SE(2) control."""

import weakref
from collections.abc import Callable

import carb
import omni
import torch
from isaaclab.devices.device_base import DeviceBase

from legged_lab.envs.base.base_env import BaseEnv


class Keyboard(DeviceBase):
    # Fallback teleoperation levels (interactive play only): step size and limits. The play entry
    # point overrides them with the command ranges the policy was actually trained with, so the
    # keyboard can exercise the whole trained envelope (e.g. dex_run goes up to 3.0 m/s).
    VX_STEP, VX_LIMITS = 0.2, (-0.6, 1.2)
    VY_STEP, VY_LIMITS = 0.1, (-0.5, 0.5)
    WZ_STEP, WZ_LIMITS = 0.2, (-1.57, 1.57)

    def __init__(
        self,
        env: BaseEnv,
        vx_limits: tuple[float, float] | None = None,
        vy_limits: tuple[float, float] | None = None,
        wz_limits: tuple[float, float] | None = None,
        vx_step: float | None = None,
        vy_step: float | None = None,
        wz_step: float | None = None,
    ):
        """Initialize the keyboard layer."""
        self.env = env
        # per-instance command levels (defaults keep the historical walk behaviour)
        self.vx_step = self.VX_STEP if vx_step is None else vx_step
        self.vy_step = self.VY_STEP if vy_step is None else vy_step
        self.wz_step = self.WZ_STEP if wz_step is None else wz_step
        self.vx_limits = tuple(self.VX_LIMITS if vx_limits is None else vx_limits)
        self.vy_limits = tuple(self.VY_LIMITS if vy_limits is None else vy_limits)
        self.wz_limits = tuple(self.WZ_LIMITS if wz_limits is None else wz_limits)
        # acquire omniverse interfaces
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        # note: Use weakref on callbacks to ensure that this object can be deleted when its destructor is called
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            lambda event, *args, obj=weakref.proxy(self): obj._on_keyboard_event(event, *args),
        )
        # bindings for keyboard to command
        self._create_key_bindings()
        # dictionary for additional callbacks
        self._additional_callbacks = dict()
        # teleoperation velocity command state: [forward, lateral, yaw]
        self.command = [0.0, 0.0, 0.0]

    def __del__(self):
        """Release the keyboard interface."""
        self._input.unsubscribe_from_keyboard_events(self._keyboard, self._keyboard_sub)
        self._keyboard_sub = None

    def __str__(self) -> str:
        """Returns: A string containing the information of joystick."""
        msg = f"Keyboard Controller for ManagerBasedRLEnv: {self.__class__.__name__}\n"
        return msg

    """
    Operations
    """

    def reset(self):
        pass

    def add_callback(self, key: str, func: Callable):
        pass

    def advance(self):
        pass

    def get_velocity_command(self) -> list[float]:
        """Returns the teleoperation command as ``[forward, lateral, yaw]``."""
        return list(self.command)

    def reset_command(self):
        """Zeroes the teleoperation command (stand still)."""
        self.command = [0.0, 0.0, 0.0]

    """
    Internal helpers.
    """

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Subscriber callback to when kit is updated.

        Reference:
            https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/input-devices/keyboard.html
        """
        # apply the command when pressed
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._INPUT_KEY_MAPPING:
                if event.input.name == "R":
                    self.env.episode_length_buf = torch.ones_like(self.env.episode_length_buf) * 1e6
                else:
                    self._apply_key(self._INPUT_KEY_MAPPING[event.input.name])

        # since no error, we are fine :)
        return True

    def _create_key_bindings(self):
        """Creates default key binding."""
        self._INPUT_KEY_MAPPING = {
            # reset envs
            "R": "reset envs",
            # forward / backward speed level
            "W": "forward up",
            "UP": "forward up",
            "S": "forward down",
            "DOWN": "forward down",
            # lateral speed level
            "A": "lateral up",
            "LEFT": "lateral up",
            "D": "lateral down",
            "RIGHT": "lateral down",
            # yaw rate level
            "Q": "yaw up",
            "E": "yaw down",
            # stop (letters only: SPACE activates the focused Kit UI widget and stops the sim)
            "C": "stop",
            "X": "stop",
        }

    def _apply_key(self, action: str):
        """Adjusts one teleoperation command level per key press (key repeats are ignored)."""
        if action == "forward up":
            self.command[0] = round(min(self.command[0] + self.vx_step, self.vx_limits[1]), 2)
        elif action == "forward down":
            self.command[0] = round(max(self.command[0] - self.vx_step, self.vx_limits[0]), 2)
        elif action == "lateral up":
            self.command[1] = round(min(self.command[1] + self.vy_step, self.vy_limits[1]), 2)
        elif action == "lateral down":
            self.command[1] = round(max(self.command[1] - self.vy_step, self.vy_limits[0]), 2)
        elif action == "yaw up":
            self.command[2] = round(min(self.command[2] + self.wz_step, self.wz_limits[1]), 2)
        elif action == "yaw down":
            self.command[2] = round(max(self.command[2] - self.wz_step, self.wz_limits[0]), 2)
        elif action == "stop":
            self.reset_command()
