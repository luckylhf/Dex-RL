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

import argparse
import os

import torch
from isaaclab.app import AppLauncher

from legged_lab.utils import task_registry
from rsl_rl.runners import AmpOnPolicyRunner, OnPolicyRunner

# local imports
import legged_lab.utils.cli_args as cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")

# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
# Start camera rendering
if "sensor" in args_cli.task:
    args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab_rl.rsl_rl import export_policy_as_jit, export_policy_as_onnx
from isaaclab_tasks.utils import get_checkpoint_path

from legged_lab.envs import *  # noqa:F401, F403
from legged_lab.utils.cli_args import update_rsl_rl_cfg


# omni.ui windows are destroyed together with their Python object, so the overlay must stay
# referenced for the whole run. The play loop unpacks steps as `obs, _, _, _ = env.step(...)`, which
# rebinds every `_` name, hence this explicit keep-alive list.
_TELEOP_OVERLAY_KEEPALIVE: list = []


def build_teleop_overlay(title: str, teleop, position: tuple[float, float] = (60.0, 150.0)):
    """Creates a small floating Kit window with the teleoperation key map and the live command.

    The key map prints the step size and the min/max of the *keyboard in use*, so the numbers always
    match the task being played instead of being hard-coded for dex_walk.

    ``position`` is passed at construction time on purpose: omni.ui only forwards a position to the
    underlying windowing system when it is set explicitly, otherwise the window keeps the default
    ``kWindowFloatInvalid`` (FLT_MAX) and Kit decides where to put it. Coordinates are Kit points
    relative to the Isaac Sim main window, so the default parks the overlay over the viewport.
    """
    import asyncio

    import omni.appwindow
    import omni.kit.app
    import omni.ui as ui

    vx_step, vx_min, vx_max = teleop.vx_step, teleop.vx_limits[0], teleop.vx_limits[1]
    vy_step, vy_min, vy_max = teleop.vy_step, teleop.vy_limits[0], teleop.vy_limits[1]
    # the yaw limits are symmetric, so only the magnitude is shown
    wz_step, wz_max = teleop.wz_step, teleop.wz_limits[1]

    window = ui.Window(
        "Teleop keys",
        width=400,
        height=380,
        visible=True,
        flags=ui.WINDOW_FLAGS_NO_DOCKING,
        position_x=position[0],
        position_y=position[1],
    )
    # survives the step unpacking in the play loop (see _TELEOP_OVERLAY_KEEPALIVE)
    _TELEOP_OVERLAY_KEEPALIVE.append(window)
    with window.frame:
        with ui.VStack(spacing=3, height=0):
            ui.Label(title, height=22, style={"font_size": 15})
            ui.Spacer(height=6)
            ui.Label(f"W / Up     forward  {vx_step:+.1f} m/s   (max {vx_max:>4.1f})", height=18)
            ui.Label(f"S / Down   slower   {-vx_step:+.1f} m/s   (min {vx_min:>4.1f})", height=18)
            ui.Label(f"A / Left   lateral  {vy_step:+.1f}       (max {vy_max:>4.1f})", height=18)
            ui.Label(f"D / Right  lateral  {-vy_step:+.1f}       (min {vy_min:>4.1f})", height=18)
            ui.Label(f"Q / E      yaw      +/-{wz_step:.1f} rad/s (max {wz_max:>4.2f})", height=18)
            ui.Label("C / X      stop     vx = vy = wz = 0", height=18)
            ui.Label("R          reset env", height=18)
            ui.Spacer(height=8)
            command = ui.Label("command  vx=+0.00  vy=+0.00  wz=+0.00  [STAND]", height=26, style={"font_size": 16})
    try:
        print(
            f"[TELEOP] overlay '{window.title}' pos=({window.position_x},{window.position_y}) "
            f"size=({window.width},{window.height}) visible={window.visible}",
            flush=True,
        )
    except Exception as exc:  # UI introspection is best effort only
        print(f"[TELEOP] overlay window created ({type(exc).__name__}: {exc})", flush=True)

    async def _place_window():
        """Waits until Kit registers the window, then pins it over the viewport."""
        app = omni.kit.app.get_app()
        native = None
        for _ in range(180):
            native = ui.Workspace.get_window("Teleop keys")
            if native is not None:
                break
            await app.next_update_async()
        if native is None:
            print("[TELEOP] overlay was not registered by Kit after 180 frames", flush=True)
            return
        # re-apply the position in case the runtime handed the window back invalid
        if native.position_x > 1.0e30 or native.position_y > 1.0e30:
            native.position_x, native.position_y = position
        native.visible = True
        main = omni.appwindow.get_default_app_window()
        print(
            f"[TELEOP] overlay placed pos=({native.position_x:.0f},{native.position_y:.0f}) "
            f"size=({native.width:.0f},{native.height:.0f}) visible={native.visible} | "
            f"kit_window pos={main.get_position()} size={main.get_size()}",
            flush=True,
        )
        for name in ("Viewport", "Stage", "Property", "Content", "Console", "Teleop keys"):
            other = ui.Workspace.get_window(name)
            if other is None:
                print(f"[TELEOP]   window {name!r}: missing", flush=True)
            else:
                print(
                    f"[TELEOP]   window {name!r}: pos=({other.position_x:.0f},{other.position_y:.0f}) "
                    f"size=({other.width:.0f},{other.height:.0f})",
                    flush=True,
                )

    asyncio.ensure_future(_place_window())
    return window, command


def play():
    runner: OnPolicyRunner
    env_cfg: BaseEnvCfg  # noqa:F405

    env_class_name = args_cli.task
    env_cfg_, agent_cfg_ = task_registry.get_cfgs(env_class_name)

    env_cfg = env_cfg_()
    agent_cfg = agent_cfg_()

    # Command ranges the task was *trained* with. The overrides below only shrink the random command
    # generator, which the keyboard teleop replaces anyway; keeping the trained ranges here lets the
    # keyboard (and the on-screen key map) cover the full trained envelope, e.g. 3.0 m/s for dex_run.
    trained_ranges = (
        tuple(env_cfg.commands.ranges.lin_vel_x),
        tuple(env_cfg.commands.ranges.lin_vel_y),
        tuple(env_cfg.commands.ranges.ang_vel_z),
    )

    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.events = None
    env_cfg.scene.max_episode_length_s = 40.0
    env_cfg.scene.num_envs = 50
    env_cfg.scene.env_spacing = 2.5
    env_cfg.commands.rel_standing_envs = 0.0
    env_cfg.commands.ranges.lin_vel_x = (0.0, 1.0)
    env_cfg.commands.ranges.lin_vel_y = (0.0, 0.0)
    env_cfg.commands.ranges.ang_vel_z = (0.0, 0.0)
    env_cfg.scene.height_scanner.drift_range = (0.0, 0.0)

    env_cfg.scene.terrain_generator = None
    env_cfg.scene.terrain_type = "plane"

    if env_cfg.scene.terrain_generator is not None:
        env_cfg.scene.terrain_generator.num_rows = 5
        env_cfg.scene.terrain_generator.num_cols = 5
        env_cfg.scene.terrain_generator.curriculum = False
        env_cfg.scene.terrain_generator.difficulty_range = (0.4, 0.4)

    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs

    agent_cfg = update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.seed = agent_cfg.seed

    env_class = task_registry.get_task_class(env_class_name)
    env = env_class(env_cfg, args_cli.headless)

    log_root_path = os.path.join("logs", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
    log_dir = os.path.dirname(resume_path)

    runner_class: OnPolicyRunner | AmpOnPolicyRunner = eval(agent_cfg.runner_class_name)
    runner = runner_class(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    runner.load(resume_path, load_optimizer=False)

    policy = runner.get_inference_policy(device=env.device)

    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
    export_policy_as_jit(runner.alg.policy, runner.obs_normalizer, path=export_model_dir, filename="policy.pt")
    export_policy_as_onnx(
        runner.alg.policy, normalizer=runner.obs_normalizer, path=export_model_dir, filename="policy.onnx"
    )

    teleop = None
    teleop_label = None
    if not args_cli.headless:
        from legged_lab.utils.keyboard import Keyboard

        vx_range, vy_range, wz_range = trained_ranges
        keyboard = Keyboard(env, vx_limits=vx_range, vy_limits=vy_range, wz_limits=wz_range)
        # Keyboard teleoperation (play only): replace the randomly resampled velocity command by
        # the one held in the keyboard device. No retraining is needed: the policy already consumes
        # the 3-dim velocity command as part of its observation.
        teleop = keyboard
        command_generator = env.command_generator
        command_generator.cfg.resampling_time_range = (1e9, 1e9)
        command_generator.time_left[:] = 1e9
        command_generator.cfg.heading_command = False
        command_generator.cfg.rel_standing_envs = 0.0
        command_generator.is_heading_env[:] = False
        command_generator.is_standing_env[:] = False
        command_generator.cfg.ranges.lin_vel_x = (0.0, 0.0)
        command_generator.cfg.ranges.lin_vel_y = (0.0, 0.0)
        command_generator.cfg.ranges.ang_vel_z = (0.0, 0.0)
        command_generator.vel_command_b[:] = 0.0
        # on-screen key map together with the live command (drag it anywhere over the viewport)
        # keep the window object alive by name (it is also held by the overlay keep-alive list)
        teleop_window, teleop_label = build_teleop_overlay(  # noqa:F841
            f"{env_class_name} play | {os.path.basename(resume_path)}", keyboard
        )
        print(
            f"[TELEOP] {env_class_name} command envelope from the task config: "
            f"vx {vx_range[0]:.1f}~{vx_range[1]:.1f} (step {keyboard.vx_step:.1f}), "
            f"vy {vy_range[0]:.1f}~{vy_range[1]:.1f} (step {keyboard.vy_step:.1f}), "
            f"wz +/-{wz_range[1]:.2f} (step {keyboard.wz_step:.1f}) | C or X stop | R reset",
            flush=True,
        )

    obs, _ = env.get_observations()

    teleop_command = None
    while simulation_app.is_running():
        if teleop is not None:
            command = teleop.get_velocity_command()
            if command != teleop_command:
                teleop_command = command
                state = "STAND" if command == [0.0, 0.0, 0.0] else "MOVE"
                print(
                    f"[TELEOP] command vx={command[0]:.2f} vy={command[1]:.2f} wz={command[2]:.2f} [{state}]",
                    flush=True,
                )
                if teleop_label is not None:
                    teleop_label.text = (
                        f"command  vx={command[0]:+.2f}  vy={command[1]:+.2f}  wz={command[2]:+.2f}  [{state}]"
                    )
            env.command_generator.vel_command_b[:] = torch.tensor(command, device=env.device)

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)


if __name__ == "__main__":
    play()
    simulation_app.close()
