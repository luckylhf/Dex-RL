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
import hashlib
import traceback

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
parser.add_argument(
    "--init_from",
    type=str,
    default=None,
    help="Checkpoint whose learned policy weights (actor/critic/std) are transferred as a warm start. "
    "The AMP discriminator, amp normalizer, optimizer state and iteration counter start fresh; "
    "use --resume instead when you want to continue a run exactly where it stopped.",
)

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
import os
from datetime import datetime

import torch
from isaaclab.sim import SimulationContext
from isaaclab.utils.io import dump_yaml
from isaaclab_tasks.utils import get_checkpoint_path

from legged_lab.envs import *  # noqa:F401, F403
from legged_lab.utils.cli_args import update_rsl_rl_cfg

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False


def transfer_policy_weights(runner: OnPolicyRunner | AmpOnPolicyRunner, checkpoint_path: str, log_dir: str) -> None:
    """Copies only the learned actor/critic/std weights of a checkpoint into a freshly built runner.

    The AMP discriminator, the AMP observation normalizer, the optimizer state and the iteration
    counter are intentionally left at their initial values: the discriminator has to learn the target
    motion clip from scratch, and the optimizer momentum of the source task would just add noise.
    """
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    source = checkpoint["model_state_dict"]
    target = runner.alg.policy.state_dict()
    transferred = {key: value for key, value in source.items() if key in target and target[key].shape == value.shape}
    skipped = sorted(key for key in source if key not in transferred)
    missing = sorted(key for key in target if key not in transferred)
    runner.alg.policy.load_state_dict(transferred, strict=False)
    runner.current_learning_iteration = 0
    with open(checkpoint_path, "rb") as handle:
        digest = hashlib.sha256(handle.read()).hexdigest()
    report = (
        f"[TRANSFER] init_from={checkpoint_path}\n"
        f"[TRANSFER] source_sha256={digest}\n"
        f"[TRANSFER] source_iteration={checkpoint.get('iter')}\n"
        f"[TRANSFER] policy tensors transferred: {len(transferred)}/{len(source)}\n"
        f"[TRANSFER] policy tensors left at random init: {missing}\n"
        f"[TRANSFER] source tensors skipped: {skipped}\n"
        f"[TRANSFER] AMP discriminator / amp normalizer / optimizer / iteration counter start fresh\n"
    )
    print(report, flush=True)
    # the runner only creates its log directory when it first saves, so do it here
    os.makedirs(log_dir, exist_ok=True)
    with open(os.path.join(log_dir, "init_from.txt"), "w", encoding="utf-8") as handle:
        handle.write(report)


def train():
    runner: OnPolicyRunner | AmpOnPolicyRunner

    env_class_name = args_cli.task
    # 修正：接收配置类，而非直接用 cfg
    env_cfg_cls, agent_cfg_cls = task_registry.get_cfgs(env_class_name)
    env_class = task_registry.get_task_class(env_class_name)

    # 实例化配置类（关键！解决 scene 属性访问问题）
    env_cfg = env_cfg_cls()
    agent_cfg = agent_cfg_cls()

    # 覆盖 num_envs 配置
    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs

    # 更新 RSL-RL 配置
    agent_cfg = update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.seed = agent_cfg.seed

    # 分布式训练配置
    if args_cli.distributed:
        env_cfg.sim.device = f"cuda:{app_launcher.local_rank}"
        agent_cfg.device = f"cuda:{app_launcher.local_rank}"

        # set seed to have diversity in different threads
        seed = agent_cfg.seed + app_launcher.local_rank
        env_cfg.scene.seed = seed
        agent_cfg.seed = seed

    # 创建环境实例
    env = env_class(env_cfg, args_cli.headless)

    # 日志路径配置
    log_root_path = os.path.join("logs", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Logging experiment in directory: {log_root_path}")

    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if agent_cfg.run_name:
        log_dir += f"_{agent_cfg.run_name}"
    log_dir = os.path.join(log_root_path, log_dir)
    runner_class: OnPolicyRunner | AmpOnPolicyRunner = eval(agent_cfg.runner_class_name)
    runner = runner_class(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)

    if args_cli.init_from is not None and agent_cfg.resume:
        raise ValueError(
            "--init_from (warm start from policy weights) and --resume (continue a previous run) are "
            "mutually exclusive; pick one."
        )

    if agent_cfg.resume:
        # get path to previous checkpoint
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
        print(f"[INFO]: Loading model checkpoint from: {resume_path}")
        # load previously trained model
        runner.load(resume_path)
    elif args_cli.init_from is not None:
        print(f"[INFO]: Warm starting the policy from: {args_cli.init_from}")
        transfer_policy_weights(runner, args_cli.init_from, log_dir)

    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)

    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)



if __name__ == "__main__":
    exit_code = 0
    try:
        train()
    except Exception:
        traceback.print_exc()
        exit_code = 1
        simulation_app.app.post_quit(exit_code)
    finally:
        SimulationContext.clear_instance()
        simulation_app.close()
    raise SystemExit(exit_code)
