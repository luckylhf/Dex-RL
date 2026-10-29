# TienKung-Lab：Dex 训练与策略回放

本目录包含基于 Isaac Lab 和 RSL RL 的 Dex 运动策略训练、回放和 MuJoCo Sim2Sim 代码。当前注册的任务是 `dex_walk` 和 `dex_run`；训练模型控制 23 个关节，物理频率 400 Hz、策略频率 100 Hz。

环境准备、训练命令、检查点及完整使用流程见[项目 README](../README.md)。本包的 walk 策略已进行独立 MuJoCo 和 ROS 闭环仿真验证；run 策略已进行独立 Sim2Sim 通路测试，步态质量尚未验收。当前模型尚未完成真机运动验收。

## 主要入口

以下路径相对于本目录；运行前请按照项目 README 配置 Isaac Lab 环境。

| 入口 | 用途 |
| --- | --- |
| `legged_lab/scripts/train.py` | 训练 `dex_walk` / `dex_run` |
| `legged_lab/scripts/play.py` | 回放检查点并导出 TorchScript / ONNX；会覆盖该 run 的现有导出文件 |
| `legged_lab/scripts/sim2sim.py` | 独立 MuJoCo 验证，使用导出的策略 |
| `legged_lab/scripts/play_amp_animation.py` | 播放配置中的参考动作 |
| `legged_lab/scripts/gmr_data_conversion.py` | 转换已准备好的动作数据 |
| `legged_lab/scripts/analyze_training_log.py` | 分析训练日志 |

任务配置位于 `legged_lab/envs/dex/`，机器人配置与资产见 [Tienkung_Dex README](legged_lab/assets/Tienkung_Dex/README.md)。运动重定向所需的外部工具和数据需自行准备。

本目录改编自 [TienKung-Lab](https://github.com/Open-X-Humanoid/TienKung-Lab)，许可见 [LICENSE](LICENSE)。

## Acknowledgement
* [GMR](https://github.com/YanjieZe/GMR):General Motion Retargeting.
* [Legged Lab](https://github.com/Hellod035/LeggedLab): a direct IsaacLab Workflow for Legged Robots.
* [Humanoid-Gym](https://github.com/roboterax/humanoid-gym):a reinforcement learning (RL) framework based on NVIDIA Isaac Gym, with Sim2Sim support.
* [RSL RL](https://github.com/leggedrobotics/rsl_rl): a fast and simple implementation of RL algorithms.
* [AMP_for_hardware](https://github.com/Alescontrela/AMP_for_hardware?tab=readme-ov-file): codebase for learning skills from short reference motions using Adversarial Motion Priors.
* [Omni-Perception](https://acodedog.github.io/OmniPerceptionPages/): a perception library for legged robots, which provides a set of sensors and perception algorithms.
* [Warp](https://github.com/NVIDIA/warp): a Python framework for writing high-performance simulation and graphics code.

