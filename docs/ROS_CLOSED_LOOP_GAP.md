# ROS2 + MuJoCo 闭环使用与验收

**用途**：在**无真机**条件下，把「控制节点 ↔ xSIM 仿真节点」经 DDS 跑通并验收。
**最后更新**：2026-10-17（验收数据测得于 2026-10-16）

---

## 1. 前置条件

- **ROS 2 Humble**：仅闭环与部署需要；训练与独立 Sim2Sim 不依赖。
- **conda 环境** `env_isaaclab`（Python 3.10）。
- 闭环**必须同时**具备 conda 与 Humble，不能用 `scripts/ubuntu_env.sh` 代劳。
- 所有命令的工作目录为**包根**，`build/` `install/` `log/` 均落在包根下。

包含四个包：`bodyctrl_msgs`（ament_cmake，自定义消息）、`rl_control`（部署侧 `Deploy_Tienkung`）、`mujoco_node`（`xSIM_MUJOCO-main`）、`sim_joy`（手柄与 GUI）。`ros_ws/src/` 中后三个是指向包根同名目录的软链接，与训练侧共用同一份代码。

## 2. 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths ros_ws/src \
    --packages-select bodyctrl_msgs rl_control mujoco_node sim_joy \
    --symlink-install \
    --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
```

每次运行前：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 3. 运行闭环验收

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
source /opt/ros/humble/setup.bash
source install/setup.bash
python scripts/ros_closed_loop.py --duration 15 \
    --output artifacts/ros_closed_loop.json
```

| 参数 | 说明 |
| --- | --- |
| `--duration` | 仿真时长（秒），默认 `12` |
| `--policy` | 显式指定候选 ONNX；不覆盖包内默认模型 |
| `--band` | 带外部支撑的**仅传输**测试，不是步态验收 |
| `--fault feedback` / `--fault inference` | 注入一处故障，验证**仅阻尼停止**（须收到 `kp=0` / `kd=1`） |
| `--output` | 报告 JSON 路径，默认 `artifacts/ros_closed_loop.json` |

> 脚本内部固定 `headless=True`，**不打开 MuJoCo 窗口**，只产出 JSON 报告。需要界面请走第 6 节。
>
> 仓库默认策略为 `Deploy_Tienkung/policy/walk_amp/model/policy.onnx`，实际路径以运行环境的 `walk_amp.yaml` 为准。换模型可修改其 `model_path`，或使用 `--policy` 临时指定；两者都不会自动同步增益。请按根 README「步行模型部署时的 KP/KD 和脚踝转换」核对模型来源、训练快照和 `gains`。

## 4. 判据

判据写入报告 JSON，并反映到进程退出码：

- **常规用例**：`validation_error` / `controller_error` / `simulator_error` / `collapsed` 任一非空，或状态序列中不含 `WALKAMP` → 退出码 **1**。
- **故障用例**：须同时满足「无 `validation_error`」「有 `controller_error`」「`damping_stop_received` 为真」，否则退出码 **1**。

## 5. 验收结论（2026-10-16）

以下是历史仿真结果，测量时的步行部署增益为膝 KP 350、肩 pitch KD 7.5；当前 YAML 已对齐训练快照（分别为 330 和 5），这些结果不构成修改后配置的验收，也不验证真机脚踝转换。

| 用例 | 退出码 | 仿真秒 | 最低骨盆 (m) | 前向位移 (m) | 关键字段 |
| --- | --- | --- | --- | --- | --- |
| 无支撑 `--duration 15` | **0** | 15.0025 | 0.95462 | 0.1322 | `states=[WALKAMP,ZERO]`、`collapsed=false`、29 路命令 |
| 无支撑，改用 3300 导出策略 | **0** | 15.0025 | 0.95232 / 0.95126 | **0.1592 / 0.1831** | 两次独立复跑，`collapsed=false`、29 路命令 |
| 带支撑 `--band` | **0** | 15.0025 | 0.98680 | −0.0223 | `external_support=true` |
| 反馈故障 `--fault feedback` | **0** | 1.5325 | 0.95212 | 0.3074 | `controller_error="imu sensor sample is older than 0.5 s"`、`damping_stop_received=true` |
| 推理故障 `--fault inference` | **0** | 1.0925 | 0.95223 | 0.1517 | `controller_error="WALKAMP inference session is unavailable"`、`damping_stop_received=true` |

- 启动期等 `leg` / `arm` / `waist` / `imu` 四组实时反馈就绪后才开始控制；运行期**命令超龄 0 条、无 WARNING**。故障用例出现的 `[ERROR] Control stopped: ...` 是注入后的预期结果。
- 原始报告：`artifacts/ros_closed_loop_verify_{15s,band,fault_feedback,fault_inference}.json`、`artifacts/ros_closed_loop_3300.json`；ROS 日志在 `artifacts/ros_logs/`。

> **波动说明**：3300 策略两次独立复跑的最低骨盆与位移并不相同（0.95232 / 0.95126、0.1592 / 0.1831 m；其中一次的报告存于 `artifacts/ros_closed_loop_3300.json`），该指标**逐次有波动**。比较时请按同一口径多跑几次看趋势，不要用单次数值下结论。

## 6. 启动带界面的仿真栈

```bash
./init_env.sh     # 首次：创建 .venv 并安装 requirements-humble.txt / requirements_22.txt
./run_sim.sh      # 仿真栈；真机栈用 ./run_real.sh
```

`run_sim.sh` 只做 `colcon build` → `source install` → `ros2 launch`，检测不到 `.venv` 会直接退出，因此首次必须先跑 `init_env.sh`。

## 7. 已知边界

- 闭环验收的是「**未倒地 + 状态机可推进**」，**不代表速度跟踪合格**；0.4 m/s 低速跟踪偏弱是当前已知边界。
- `--band` 仅验证传输链路，不作为步态判据。
- 独立的（非 ROS）MuJoCo Sim2Sim 已单独验证：5.32 s / 532 步、前向 1.267 m、未倒地，见 `artifacts/sim2sim_walk3300_mujoco.json`。
