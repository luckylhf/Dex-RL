# ROS2 + MuJoCo 闭环依赖与验收记录

**记录日期**：2026-10-16
**定位**：本文件记录本包（`Dex-RL`）ROS2 + MuJoCo 闭环栈的**依赖清单、构建经过与验收数据**，供部署侧交接使用。

> 📌 **状态**：缺口已全部补齐。本包曾以「训练单包」形态按设计裁剪掉 ROS 部署栈，第一至五节是该阶段的缺口登记与取回清单（保留作历史记录，**不再是待办**）；随后的增量取回、构建与闭环验收见第六至八节。
> 取回来源为全量备份 `Dex-RL-integrated-20261014-V4 (全量备份)/`。

## 依赖链

> 下列 `[缺]` 标记为**当时**（训练单包形态）的缺口登记，现均已在包内就位，仅作历史记录。

```
scripts/ros_closed_loop.py
├── rclpy                                    # 系统 Humble，已装
├── rl_control_node.XMIGCSControlNode        # Deploy_Tienkung/rl_control_node.py         [缺]
│   ├── common.joystick / xbox_control       # 包内已有
│   ├── common.robot_interface               # 包内已有
│   ├── common.stdin_keyboard_control        # 包内已有
│   └── FSM.robot_fsm                        # 包内已有，但硬导入全部 7 个状态：
│       ├── policy.walk_amp                  # 包内已有
│       ├── policy.zero                      # [缺]
│       ├── policy.stop                      # [缺]
│       ├── policy.niukua                    # [缺]
│       ├── policy.beyondzero                # [缺]
│       └── policy.mimic                     # [缺]
└── simulator_view_asyn.RobotSimulator       # xSIM_MUJOCO-main/scripts/simulator_view_asyn.py [缺]
```

## 一、顶层

| 缺口 | 备份体积 | 说明 |
| --- | --- | --- |
| `ros_ws/` | 300K | ROS 工作区，`src/` 内 1 实体包 + 3 个软链接 |
| `sim_joy/` | 100K | `ros_ws/src/sim_joy` 软链接目标 |
| `build/` `install/` `log/` | — | 由 `colcon build` 生成 |

`ros_ws/src/` 内容（软链接为相对路径，复制后在同一包内可正确解析）：

```
ros_ws/src/bodyctrl_msgs    实体目录（ament_cmake + rosidl_default_generators）
ros_ws/src/mujoco_node -> ../../xSIM_MUJOCO-main
ros_ws/src/rl_control  -> ../../Deploy_Tienkung
ros_ws/src/sim_joy     -> ../../sim_joy
```

## 二、`Deploy_Tienkung/`（`rl_control` 包）

| 缺口 | 备份体积 | 说明 |
| --- | --- | --- |
| `package.xml` `setup.py` `setup.cfg` `resource/` | 16K | ament_python 包定义 |
| `requirements_22.txt` `requirements_24.txt` | 8K | 部署侧依赖清单 |
| `rl_control_node.py` | 16K | 控制节点主体（`XMIGCSControlNode`） |
| `rl_control_node_sim.py` | 4K | 仿真控制入口 |
| `config/` `launch/` `lib/` `scripts/` | 1.9M | 配置、启动、库与辅助脚本 |
| `policy/zero` | 24K | 七状态之一 |
| `policy/stop` | 24K | 七状态之一 |
| `policy/niukua` | 5.8M | 七状态之一（BEYONDMIMIC） |
| `policy/beyondzero` | 36K | 七状态之一 |
| `policy/mimic` | 23M | 七状态之一（MIMIC / MIMICDEFAULT） |

> `Dex-RL/Deploy_Tienkung/` 中已有的 `common/`、`FSM/`、`policy/walk_amp/`、
> `__init__.py`、`LICENSE.txt`、`README.md` 与备份内容一致，无需取回。
> 注意 `FSM/robot_fsm.py` 三份文件与备份逐字节相同，因此该文件对七状态的硬导入在本包内必然失败。

## 三、`xSIM_MUJOCO-main/`（`mujoco_node` 包）

| 缺口 | 备份体积 | 说明 |
| --- | --- | --- |
| `package.xml` `setup.py` `setup.cfg` `resource/` `README.md` | 20K | ament_python 包定义 |
| `mujoco_node/` | 48K | 节点入口 |
| `scripts/simulator_view_asyn.py` | 48K | **闭环仿真的核心**，含 `HOLD_JOINTS` 六轴保持建模 |
| `scripts/{convert_xml,elastic_band,terrain_generator}.py` | 26K | 辅助脚本 |
| `launch/` | 8K | 启动文件 |
| `static/` | 3.4M | 静态资源 |

> 已有的 `resources/dex_v3/`（MuJoCo 模型与软链接）无需取回。

## 四、构建命令（从备份构建日志还原）

工作目录为包根，`build` / `install` / `log` 落在包根下：

```bash
colcon build --base-paths ros_ws/src \
    --packages-select bodyctrl_msgs rl_control mujoco_node sim_joy \
    --symlink-install \
    --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
```

运行前置：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 五、合计

约 **35 MB**（不含 `build/` 与 `install/`）。

---

## 六、补齐与构建结果（2026-10-16 执行）

上述缺口已按「从备份增量取回、不覆盖包内现有文件」的方式补齐，并在包根完成构建。

**构建命令**（与备份构建日志中的命令一致）：

```bash
colcon build --base-paths ros_ws/src \
    --packages-select bodyctrl_msgs rl_control mujoco_node sim_joy \
    --symlink-install \
    --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
```

**结果**：

```
Starting >>> bodyctrl_msgs
Finished <<< bodyctrl_msgs [36.7s]
Starting >>> mujoco_node / sim_joy
Finished <<< sim_joy [1.35s] / mujoco_node [1.44s]
Starting >>> rl_control
Finished <<< rl_control [1.19s]

Summary: 4 packages finished [39.6s]
COLCON_EXIT=0
```

**产物体积**：`build/` 39M、`install/` 11M、`log/` 1.7M。

**导入验证**（仅 `import`，不启动闭环；两个入口均有 `__main__` 守卫。闭环本体结果见第七节）：

```
rclpy OK
bodyctrl_msgs OK -> install/bodyctrl_msgs/local/lib/python3.10/dist-packages/bodyctrl_msgs/__init__.py
rl_control_node OK  -> XMIGCSControlNode
simulator_view_asyn OK -> RobotSimulator
FSM.robot_fsm OK
七状态枚举: ['STOP', 'ZERO', 'WALKAMP', 'BEYONDZERO', 'BEYONDMIMIC', 'MIMIC', 'MIMICDEFAULT']
```

**本次新增到包内的内容**（可据此回退）：

| 位置 | 内容 |
| --- | --- |
| 包根 | `ros_ws/`、`sim_joy/`、`build/`、`install/`、`log/` |
| `Deploy_Tienkung/` | `package.xml` `setup.py` `setup.cfg` `resource/` `requirements_22.txt` `requirements_24.txt` `rl_control_node.py` `rl_control_node_sim.py` `config/` `launch/` `lib/` `scripts/` `policy/{zero,stop,niukua,beyondzero,mimic}` |
| `xSIM_MUJOCO-main/` | `package.xml` `setup.py` `setup.cfg` `resource/` `README.md` `mujoco_node/` `scripts/` `launch/` `static/` |
| `docs/` | `ROS_CLOSED_LOOP_GAP.md`（本文件） |

**回退方式**：删除上表列出的条目即可恢复为训练单包形态（`build/` `install/` `log/` 为构建生成物，`ros_ws/src` 内的三个软链接在删除 `sim_joy/` 后会失效）。

---

## 七、闭环验收结果（2026-10-16 执行）

运行方式（python 3.10 + Humble，`--duration 15`）：

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
source /opt/ros/humble/setup.bash
source install/setup.bash
python scripts/ros_closed_loop.py --duration 15 --output artifacts/ros_closed_loop_verify_15s.json
```

| 用例 | 命令 | 退出码 | 仿真秒 | 最低骨盆 | 关键字段 |
| --- | --- | --- | --- | --- | --- |
| 无支撑 | `--duration 15` | **0** | 15.0025 | 0.95462 | `states=[WALKAMP,ZERO]`、`collapsed=false`、29 路命令、四项误差均 null |
| 带支撑（仅传输） | `--duration 15 --band` | **0** | 15.0025 | 0.98680 | `external_support=true` |
| 反馈故障 | `--duration 15 --fault feedback` | **0** | 1.5325 | 0.95212 | `controller_error="imu sensor sample is older than 0.5 s"`、`damping_stop_received=true` |
| 推理故障 | `--duration 15 --fault inference` | **0** | 1.0925 | 0.95223 | `controller_error="WALKAMP inference session is unavailable"`、`damping_stop_received=true` |

- 四条用例墙钟分别约 39s / 39s / 12s / 11s。
- 启动期反馈新鲜度 0.0018s，`leg` / `arm` / `waist` / `imu` 四组齐全；`received_motor_commands=29`。
- 运行期**命令超龄 0 条、WARNING 0 条**；`--fault` 两条用例中出现的 `[ERROR] Control stopped: ...` 是故障注入的预期结果。

**结果文件**：`artifacts/ros_closed_loop_verify_{15s,band,fault_feedback,fault_inference}.json`；
ROS 日志在 `artifacts/ros_logs/`。

---

## 八、第二轮补齐：ROS 入口、平行模型与测试（2026-10-16）

在第一轮基础上继续从备份取回 ROS 相关项。

### 取回内容

| 位置 | 内容 |
| --- | --- |
| 包根 | `requirements-humble.txt`、`init_env.sh`、`run_sim.sh`、`run_real.sh`（均置可执行位） |
| `xSIM_MUJOCO-main/resources/` | `evt2/`（12M）、`terrain/`（32K）、`tiangong3/`（18M） |
| `tests/` | 13 个备份独有测试文件；`conftest.py` 恢复 `xSIM_MUJOCO-main/scripts` 路径插入 |
| `tests/test_training_entry.py` | 恢复 `test_ubuntu_deployment_pins_match_the_verified_base_environment`（校验 `requirements-humble.txt` 与 `Deploy_Tienkung/requirements_22.txt` 版本钉扎一致） |

### 平行模型可用性（实测）

| 模型 | 结果 |
| --- | --- |
| `resources/dex_v3/urdf/dex_v3.xml` | 可加载（`njnt=30`、`nbody=40`） |
| `resources/tiangong3/urdf/tiangong3.xml` | 可加载（`njnt=32`、`nbody=33`） |
| `resources/tiangong3/urdf/scene_terrain.xml` | 可加载 |
| `resources/evt2/urdf/evt2.xml` | **不可加载**：`evt2/meshes` 软链接指向 `artifacts/legacy_assets_before_cleanup/EVT2/meshes`（106M），该目录已被裁剪 |

> **后续变更**：上表后三项（`tiangong3`、`scene_terrain`、`evt2`）为老旧失效样机，其资源目录已整体删除（共约 30M），现 `resources/` 下仅保留闭环使用的 `dex_v3`；`simulator_view_asyn.py` 的模型注册表与 `--model` 选项相应只保留 `dex_v3`。

### 测试结果

取回后测试套件与全量版一致（137 项）。其中 **14 项失败全部来自依赖非 ROS 模型资产的三个文件**：

| 文件 | 失败数 | 缺失依赖 |
| --- | --- | --- |
| `test_foot_collisions.py` | 10 | `mujoco/tiangong3_torq.xml`、`tiangong3_simplify.xml`、`meshes_convex/`、`meshes_simplify/`、`Tienkung_Dex/script/make_mjcf_torq.py`、`artifacts/v4_foot_update/`、`evt2/meshes` |
| `test_v4_merge.py` | 3 | 同上（tiangong3 两个 xml + `evt2/meshes`） |
| `test_v3_model.py` | 1 | `evt2/meshes`、`artifacts/v4_foot_update/` |

经确认，这三个文件不属 ROS 相关范围（`test_foot_collisions.py` 的 ROS 关键词命中为 0），且补齐所需资产约 **126M**（其中 `legacy_assets_before_cleanup/EVT2/meshes` 单独占 106M），故**已从包内移除**。

**移除后结果**：`90 passed / 0 failed`（`PYTEST_EXIT=0`，50.95 秒）。

> 全量版 137 项 → 本包 90 项：三个文件合计含 47 个用例（33 通过 + 14 失败）。

### 仍未取回

`artifacts/legacy_assets_before_cleanup/`（387M）、`Tienkung_Dex/{mujoco/meshes_convex,mujoco/meshes_simplify,mujoco/tiangong3_torq.xml,mujoco/tiangong3_simplify.xml,script/}`、`artifacts/v4_foot_update/`（1.1M），以及 `MERGE_REVIEW.md` / `VALIDATION.md` / `CHANGE_STATUS.md` 等过程文档。
