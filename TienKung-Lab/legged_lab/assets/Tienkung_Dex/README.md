# 天工行者Dex 本体资产

本目录是**当前裁剪包内保留的本体资产**：为 Isaac Lab 训练与 ROS2 / MuJoCo 部署提供 URDF、网格与本体配置。它源自上游完整的 `tiangong3_urdf` ROS 描述包，但只保留训练与闭环真正用到的部分——被移除的内容见文末「与全量资产包的关系」。

## 命名说明

文档与文件名中出现的几种名称，指的都是同一台机器人：

| 中文 | 英文 | 说明 |
| --- | --- | --- |
| 天工行者Dex | Walker Tienkung Dex | 机器人名称。「行者」对应 `Walker`，`Dex` 是机器人指定型号 |
| 天工、天工3、天工3.0 | Tiangong、TIANGONG 3.0 | 早期写法，仍保留在部分技术标识中 |

**`tiangong` 与 `tienkung` 是同一名称的两种拼写**：早期标识沿用 `tiangong`，后续统一为 `tienkung`。本包的显示名称已按新写法更新，但为免牵动 `package://` 资源路径与文件名，下列技术标识**保持旧拼写不变**：

- `urdf/tiangong3*.urdf` 的文件名，以及 `urdf/tiangong3.urdf` 中的 `<robot name="tiangong3_urdf">`（由它派生的 MuJoCo 模型沿用同一名称）

因此文档里的「天工行者Dex / Walker Tienkung Dex」与文件名里的 `tiangong3` 指同一套模型。上游仓库 `Open-X-Humanoid/TienKung_URDF` 用的是新拼写，而其内部存放模型的目录仍叫 `tiangong3_urdf`，与本地情况一致。

## 目录结构

| 路径 | 内容 |
| --- | --- |
| `urdf/` | 三份 URDF，见「模型文件」 |
| `meshes/` | 40 个 STL，训练与部署模型共用 |
| `mujoco/meshes_import/` | 4 个 OBJ；闭环主模型 `dex_v3.xml` 实际引用其中 3 个（`pelvis.obj`、`waist_pitch_link_lower.obj`、`waist_pitch_link_upper.obj`） |
| `walk.py`、`run.py` | 对应任务的 `DEX_V3_CFG`：初始姿态 `init_state`、执行器 PD 增益 `actuators` 等本体配置 |
| `关键参数/` | V3 关键参数表（CSV）与参考图（PNG） |
| `__init__.py` | 使本目录可作为资产包被导入 |

## 模型文件

| 文件 | links / joints | 其中 fixed | 活动关节 | 用途 |
| --- | --- | --- | --- | --- |
| `urdf/tiangong3.urdf` | 39 / 38 | 7 | 31 | **源本体**；`scripts/sync_dex.py` 由它派生另外两份 |
| `urdf/tiangong3_locomotion.urdf` | 39 / 38 | 15 | **23** | **训练用**：`walk.py` / `run.py` 的 `asset_path` 指向它（头两轴 + 6 个末端轴 fixed） |
| `urdf/tiangong3_ros.urdf` | 39 / 38 | 9 | **29** | **部署侧参照**：保留备查，训练与独立 Sim2Sim 都不加载它 |

> 训练侧只驱动 23 个关节；部署侧模型保留 29 路执行器，独立 Sim2Sim 用 `MjSpec` 删除策略不控制的 6 个末端关节（`elbow_yaw` / `wrist_pitch` / `wrist_roll` 左右各 3）以复现训练侧模型。

## 依赖

本目录不构成 ROS 包，也没有 `package.xml`：训练侧由 `legged_lab` 通过文件路径读取（见 `walk.py` / `run.py` 的 `asset_path`），部署与 Sim2Sim 侧由 `xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml` 加载本目录的网格。因此本目录**不需要 `colcon build`**，也没有 RViz 显示启动文件。

## MuJoCo 导入资源

`mujoco/meshes_import/` 提供 Sim2Sim 与闭环模型导入所需的 OBJ。上游完整资产包中的两份 MJCF（`tiangong3_torq.xml` 用凸包网格、`tiangong3_simplify.xml` 用 10% 减面网格）与生成它们的 `script/` **不在本裁剪包内**（见下节）。

## 与全量资产包的关系

本裁剪包移除了以下内容，如需这些资源，请使用对应的完整资产包：

- `mujoco/` 下的 `meshes_convex/`（凸包网格）、`meshes_simplify/`（减面网格）与两份 MJCF `tiangong3_torq.xml`、`tiangong3_simplify.xml`
- 生成与校验脚本 `script/`（`check_link_and_update.py`、`check_joint_and_update.py`、`prime_to_convex.py`、`prime_to_simplify.py`、`make_mjcf_torq.py`）
- RViz 显示资源 `launch/display.launch.py`、`config/display.rviz`
- 随包独立 ROS 包（BrainCo Revo2 / Inspire RH5DG2 灵巧手描述包、USD 候选资产包）及其派生 URDF（`tiangong3_brainco.urdf`、`tiangong3_inspire.urdf`）
- 本目录的许可证正文 `LICENSE`

`scripts/sync_dex.py` **不依赖**上述被移除内容：它只读 `urdf/tiangong3.urdf`、写 `urdf/tiangong3_ros.urdf`，并解析 `walk.py` 取 `DEX_V3_CFG` 参数。

## 许可证

本目录未包含独立的许可证正文；据上游说明该资产采用 OpenAtom Open Hardware License 1.0，使用和分发前请核对相应完整许可文本。本仓库整体与各子包的许可证见包根 README 的「许可证」一节及 `TienKung-Lab/LICENSE`、`Deploy_Tienkung/LICENSE.txt`、`xSIM_MUJOCO-main/LICENSE`。
