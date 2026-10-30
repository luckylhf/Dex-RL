# Dex 真机部署与联调记录

本文记录天工行者 Dex 的一次真机部署环境、操作路径和联调中遇到的问题，供后续部署时参考。训练、策略导出和仿真闭环说明见[根目录 README](../README.md)。

## 验证范围

- 机器人：天工行者 Dex，测试记录中的配置为 tg3.0 evt2 / `robot_type 4`。
- 部署机器：x86_64 本体板（node1），Ubuntu 24.04、ROS 2 Jazzy；厂商 EtherCAT 接口、`body_control` 和 `joystick` 位于本体板。
- 测试结果：已完成 `STOP → ZERO → WALKAMP` 真机闭环联调及真机地面行走测试；根据测试反馈，真机测试结果与设计相符，测试项目均通过。地面行走的速度、时长等具体数据未在本记录中提供。
- 测试包内的 `policy.onnx` 和 `walk_amp.yaml` 与当前仓库中的对应文件逐字节一致；本记录未提供实机运行时加载文件的校验值。

本次使用的 `sptlib_python` 是 Python 3.12 的 x86_64 wheel；真机模式下踝关节串并联转换依赖该库。测试时 Thor（node2）为 aarch64，未提供匹配 wheel，也没有本次使用的 EtherCAT 和厂商节点，因此本次部署使用本体板。

## 部署前确认

1. 核对本体板的系统、ROS、厂商软件和网络配置。以下路径及设备名来自本次测试，其他机器可能不同：
   - ROS：`/opt/ros/jazzy/setup.bash`。
   - 厂商工作区：`/home/ubuntu/xos/setup.bash`。
   - EtherCAT 网卡：`enp4s0`；手柄串口：`/dev/ttySBUS0`。
2. 检查厂商 `body_hw_config.yaml` 和实际硬件配置。本次机器原配置缺少 `slave_mode`，驱动按 `8/8/8` 处理，出现预期从站数与实际不符的告警。测试时依据厂商 `config_dex.yaml` 补充了以下配置，修改前应备份原文件：

   ```yaml
   slave_mode:
     - 7
     - 7
     - 5
   ```

   该值对应本次机器的板卡配置，其他机器应先核对，不能直接套用。
3. 准备 Python 3.12 环境与部署依赖，参见仓库根目录 `init_env.sh` 和 `Deploy_Tienkung/requirements_24.txt`。缺少 `python3.12-venv` 时需先安装对应系统包。无 X 显示时，`pynput` 若因无法连接 X 而导入失败，初始化会提示图形输入需要显示环境；其他导入错误仍会使检查失败。
4. 确认机器人有可靠支撑，并按现场规程安排人员、急停和控制权切换。停止厂商服务可能使机器人失去力矩。

## 构建与启动

### 复制到本体板

从仓库所在机器的根目录执行以下命令，将 `ROBOT_HOST` 改为本体板的 SSH 登录目标。该方式保留源码及资源链接和链接目标，排除训练日志、Git 数据、本机环境及构建产物；不使用 `--delete`，不会删除目标端已有文件。目标目录中若有旧的 `.venv`、`build`、`install` 或 `log`，应确认它们适用于当前机器，必要时先备份并清理后重新初始化和构建。

```bash
ROBOT_HOST=ubuntu@本体板地址
rsync -az --exclude='/.git/' --exclude='/.venv/' \
    --exclude='/build/' --exclude='/install/' --exclude='/log/' \
    --exclude='/TienKung-Lab/logs/' --exclude='__pycache__/' \
    --exclude='/docs/原始测试文件.mp4' --exclude='.DS_Store' \
    ./ "$ROBOT_HOST:/home/ubuntu/Dex-RL/"
```

此部署副本不含训练日志和模型存档；它不等同于完整训练包。到本体板后，在仓库根目录检查断开的符号链接：

```bash
cd /home/ubuntu/Dex-RL
find -L . -type l -print
```

检查结果应为空；若有输出，先补齐链接目标并重新检查。接着准备依赖并构建。本次测试中，`rl_control` 的构建依赖检查需要工作区中的四个包一起构建：

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/xos/setup.bash
colcon build --base-paths ros_ws/src \
    --packages-select bodyctrl_msgs rl_control mujoco_node sim_joy --symlink-install
```

厂商工作区与仓库均包含 `bodyctrl_msgs`。测试记录称两份消息定义逐字段一致；运行时实际使用哪一份取决于环境加载顺序。遇到类型或通信问题时，应检查当前环境解析到的包及消息定义。

本次启动脚本依次加载系统 ROS、厂商工作区、仓库 `.venv` 和仓库 `install`。可在与启动脚本相同的用户及环境中检查实际解析结果，例如在 root shell 中：

```bash
cd /home/ubuntu/Dex-RL
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/xos/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 pkg prefix bodyctrl_msgs
python -c 'import bodyctrl_msgs; print(bodyctrl_msgs.__file__)'
ros2 interface show bodyctrl_msgs/msg/MotorStatusMsg
ros2 interface show bodyctrl_msgs/msg/CmdMotorCtrl
```

核对包路径及接口内容是否符合现场预期；如有多个安装位置，不要仅凭源码目录判断运行时使用的是哪一份。现场厂商版本可能变化，必要时再比对其余消息定义。

### 切换控制权

本次机器上的厂商运控与 `body_control` 争用同一个 EtherCAT 主站，不能同时控制。测试记录显示 `proc_manager` 还可能重新拉起相关进程，因此仅结束运控进程不足以完成切换。停止该服务还会影响其管理的电源板监控、云端服务、音频和 OTA；长期运行需要另行安排这些功能。

按现场流程停止 `proc_manager`，确认其已停止且原控制进程已退出，再启动 Dex-RL。测试中服务曾停留在 `deactivating`，并出现机器人失去力矩的情况；不要把停止命令已发出视为控制权已释放。

本次测试遇到服务卡住时，记录使用了 `sudo systemctl kill -s SIGKILL proc_manager`，之后单元变为 `inactive`，同时出现过辅助进程发送信号失败的提示。强制结束服务会影响其管理的进程，应在确认现场状态和影响后按厂商流程处理，并重新核对单元和残留进程状态。

### 真机测试脚本

测试包中的两个脚本已保留在仓库中：

- [`scripts/start_real_x86.sh`](../scripts/start_real_x86.sh)：以 root 分步启动 `body_control`、`joystick` 和 `rl_control`，默认加载本机 Jazzy、`/home/ubuntu/xos`、仓库 `.venv` 和 `install` 环境。
- [`scripts/diag_real.py`](../scripts/diag_real.py)：订阅一组电机命令和反馈及手柄话题，打印位置误差、电流和温度。

**脚本保留测试包的运行逻辑，已补充注意事项。** 启动脚本依赖固定路径和等待时间，会尝试清理旧进程；其日志检查不能单独证明所有节点与反馈正常。诊断脚本在缺少数据时仍可能以成功退出，且只比较两边都收到的电机 ID。使用时需同时核对进程状态、话题数据和日志。

本次测试将 `body_control`、`joystick` 和 `rl_control` 统一以 root 运行：`body_control` 需要访问 EtherCAT 原始套接字，手柄串口也有权限要求；测试中不同用户运行节点还出现过同机通信问题。诊断时也应核对用户、ROS 环境和 DDS 配置，避免将权限或通信问题误判为控制器故障。

## 手柄与状态机

| 操作 | 状态或行为 |
| --- | --- |
| `C` | 请求 `STOP`；进入时锁存当前实测位置并保持 |
| `D` | 请求 `ZERO`；向配置的准备姿态插值 |
| 先 `D`，再按 `A` | 请求 `WALKAMP`；进入站走一体策略 |
| 摇杆 | 输入速度指令 |

当前手柄逻辑要求先发出 `gotoZERO`，`A` 才能切换到 `gotoWALKAMP`。`ZERO` 是配置的蹲姿准备位，不是直立姿态。`WALKAMP` 在零速度指令下可以保持站立；需要速度指令才能观察行走。`STOP` 锁存进入时的实测位置，若机身已下沉，会保持该时刻的姿态。

## 联调问题与观察

以下现象和处理均来自本次测试记录，排查时应结合当前机器的日志和状态：

- **EtherCAT 初始化失败、状态为零**：普通用户启动 `body_control` 时曾出现网卡原始套接字权限错误。本次以 root 启动解决；尝试给 ROS `component_container` 设置 capability 时，动态链接器未加载所需的厂商库，因此该尝试没有解决问题。
- **手柄无数据**：测试中普通用户无 `/dev/ttySBUS0` 权限，`/sbus_data` 无发布者。需核对串口映射、设备权限和实际运行用户。
- **状态话题可见但收不到数据**：测试中不同用户启动节点时出现该现象；还需结合 DDS、QoS 和实际消息检查，不能仅凭话题有发布者判断数据正常。
- **命令话题检查**：测试环境中 `ros2 topic echo` / `hz` 曾无法观察到命令，即使话题显示有发布者。可使用诊断脚本辅助检查，同时核对其输出是否覆盖所需电机和新鲜数据。
- **残留进程**：root 启动的进程无法由普通用户结束。测试中还遇到 `body_control` 未按预期响应退出信号、关闭时产生 `SIGABRT`；应先确认具体进程及状态，再按现场流程处理。避免用宽泛的 `pkill -f` 匹配远程命令行，以免误杀 SSH 会话。
- **切换到 WALKAMP 的瞬时现象**：测试记录中曾出现一批踝关节 IK 求解失败日志，49 条后停止增长，随后记录的跟随误差小于 0.04 rad；髋部电流也曾瞬时超过 20 A，随后回落。吊挂状态下还观察到左右腿姿态不对称。这些是该次测试的观察，不应单独作为其他工况的正常判据；需持续关注日志、位置误差和电流。

## 检查与恢复

启动后，结合 `body_control` 日志、节点存活情况、四组状态数据（腿、臂、腰和 IMU）以及命令和反馈，确认系统持续正常工作；不要仅凭 `All devices ready`、话题发布者数量或单次诊断结果判断。测试记录中站立时曾观察到约 0.00–0.04 rad 的位置误差和 ±6 A 内的电流，具体数值应结合状态、负载和硬件判断。

恢复厂商运控前，先按现场流程停止 Dex-RL 控制节点、joystick 和 `body_control`，核对残留进程并确认 EtherCAT 控制权已释放，再启动 `proc_manager`。测试记录称恢复后还需按手柄 `A` 或通过 APP 指令启动 `robot_control`；具体操作以现场厂商系统为准。
