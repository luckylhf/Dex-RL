# MuJoCo ROS 仿真节点

本目录是 `mujoco_node` ROS 2 包，使用 `resources/dex_v3/urdf/dex_v3.xml` 模型，与 `rl_control` 通过 ROS 消息进行闭环仿真。模型包含 29 路关节执行器；公开 ROS 入口使用 `dex_v3` 和 `full` 配置。

环境准备、构建及整栈启动请参阅[项目 README](../README.md#-ros2--mujoco-闭环)。构建并加载工作区后，可单独启动仿真节点：

```bash
ros2 launch mujoco_node mujoco_sim.launch.py
```

单独启动仿真节点不会自动启动控制节点或手柄。需要无界面闭环报告时，可使用项目根目录的 `scripts/ros_closed_loop.py`；独立策略 Sim2Sim 则使用 `TienKung-Lab/legged_lab/scripts/sim2sim.py`。两者用途和模型控制方式不同，详见项目 README。

模型网格通过软链接引用 `TienKung-Lab/legged_lab/assets/Tienkung_Dex/`；复制或打包时需保留链接及目标。仿真结果不能替代真机运动验收。

主要文件：`scripts/simulator_view_asyn.py` 为仿真节点，`scripts/elastic_band.py` 为外部支撑模型，`launch/mujoco_sim.launch.py` 为 ROS 启动入口。

许可证见 [LICENSE](LICENSE)。
