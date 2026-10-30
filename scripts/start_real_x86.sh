#!/usr/bin/env bash
# 在本体板（x86, node1）上分步启动 Dex-RL 真机栈。
#
# 用法：  sudo bash scripts/start_real_x86.sh
#
# 注意事项：
# - 本脚本用于已完成联调的 x86 本体板环境；启动前确认机器人有可靠支撑，
#   并按现场规程安排急停及控制权切换。停止厂商服务可能使机器人失去力矩。
# - 先确认 proc_manager 已完全停止、厂商运控已退出且 EtherCAT 控制权已释放。
#   脚本只检查服务是否 active，不能识别所有未完成停止或残留进程的情况。
# - 脚本会用 pkill -f 清理匹配的旧进程；远程执行时应留意匹配范围，
#   避免误结束其他进程或 SSH 会话。不要同时运行厂商运控和本脚本的 body_control。
# - 请核对 Jazzy、/home/ubuntu/xos、仓库 .venv 和 install 环境，以及日志目录。
#   默认日志写入 /tmp/bc.log、joy.log 和 rl.log，同名日志会被覆盖。
# - 脚本使用固定等待时间和日志关键字判断启动；提示完成不代表节点及反馈均正常。
#   请另查进程、日志、手柄数据，以及腿、臂、腰和 IMU 的持续新鲜反馈。
# - 中途失败时，前面启动的节点可能仍在运行；检查并按现场流程处理残留进程，
#   确认 EtherCAT 控制权释放后再恢复厂商运控。
#
# 为什么必须 root：body_control 用 SOEM EtherCAT 原始套接字访问 enp4s0，
# 普通用户会报 "No socket connection on netcard"。三个节点都要同用户运行，
# 否则 FastDDS 共享内存不通。
#
# 前置条件：proc_manager 已停止。运控 robot_control + xmigcs 与 body_control
# 互斥地占用 EtherCAT 主站，同跑会互相打架：
#     sudo systemctl stop proc_manager
# 需要回滚运控时：sudo systemctl start proc_manager
set -eo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "[start_real] 需要 root：sudo bash $0" >&2
  exit 1
fi

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${LOG_DIR:-/tmp}"
ROS_HOME_OWNER="ubuntu"

if systemctl is-active --quiet proc_manager; then
  echo "[start_real] 警告：proc_manager 仍在运行，它会和 body_control 抢 EtherCAT 主站" >&2
  echo "[start_real] 请先执行：sudo systemctl stop proc_manager" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /home/ubuntu/xos/setup.bash
# shellcheck disable=SC1091
source "$REPO/.venv/bin/activate"
# shellcheck disable=SC1091
source "$REPO/install/setup.bash"

echo "[start_real] 清理上一轮残留"
pkill -f "component_container --ros-args -r __node:=bodyctrl_nodes" 2>/dev/null || true
pkill -f "install/rl_control/lib" 2>/dev/null || true
pkill -f "joystick_bridge_node" 2>/dev/null || true
pkill -f "ros2 launch joystick joystick.launch.py" 2>/dev/null || true
sleep 2

echo "[start_real] 1/3 body_control（EtherCAT，约 12 s 就绪）"
nohup ros2 launch body_control body_control.launch.py > "$LOG_DIR/bc.log" 2>&1 &
sleep 14
if ! grep -q "All devices ready" "$LOG_DIR/bc.log"; then
  echo "[start_real] body_control 未就绪，最近日志：" >&2
  tail -20 "$LOG_DIR/bc.log" >&2
  exit 1
fi
echo "[start_real]     All devices ready"

echo "[start_real] 2/3 joystick（/sbus_data）"
nohup ros2 launch joystick joystick.launch.py > "$LOG_DIR/joy.log" 2>&1 &
sleep 6
if grep -q "Permission denied" "$LOG_DIR/joy.log"; then
  echo "[start_real] joystick 打不开 /dev/ttySBUS0：" >&2
  grep "Permission denied" "$LOG_DIR/joy.log" >&2
  exit 1
fi
echo "[start_real]     joystick 已启动"

echo "[start_real] 3/3 rl_control（FSM 进入 STOP，锁存当前位姿）"
nohup ros2 run rl_control rl_control_node > "$LOG_DIR/rl.log" 2>&1 &
sleep 8

if ! grep -q "Control stopped" "$LOG_DIR/rl.log"; then
  echo "[start_real] rl_control 运行中，未报错"
else
  echo "[start_real] rl_control 报错：" >&2
  grep "Control stopped" "$LOG_DIR/rl.log" >&2
  exit 1
fi

cat <<EOF

[start_real] 完成。日志：
  $LOG_DIR/bc.log   body_control
  $LOG_DIR/joy.log  joystick
  $LOG_DIR/rl.log   rl_control

手柄状态切换：
  C 键        -> gotoSTOP（锁存当前位姿）
  D 键        -> gotoZERO（回零，进入蹲姿准备位）
  D 后按 A    -> gotoWALKAMP（开始行走）

注意：首次上电务必确认机器人有支撑或有人扶持。
EOF
