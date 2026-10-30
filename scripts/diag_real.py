#!/usr/bin/env python3
"""真机诊断：对比 /leg|arm|waist/cmd_ctrl 的下发值与实测反馈。

注意事项：
    - 先加载与控制节点一致的 ROS、厂商工作区、仓库 install 和 Python 环境；
      核对运行用户、消息包来源及 DDS / QoS 配置。跨用户通信可能受影响。
    - 本工具默认只检查 leg，可用 --group 选择 arm 或 waist；未检查 IMU。
      无数据可能由通信、权限或 QoS 等多种原因造成，不应仅凭输出判断节点故障。
    - 仅比较采集期间命令和反馈中均出现的电机 ID；数据可能不完整或并非同一时刻，
      脚本不检查样本新鲜度。请核对电机覆盖范围及数据是否持续更新。
    - 缺少数据时脚本仍可能返回成功；双方没有共同 ID 时 max |err| 可能显示 0。
      0.1 rad 的警告阈值仅为提示，不能代替现场验收或其他状态检查。

为什么不用 `ros2 topic echo`：厂商自建 ROS 的 QoS 组合下 echo 常常收不到
`/leg/cmd_ctrl`（报 "does not appear to be published yet"，而 `topic info`
却显示 Publisher count: 1）。本工具用 rclpy 直接订阅辅助检查，仍需确认实际收到的数据。

用法（需与 rl_control 同一用户，通常 root）：
    sudo -E python3 scripts/diag_real.py --seconds 10
"""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from bodyctrl_msgs.msg import CmdMotorCtrl, MotorStatusMsg

GROUPS = {
    "leg": (CmdMotorCtrl, MotorStatusMsg),
    "arm": (CmdMotorCtrl, MotorStatusMsg),
    "waist": (CmdMotorCtrl, MotorStatusMsg),
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--group", default="leg", choices=sorted(GROUPS))
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("dexrl_diag")

    cmd: dict[int, object] = {}
    act: dict[int, object] = {}
    joy = []

    def on_cmd(msg):
        for c in msg.cmds:
            cmd[c.name] = c

    def on_status(msg):
        for s in msg.status:
            act[s.name] = s

    qos = QoSProfile(depth=10)
    node.create_subscription(
        CmdMotorCtrl, f"/{args.group}/cmd_ctrl", on_cmd, qos)
    node.create_subscription(
        MotorStatusMsg, f"/{args.group}/status", on_status, qos)
    node.create_subscription(Joy, "/sbus_data", lambda m: joy.append(m), 10)

    deadline = time.time() + args.seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if joy:
        ax = joy[-1].axes
        print("joy axes :", [round(x, 2) for x in ax[:8]])
        print("joy btn  :", [round(x, 0) for x in ax[8:12]], "(a, b, c, d)")
    else:
        print("joy      : 无 /sbus_data（手柄未启动或无权限）")

    if not cmd:
        print(f"cmd      : 无 /{args.group}/cmd_ctrl 数据 —— rl_control 没在发指令")
    if not act:
        print(f"status   : 无 /{args.group}/status 数据 —— body_control 没起来或未就绪")

    if cmd and act:
        print()
        print("  id    cmd_pos   act_pos       err     cur    kp     temp")
        worst = 0.0
        for k in sorted(act):
            if k not in cmd:
                continue
            c, a = cmd[k], act[k]
            err = a.pos - c.pos
            worst = max(worst, abs(err))
            print("%4d %10.4f %9.4f %9.4f %7.2f %6.0f %7.1f"
                  % (k, c.pos, a.pos, err, a.current, c.kp, a.temperature))
        print()
        print("max |err| = %.4f rad" % worst)
        if worst > 0.1:
            print("警告：跟随误差偏大，检查机器人是否被卡住或增益不匹配")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
