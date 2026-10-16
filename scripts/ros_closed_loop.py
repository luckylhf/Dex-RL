"""Exercise the real ROS2 deploy node and xSIM topics on localhost, with no robot."""
import argparse
import json
import os
from pathlib import Path
import queue
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'Deploy_Tienkung'))
sys.path.insert(0, str(ROOT / 'xSIM_MUJOCO-main/scripts'))
os.environ.setdefault('ROS_DOMAIN_ID', '137')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.setdefault('ROS_LOG_DIR', str(ROOT / 'artifacts/ros_logs'))

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rl_control_node import XMIGCSControlNode
from simulator_view_asyn import RobotSimulator

parser = argparse.ArgumentParser()
parser.add_argument('--duration', type=float, default=12)
parser.add_argument('--policy', type=Path, help='Explicit candidate ONNX; does not overwrite the default model')
parser.add_argument('--band', action='store_true', help='Transport-only test with external support; not a gait acceptance test')
parser.add_argument('--fault', choices=['feedback', 'inference'], help='Inject one failure and verify damping-only stop')
parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/ros_closed_loop.json')
args = parser.parse_args()
rclpy.init()
sim_node = rclpy.create_node('dex_sim_validation')
simulator = RobotSimulator(ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml', sim_node,
                           headless=True, band=args.band, duration=args.duration)
controller = XMIGCSControlNode(sim=True, control_tool='manual', start_thread=False)
if args.policy:
    from FSM.fsm_base import FSMStateName
    policy_state = controller.robot_fsm.state_objects[FSMStateName.WALKAMP]
    policy_state.model_path = str(args.policy.resolve())
    policy_state._init_onnx_session()
executor = SingleThreadedExecutor()
executor.add_node(sim_node)
executor.add_node(controller)
spin = threading.Thread(target=executor.spin, daemon=True)
spin.start()
simulation = threading.Thread(target=simulator.start)
heights = [float(simulator.data.qpos[2])]
initial_position = simulator.data.qpos[:3].copy()
states = []
fault_injected = False
validation_error = None
interface = controller.robot_interface
queues = {'leg': interface.queue_leg_motor_state, 'arm': interface.queue_arm_motor_state,
          'waist': interface.queue_waist_motor_state, 'imu': interface.queue_imu_xsens}
feedback_ages = dict.fromkeys(queues)
try:
    # Paused physics still refreshes sensor stamps; hold the pose until ZERO commands arrive.
    simulator.paused = True
    sample_after_ns = sim_node.get_clock().now().nanoseconds
    simulation.start()
    deadline = time.monotonic() + controller.initial_feedback_timeout
    while True:
        now_ns = sim_node.get_clock().now().nanoseconds
        fresh = set(interface.last_received) == set(queues)
        for group, messages in queues.items():
            try:
                stamp = messages.peek().header.stamp
            except queue.Empty:
                feedback_ages[group] = None
                fresh = False
                continue
            stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
            feedback_ages[group] = (now_ns - stamp_ns) / 1e9
            fresh = fresh and stamp_ns > sample_after_ns and 0 <= feedback_ages[group] < 0.1
        if fresh:
            break
        if not simulation.is_alive() or time.monotonic() > deadline:
            raise TimeoutError(f'Fresh ROS2 feedback not ready; sample ages: {feedback_ages}')
        time.sleep(0.01)
    controller.control_flag.fsm_state_command = 'gotoZERO'
    controller._start_control_thread()
    while (controller.robot_fsm.get_current_state().name != 'ZERO'
           or len(simulator.last_raw_joint_commands) != controller.motor_num):
        if controller.control_error or not simulation.is_alive() or time.monotonic() > deadline:
            raise TimeoutError(f'ZERO motor commands not ready: {controller.control_error}')
        time.sleep(0.01)
    if controller.control_error:
        raise RuntimeError(controller.control_error)
    started = time.monotonic()
    simulator.paused = False
    while simulation.is_alive():
        t = float(simulator.data.time)
        if t >= 0.5:
            controller.control_flag.fsm_state_command = 'gotoWALKAMP'
        controller.control_flag.x_speed_command = 0.4 if args.duration/3 <= t < args.duration*2/3 else 0.0
        if args.fault and t >= 1.0 and not fault_injected:
            fault_injected = True
            if args.fault == 'feedback':
                simulator.stop_publishing()
            else:
                from FSM.fsm_base import FSMStateName
                controller.robot_fsm.state_objects[FSMStateName.WALKAMP].ort_session_ = None
        heights.append(float(simulator.data.qpos[2]))
        states.append(controller.robot_fsm.get_current_state().name)
        if time.monotonic() - started > args.duration*3 + 10:
            raise TimeoutError('ROS2 closed loop exceeded its deadline')
        if controller.control_error:
            time.sleep(0.1)  # allow the real ROS damping command to reach xSIM
            break
        time.sleep(0.01)
except (TimeoutError, RuntimeError) as error:
    validation_error = str(error)
finally:
    simulator.stop_event.set()
    simulator.stop_publishing()
    simulation.join(timeout=2)
    controller.destroy_node()
    executor.shutdown(timeout_sec=2)
    spin.join(timeout=2)
    sim_node.destroy_node()
    rclpy.shutdown()
report = {'policy': str(args.policy) if args.policy else 'configured default ONNX', 'simulation_seconds': float(simulator.data.time), 'min_pelvis_height': min(heights),
          'states': sorted(set(states)), 'controller_error': controller.control_error,
          'validation_error': validation_error, 'startup_feedback_age_seconds': feedback_ages,
          'initial_position': initial_position.tolist(), 'final_position': simulator.data.qpos[:3].tolist(),
          'forward_displacement': float(simulator.data.qpos[0] - initial_position[0]),
          'simulator_error': simulator.simulation_error, 'external_support': args.band,
          'collapsed': min(heights) < 0.5,
          'ros_feedback_groups': sorted(controller.robot_interface.last_received),
          'received_motor_commands': len(simulator.last_raw_joint_commands)}
report['fault'] = args.fault
if args.fault:
    commands = list(simulator.last_raw_joint_commands.values())
    report['damping_stop_received'] = bool(commands) and all(c['kp'] == 0 and c['kd'] == 1 for c in commands)
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
if args.fault:
    if report['validation_error'] or not report['controller_error'] or not report['damping_stop_received']:
        raise SystemExit(1)
elif report['validation_error'] or report['controller_error'] or report['simulator_error'] or report['collapsed'] or 'WALKAMP' not in states:
    raise SystemExit(1)

