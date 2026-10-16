"""Run the top-level harness with deterministic ROS/thread stand-ins, without DDS."""
import argparse
import ast
import json
from pathlib import Path
import queue
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def run_harness(tmp_path, monkeypatch):
    def run(mode='advancing', fail_control=False):
        events = []
        wall = [3.0]  # constructor snapshot is already three seconds old
        state = ['STOP']
        running = [False]
        control_running = [False]
        stamps = {group: 0.0 for group in ['leg', 'arm', 'waist', 'imu']}
        if mode == 'fresh_frozen':
            stamps = dict.fromkeys(stamps, wall[0])
        messages = {}
        received = {}
        for group in stamps:
            # PeekableQueue raises Empty between a callback's get and put.
            def peek(group=group):
                if mode == 'missing' and group == 'imu':
                    raise queue.Empty()
                value = stamps[group]
                sec = int(value)
                return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
                    sec=sec, nanosec=round((value - sec) * 1e9))))
            messages[group] = SimpleNamespace(peek=peek)
            if mode != 'missing' or group != 'imu':
                received[group] = wall[0]
        interface = SimpleNamespace(last_received=received, **{
            name: messages[group] for name, group in [
                ('queue_leg_motor_state', 'leg'), ('queue_arm_motor_state', 'arm'),
                ('queue_waist_motor_state', 'waist'), ('queue_imu_xsens', 'imu')]})
        sim = SimpleNamespace(
            data=SimpleNamespace(time=0.0, qpos=np.array([0.0, 0.0, 0.995])),
            paused=False, simulation_error=None, last_raw_joint_commands={})
        sim.stop_event = SimpleNamespace(set=lambda: running.__setitem__(0, False))
        sim.start_publishing = lambda: events.append('publish')
        sim.stop_publishing = lambda: events.append('stop_publish')
        def start_simulation():
            events.append('simulate')
            running[0] = True
        sim.start = start_simulation
        flag = SimpleNamespace(fsm_state_command=None)
        controller = SimpleNamespace(
            robot_interface=interface, control_flag=flag, control_error=None,
            motor_num=29, initial_feedback_timeout=0.3,
            robot_fsm=SimpleNamespace(get_current_state=lambda: SimpleNamespace(name=state[0])))
        def start_control():
            events.append('control')
            events.append(('physics_at_control_start', sim.data.time))
            control_running[0] = True
            if fail_control:
                controller.control_error = 'initial control failed'
            elif wall[0] - stamps['imu'] > 0.5:
                controller.control_error = 'imu sensor sample is older than 0.5 s'
        controller._start_control_thread = start_control
        controller.destroy_node = lambda: events.append('destroy_control')
        node = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(wall[0]*1e9))),
            destroy_node=lambda: events.append('destroy_sim'))
        executor = SimpleNamespace(add_node=lambda node: None, spin=lambda: None,
                                   shutdown=lambda timeout_sec: events.append('shutdown_executor'))
        def sleep(dt):
            wall[0] += dt
            if running[0]:
                if mode in ['advancing', 'future']:
                    for group in stamps:
                        stamps[group] = wall[0] + (1.0 if mode == 'future' else 0.0)
                        received[group] = wall[0]
                if control_running[0] and not controller.control_error:
                    state[0] = {'gotoZERO': 'ZERO', 'gotoWALKAMP': 'WALKAMP'}.get(
                        flag.fsm_state_command, state[0])
                    if state[0] != 'STOP':
                        sim.last_raw_joint_commands = {i: {'kp': 10, 'kd': 1} for i in range(29)}
                if not sim.paused:
                    if 'physics' not in events:
                        events.append('physics')
                        events.append(('state_at_first_step', state[0]))
                    sim.data.time += dt
                    sim.data.qpos[0] += dt * getattr(flag, 'x_speed_command', 0)
                    if sim.data.time >= 1.2:
                        running[0] = False
        class Thread:
            def __init__(self, target, **kwargs):
                self.target = target
            def start(self):
                if self.target == sim.start:
                    self.target()
            def is_alive(self):
                return running[0]
            def join(self, timeout):
                events.append('join')
        output = tmp_path / 'result.json'
        monkeypatch.setattr(sys, 'argv', ['ros_closed_loop.py', '--duration', '1.2', '--output', str(output)])
        path = ROOT / 'scripts/ros_closed_loop.py'
        # Only imports are replaced; execute the actual startup, checks, report and cleanup.
        tree = ast.parse(path.read_text())
        tree.body = [stmt for stmt in tree.body if not isinstance(stmt, (ast.Import, ast.ImportFrom))]
        namespace = dict(__file__=str(path), argparse=argparse, json=json, Path=Path, queue=queue,
                         os=SimpleNamespace(environ={}), sys=SimpleNamespace(path=[]), np=np,
                         threading=SimpleNamespace(Thread=Thread),
                         time=SimpleNamespace(monotonic=lambda: wall[0], sleep=sleep),
                         rclpy=SimpleNamespace(init=lambda: None, create_node=lambda name: node,
                                               shutdown=lambda: events.append('shutdown_ros')),
                         SingleThreadedExecutor=lambda: executor,
                         RobotSimulator=lambda *args, **kwargs: sim,
                         XMIGCSControlNode=lambda **kwargs: controller)
        exit_code = 0
        try:
            exec(compile(tree, str(path), 'exec'), namespace)
        except SystemExit as error:
            exit_code = error.code
        return json.loads(output.read_text()), events, exit_code
    return run


@pytest.mark.parametrize('mode', ['stale', 'fresh_frozen', 'missing', 'future'])
def test_harness_rejects_unready_feedback_and_cleans_up(run_harness, mode):
    report, events, status = run_harness(mode)
    assert status == 1
    assert 'control' not in events
    assert report['validation_error']
    assert 'imu' in report['startup_feedback_age_seconds']
    assert 'physics' not in events
    assert all(event in events for event in ['stop_publish', 'destroy_control', 'shutdown_executor',
                                             'destroy_sim', 'shutdown_ros'])


def test_harness_samples_before_control_and_zero_before_physics(run_harness):
    report, events, status = run_harness()
    assert status == 0
    assert events.index('simulate') < events.index('control') < events.index('physics')
    assert ('physics_at_control_start', 0.0) in events
    assert ('state_at_first_step', 'ZERO') in events
    assert report['controller_error'] is None and report['validation_error'] is None
    assert 'WALKAMP' in report['states']
    assert report['simulation_seconds'] >= 1.2
    assert report['forward_displacement'] > 0
    assert all(0 <= age < 0.1 for age in report['startup_feedback_age_seconds'].values())


def test_harness_keeps_physics_paused_after_control_start_failure(run_harness):
    report, events, status = run_harness(fail_control=True)
    assert status == 1
    assert report['controller_error'] == 'initial control failed'
    assert report['validation_error']
    assert 'physics' not in events
    assert 'shutdown_ros' in events
