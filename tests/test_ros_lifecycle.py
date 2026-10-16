from pathlib import Path
import importlib
import signal
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sim_joy'))


@pytest.mark.parametrize('topic', ['leg/cmd_ctrl', '/sbus_data'])
def test_simulator_initializes_before_processing_early_commands(monkeypatch, topic):
    import simulator_view_asyn as sim
    callbacks = {}
    events = []
    stamp = SimpleNamespace(sec=0, nanosec=0)
    axes = [0.0] * 12
    axes[8] = 1.0
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp),
                              cmds=[SimpleNamespace(name=51)], axes=axes)
    def subscribe(msg_type, name, callback, qos):
        callbacks[name] = callback
        if 'spin' in events and name == topic:
            callback(message)
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp)),
        create_subscription=subscribe,
        create_publisher=lambda *args: SimpleNamespace(publish=lambda msg: None),
        create_timer=lambda *args: None,
        destroy_node=lambda: events.append('destroy'))
    def start_spin():
        events.append('spin')
        if topic in callbacks:
            callbacks[topic](message)
    monkeypatch.setattr(sim.rclpy, 'init', lambda: None)
    monkeypatch.setattr(sim.rclpy, 'create_node', lambda name: node)
    monkeypatch.setattr(sim.rclpy, 'shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(sim.rclpy, 'try_shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(sim.threading, 'Thread', lambda **kwargs: SimpleNamespace(
        start=start_spin, join=lambda timeout: events.append('join')))
    monkeypatch.setattr(sim.RobotSimulator, 'start', lambda self: events.append('simulate'))
    monkeypatch.setattr(sys, 'argv', ['simulator_view_asyn.py', '--headless'])
    sim.main()
    assert events.index('spin') < events.index('simulate')
    assert 'destroy' in events and 'shutdown' in events
    if topic == 'leg/cmd_ctrl':
        assert callbacks[topic].__self__.raw_joint_commands.qsize() == 1
    else:
        assert callbacks[topic].__self__.pre_key == 'C'


def test_simulator_constructor_failure_cleans_up_ros(monkeypatch):
    import simulator_view_asyn as sim
    events = []
    def fail_constructor(*args, **kwargs):
        raise ValueError('model initialization failed')
    monkeypatch.setattr(sim.rclpy, 'init', lambda: None)
    monkeypatch.setattr(sim.rclpy, 'create_node', lambda name: SimpleNamespace(
        destroy_node=lambda: events.append('destroy')))
    monkeypatch.setattr(sim.rclpy, 'shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(sim.rclpy, 'try_shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(sim.threading, 'Thread', lambda **kwargs: SimpleNamespace(
        start=lambda: events.append('spin'), join=lambda timeout: None))
    monkeypatch.setattr(sim, 'RobotSimulator', fail_constructor)
    monkeypatch.setattr(sys, 'argv', ['simulator_view_asyn.py', '--headless'])
    with pytest.raises(ValueError, match='model initialization failed'):
        sim.main()
    assert 'spin' not in events
    assert 'shutdown' in events and 'destroy' in events


@pytest.mark.parametrize('failure,context_alive', [('external', False), ('runtime', False), ('runtime', True)])
@pytest.mark.parametrize('entry', ['rl_control_node', 'rl_control_node_sim'])
def test_control_main_handles_shutdown_before_spin(monkeypatch, failure, context_alive, entry):
    from rclpy.executors import ExternalShutdownException
    control = importlib.import_module(entry)
    events = []
    def stopped_spin():
        if failure == 'runtime':
            raise RuntimeError('ROS wait-set context is invalid')
        raise ExternalShutdownException()
    executor = SimpleNamespace(add_node=lambda node: events.append('add'), spin=stopped_spin,
                               shutdown=lambda timeout_sec: events.append('executor'))
    monkeypatch.setattr(control, 'SingleThreadedExecutor', lambda: executor, raising=False)
    monkeypatch.setattr(control.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(control.rclpy, 'get_global_executor', lambda: None)
    monkeypatch.setattr(control.rclpy, 'ok', lambda: context_alive)
    monkeypatch.setattr(control.rclpy, 'try_shutdown', lambda: events.append('shutdown'))
    constructor = 'XMIGCSControlNode_sim' if entry == 'rl_control_node_sim' else 'XMIGCSControlNode'
    monkeypatch.setattr(control, constructor, lambda **kwargs: SimpleNamespace(
        destroy_node=lambda: events.append('destroy')))
    if context_alive:
        with pytest.raises(RuntimeError, match='context is invalid'):
            control.main()
    else:
        control.main()
    assert events == ['add', 'executor', 'destroy', 'shutdown']


@pytest.mark.parametrize('simulated,configured,expected', [
    (True, None, 20.0), (True, 12.0, 12.0), (False, None, 5.0), (False, 20.0, 5.0),
])
def test_initial_feedback_timeout_configuration(tmp_path, simulated, configured, expected):
    from rl_control_node import XMIGCSControlNode
    config = {'motor_num': 29, 'dt': 0.01, 'sim': simulated, 'control_tool': 'manual'}
    if configured is not None:
        config['initial_feedback_timeout'] = configured
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(config))
    node = SimpleNamespace(sim_override=simulated, control_tool_override=None)
    node.declare_parameter = lambda *args: None
    node.get_parameter = lambda name: SimpleNamespace(get_parameter_value=lambda: SimpleNamespace(string_value=str(path)))
    XMIGCSControlNode._load_config(node)
    assert node.initial_feedback_timeout == expected
    assert yaml.safe_load((ROOT / 'Deploy_Tienkung/config/dex_config_sim.yaml').read_text())['initial_feedback_timeout'] == 20.0


@pytest.mark.parametrize('feedback_arrives', [True, False])
def test_initial_wait_accepts_slow_start_and_still_times_out(monkeypatch, feedback_arrives):
    import rl_control_node as control
    clock = [0.0]
    commands = []
    node = SimpleNamespace(control_running=True, initial_feedback_timeout=20.0, control_error=None)
    node.get_logger = lambda: SimpleNamespace(info=lambda msg: None, error=lambda msg: None)
    node._process_controller_data = lambda: None
    node.control_flag = SimpleNamespace()
    node.robot_data = SimpleNamespace(**{name: np.ones(1) for name in [
        'joint_kp_p_', 'joint_kd_p_', 'q_dot_d_', 'tau_d_']})
    node.robot_interface = SimpleNamespace(publish_motor_commands=lambda flag: commands.append(
        (node.robot_data.joint_kp_p_[0], node.robot_data.joint_kd_p_[0])))
    def update(flag, elapsed):
        if feedback_arrives and clock[0] >= 6.7:
            node.control_running = False
            return
        raise TimeoutError('no initial feedback')
    node._update_robot_data = update
    monkeypatch.setattr(control.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(control.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(control.time, 'sleep', lambda dt: clock.__setitem__(0, clock[0] + dt))
    control.XMIGCSControlNode._rl_control_loop(node)
    if feedback_arrives:
        assert node.control_error is None and not commands
        assert clock[0] >= 6.7
    else:
        assert node.control_error == 'Initial robot feedback timed out'
        assert clock[0] == pytest.approx(20.0, abs=0.02)
        assert commands == [(0.0, 1.0)]


def test_simulation_stops_when_ros_context_closes(monkeypatch):
    import simulator_view_asyn as sim
    simulator = sim.RobotSimulator(ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml',
                                   headless=True, band=False)
    stamp = SimpleNamespace(sec=0, nanosec=0)
    simulator.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: stamp)))
    def forbid_step(*args):
        raise AssertionError('physics must stop after ROS shutdown')
    monkeypatch.setattr(sim.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(sim.mujoco, 'mj_step', forbid_step)
    simulator.simulate_thread()
    assert simulator.simulation_error is None


@pytest.mark.parametrize('failure', [False, True])
def test_spin_exit_stops_simulation_and_reports_callback_failure(monkeypatch, failure):
    import simulator_view_asyn as sim
    simulator = SimpleNamespace(stop_event=threading.Event(), simulation_error=None)
    def fail_spin(*args, **kwargs):
        raise ValueError('command callback failed')
    monkeypatch.setattr(sim.rclpy, 'ok', lambda: failure)
    monkeypatch.setattr(sim.rclpy, 'spin_once', fail_spin)
    sim._spin_wrapper(SimpleNamespace(), simulator)
    assert simulator.stop_event.is_set()
    assert bool(simulator.simulation_error) is failure


def test_gui_close_stops_executor_and_destroys_window(monkeypatch):
    from sim_joy import gui_monitor as gui
    app = gui.TeleopGuiApp.__new__(gui.TeleopGuiApp)
    app._closed = False
    events = []
    app.executor = SimpleNamespace(shutdown=lambda timeout_sec: events.append('executor'))
    app._spin_thread = SimpleNamespace(join=lambda timeout: events.append('join'))
    app.teleop_node = SimpleNamespace(destroy_node=lambda: events.append('teleop'))
    app.monitor_node = SimpleNamespace(destroy_node=lambda: events.append('monitor'))
    app.root = SimpleNamespace(quit=lambda: events.append('quit'), destroy=lambda: events.append('destroy'))
    monkeypatch.setattr(gui.rclpy, 'try_shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(gui.rclpy, 'shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(gui.rclpy, 'ok', lambda: True)
    app.close()
    app.close()
    assert events == ['executor', 'join', 'teleop', 'monitor', 'shutdown', 'quit', 'destroy']


@pytest.mark.parametrize('closed', [False, True])
def test_gui_stops_refreshing_after_shutdown(monkeypatch, closed):
    from sim_joy import gui_monitor as gui
    app = gui.TeleopGuiApp.__new__(gui.TeleopGuiApp)
    app._closed = closed
    events = []
    app.close = lambda: events.append('close')
    app._refresh = lambda: events.append('refresh')
    app.root = SimpleNamespace(after=lambda *args: events.append('after'))
    monkeypatch.setattr(gui.rclpy, 'ok', lambda: False)
    app._schedule_update()
    assert events == ([] if closed else ['close'])


@pytest.mark.parametrize('termination_signal', [signal.SIGINT, signal.SIGTERM])
def test_gui_registers_both_termination_signals(monkeypatch, termination_signal):
    from sim_joy import gui_monitor as gui
    handlers = {}
    node = SimpleNamespace()
    monkeypatch.setattr(gui.rclpy, 'init', lambda: None)
    monkeypatch.setattr(gui, 'TeleopJoyNode', lambda **kwargs: node)
    monkeypatch.setattr(gui, 'TeleopGuiMonitorNode', lambda: node)
    monkeypatch.setattr(gui, 'MultiThreadedExecutor', lambda: SimpleNamespace(add_node=lambda node: None, spin=lambda: None))
    monkeypatch.setattr(gui.threading, 'Thread', lambda **kwargs: SimpleNamespace(start=lambda: None))
    monkeypatch.setattr(gui.tk, 'Tk', lambda: SimpleNamespace(title=lambda text: None, protocol=lambda *args: None))
    monkeypatch.setattr(gui.signal, 'signal', lambda key, callback: handlers.__setitem__(key, callback))
    monkeypatch.setattr(gui.TeleopGuiApp, '_build_layout', lambda self: None)
    monkeypatch.setattr(gui.TeleopGuiApp, '_schedule_update', lambda self: None)
    events = []
    monkeypatch.setattr(gui.TeleopGuiApp, 'close', lambda self: events.append('close'))
    gui.TeleopGuiApp()
    assert termination_signal in handlers
    handlers[termination_signal](termination_signal, None)
    assert events == ['close']
