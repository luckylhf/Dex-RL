from threading import Event, Lock
from types import SimpleNamespace

import numpy as np
import pytest


@pytest.mark.parametrize('loop', ['physics', 'publish'])
@pytest.mark.parametrize('first_work_seconds', [0.0004, 0.006])
def test_simulator_sleep_overshoot_does_not_accumulate(monkeypatch, loop, first_work_seconds):
    import simulator_view_asyn as sim
    clock = [0.0]
    starts = []
    stopped = Event()
    def sleep(seconds):
        assert seconds > 0
        clock[0] += seconds + 0.0001
    monkeypatch.setattr(sim, 'time', SimpleNamespace(perf_counter=lambda: clock[0], sleep=sleep))
    monkeypatch.setattr(sim.rclpy, 'ok', lambda: True)
    def work(*args):
        clock[0] += first_work_seconds if len(starts) == 1 else 0.0004
        if len(starts) == 5:
            stopped.set()
    node = SimpleNamespace(stop_event=stopped)
    if loop == 'publish':
        node.pub_thread_running = True
        node.sensor_data_lock = Lock()
        node.last_sensor_data = {}
        node.arm_joint_names = node.waist_joint_names = node.leg_joint_names = []
        node.arm_status_pub = node.waist_status_pub = node.leg_status_pub = node.imu_status_pub = None
        node._publish_joint_group = lambda *args: None
        def publish(*args):
            starts.append(clock[0])
            work()
        node._publish_imu_status = publish
        sim.RobotSimulator._publish_thread(node)
    else:
        node.node = node.viewer = None
        node.model = SimpleNamespace(opt=SimpleNamespace(timestep=0.0025))
        node.data = SimpleNamespace(time=0.0)
        node.sim_view_lock = Lock()
        node.debug = node.trigger_reset = node.paused = False
        node.simulation_started = True
        node.duration = node.simulation_error = None
        node.add_virtual_force = node.set_motor_cmd = lambda: None
        for name in ['time_history', 'joint_pos_history', 'joint_vel_history', 'joint_torque_history',
                     'imu_orient_history', 'imu_accel_history', 'imu_pos_history']:
            setattr(node, name, [])
        def read():
            starts.append(clock[0])
            return {name: np.zeros(4) for name in ['joint_positions', 'joint_velocities',
                    'joint_torques', 'imu_orientation', 'imu_linear_acceleration', 'imu_position']}
        node.read_sensors = read
        monkeypatch.setattr(sim.mujoco, 'mj_step', work)
        sim.RobotSimulator.simulate_thread(node)
        assert node.simulation_error is None
    # A slow first cycle starts a new schedule; no multi-step catch-up burst.
    first_next = 0.0026 if first_work_seconds < 0.0025 else first_work_seconds
    later_offset = 0.0 if first_work_seconds < 0.0025 else 0.0001
    assert starts == pytest.approx([0.0, first_next,
        first_next+0.0025+later_offset, first_next+0.005+later_offset,
        first_next+0.0075+later_offset], abs=1e-10)
