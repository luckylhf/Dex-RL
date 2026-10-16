from pathlib import Path

import mujoco
import yaml
from bodyctrl_msgs.msg import MotorCtrl

ROOT = Path(__file__).resolve().parents[1]


def test_stop_gains_do_not_excite_fast_joint_oscillation():
    from simulator_view_asyn import RobotSimulator
    simulator = RobotSimulator(ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml',
                               headless=True, band=False)
    # Isolate numerical PD response from gravity/contact and excite it with a joint perturbation.
    simulator.model.opt.gravity[:] = 0
    simulator.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    cfg = yaml.safe_load((ROOT / 'Deploy_Tienkung/policy/stop/config/stop.yaml').read_text())
    ids = [51, 52, 53, 54, 55, 56, 61, 62, 63, 64, 65, 66, 33, 32, 31,
           11, 12, 13, 14, 15, 16, 17, 21, 22, 23, 24, 25, 26, 27]
    for i, motor in enumerate(ids):
        name = simulator.motor_id_to_joint[motor]
        status = simulator.get_status(name)
        msg = MotorCtrl(name=motor, pos=status['position'], spd=0.0, tor=0.0,
                        kp=float(cfg['kp_pos'][i]), kd=float(cfg['kd_pos'][i]))
        simulator.raw_joint_commands.put(msg)
    joint = simulator.model.joint('knee_pitch_l_joint')
    target = float(simulator.data.qpos[joint.qposadr[0]])
    simulator.data.qpos[joint.qposadr[0]] += 0.05
    peak_velocity = 0.0
    for _ in range(200):
        simulator.set_motor_cmd()
        mujoco.mj_step(simulator.model, simulator.data)
        peak_velocity = max(peak_velocity, float(abs(simulator.data.qvel[6:]).max()))
    assert peak_velocity < 3.0
    assert abs(simulator.data.qpos[joint.qposadr[0]] - target) < 0.001


def test_ros_command_pd_force_and_saturation():
    import pytest
    from simulator_view_asyn import RobotSimulator
    simulator = RobotSimulator(ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml',
                               headless=True, band=False)
    joint = simulator.model.joint('knee_pitch_l_joint')
    motor = simulator.model.actuator('motor_knee_pitch_l_joint')
    simulator.data.qpos[joint.qposadr[0]] = 0.3
    simulator.data.qvel[joint.dofadr[0]] = 0.2
    simulator.raw_joint_commands.put(MotorCtrl(name=54, pos=0.4, spd=0.0, tor=2.0, kp=100.0, kd=5.0))
    simulator.set_motor_cmd()
    mujoco.mj_forward(simulator.model, simulator.data)
    assert simulator.data.actuator_force[motor.id] == pytest.approx(11, abs=1e-5)
    simulator.raw_joint_commands.put(MotorCtrl(name=54, pos=10.0, spd=0.0, tor=0.0, kp=1000.0, kd=5.0))
    simulator.set_motor_cmd()
    mujoco.mj_forward(simulator.model, simulator.data)
    assert simulator.data.actuator_force[motor.id] == pytest.approx(400)


def test_expired_motor_command_becomes_damping_only():
    import time
    import pytest
    from simulator_view_asyn import RobotSimulator
    simulator = RobotSimulator(ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml',
                               headless=True, band=False)
    joint = simulator.model.joint('knee_pitch_l_joint')
    motor = simulator.model.actuator('motor_knee_pitch_l_joint')
    simulator.data.qvel[joint.dofadr[0]] = 0.2
    simulator.last_raw_joint_commands['knee_pitch_l_joint'] = {
        'pos': 10.0, 'spd': 0.0, 'tor': 100.0, 'kp': 1000.0, 'kd': 5.0,
        'timestamp': time.monotonic()-1.0,
    }
    simulator.set_motor_cmd()
    mujoco.mj_forward(simulator.model, simulator.data)
    assert simulator.data.actuator_force[motor.id] == pytest.approx(-0.2)

