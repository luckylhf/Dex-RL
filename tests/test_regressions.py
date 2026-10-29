import ast
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_real_joint_limits_match_source_urdf():
    cfg = yaml.safe_load((ROOT / 'Deploy_Tienkung/config/dex_config_real.yaml').read_text())
    urdf = ET.parse(ROOT / 'TienKung-Lab/legged_lab/assets/Tienkung_Dex/urdf/tiangong3.urdf').getroot()
    joints = {joint.get('name'): joint for joint in urdf.findall('joint')}
    aliases = {'knee': 'knee_pitch', 'elbow': 'elbow_pitch', 'wrist_yaw': 'elbow_yaw'}
    limits = cfg['robot_interface']['joint_limits']
    assert len(limits) == 29
    mismatches = []
    for name, actual in limits.items():
        if name.startswith(('l_', 'r_')):
            side, part = name.split('_', 1)
            joint_name = f'{aliases.get(part, part)}_{side}_joint'
        else:
            joint_name = name + '_joint'
        limit = joints[joint_name].find('limit')
        expected = {'min': float(limit.get('lower')), 'max': float(limit.get('upper'))}
        if actual != expected:
            mismatches.append((name, actual, expected))
    assert not mismatches, mismatches


def test_unknown_state_does_not_crash():
    from common.robot_data import RobotData
    from policy.zero.fsm_zero import FSMStateZero
    from policy.stop.fsm_stop import FSMStateStop
    flag = SimpleNamespace(fsm_state_command='gotoUNKNOWN')
    for state in (FSMStateZero(RobotData()), FSMStateStop(RobotData())):
        assert state.check_transition(flag) is None


def test_warm_start_config_is_read_from_control():
    cfg = yaml.safe_load((ROOT / 'Deploy_Tienkung/policy/walk_amp/config/walk_amp.yaml').read_text())
    assert 'warm_start_time' not in cfg
    assert cfg['control']['warm_start_time'] == 0.3


def test_training_and_deployment_keep_their_paired_gains():
    tree = ast.parse((ROOT / 'TienKung-Lab/legged_lab/assets/Tienkung_Dex/walk.py').read_text())
    call = next(n.value for n in tree.body if isinstance(n, ast.Assign) and
                any(isinstance(a, ast.Name) and a.id == 'DEX_V3_CFG' for a in n.targets))
    actuators = next(k.value for k in call.keywords if k.arg == 'actuators')
    cfg = yaml.safe_load((ROOT / 'Deploy_Tienkung/policy/walk_amp/config/walk_amp.yaml').read_text())
    snapshot = yaml.load((ROOT / 'TienKung-Lab/logs/dex_walk/2026-10-15_19-52-21_v4_walk_4096_20000/params/env.yaml').read_text(), Loader=yaml.BaseLoader)
    import re
    for i, name in enumerate(cfg['joint_names']):
        for field, gain in [('stiffness', 'kp'), ('damping', 'kd')]:
            matches = []
            for actuator in actuators.values:
                values = ast.literal_eval(next(k.value for k in actuator.keywords if k.arg == field))
                for pattern, value in values.items():
                    if re.fullmatch(pattern, name):
                        matches.append(value)
            saved = [float(value) for actuator in snapshot['scene']['robot']['actuators'].values()
                     for pattern, value in actuator[field].items() if re.fullmatch(pattern, name)]
            assert len(matches) == len(saved) == 1, (name, gain, matches, saved)
            assert cfg['gains'][gain][i] == matches[0] == saved[0], (name, gain)


def test_mujoco_matches_training_active_joints_and_mass():
    urdf = ET.parse(ROOT / 'TienKung-Lab/legged_lab/assets/Tienkung_Dex/urdf/tiangong3_ros.urdf').getroot()
    names = {j.get('name') for j in urdf.findall('joint') if j.get('type') != 'fixed'}
    mass = sum(float(l.find('inertial/mass').get('value')) for l in urdf.findall('link'))
    model = mujoco.MjModel.from_xml_path(str(ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml'))
    assert model.nq == 36 and model.nv == 35  # 29 joints plus a floating base
    active = {model.joint(i).name for i in range(model.njnt) if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE}
    assert active == names
    assert float(model.body_mass.sum()) == pytest.approx(mass, abs=5e-6)
    for link in urdf.findall('link'):
        body = model.body(link.get('name')).id
        inertia = link.find('inertial/inertia')
        expected = np.array([[float(inertia.get('ixx')), float(inertia.get('ixy')), float(inertia.get('ixz'))],
                             [float(inertia.get('ixy')), float(inertia.get('iyy')), float(inertia.get('iyz'))],
                             [float(inertia.get('ixz')), float(inertia.get('iyz')), float(inertia.get('izz'))]])
        assert link.find('inertial/origin').get('rpy') == '0 0 0'
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, model.body_iquat[body])
        rotation = rotation.reshape(3, 3)
        # Native URDF import diagonalizes the tensor numerically; compare relative to its magnitude.
        np.testing.assert_allclose(rotation @ np.diag(model.body_inertia[body]) @ rotation.T,
                                   expected, rtol=0, atol=np.abs(expected).max()*1e-6)
        np.testing.assert_allclose(model.body_ipos[body],
                                   np.fromstring(link.find('inertial/origin').get('xyz'), sep=' '), atol=1e-10)
    for joint in urdf.findall('joint'):
        if joint.get('name') not in names:
            continue
        mj_joint = model.joint(joint.get('name'))
        limit = joint.find('limit')
        np.testing.assert_allclose(model.jnt_range[mj_joint.id],
                                   [float(limit.get('lower')), float(limit.get('upper'))], atol=1e-8)


def test_ros_sim_import_does_not_require_ankle_binary():
    import common.robot_interface
    assert callable(common.robot_interface.get_robot_interface)


def test_inference_failure_is_not_silently_ignored():
    from common.robot_data import RobotData
    from policy.walk_amp.fsm_walkamp import FSMStateWALKAMP
    state = FSMStateWALKAMP(RobotData())
    class BrokenSession:
        def get_inputs(self):
            return [SimpleNamespace(name='obs')]
        def run(self, *args):
            raise RuntimeError('injected inference failure')
    state.ort_session_ = BrokenSession()
    with pytest.raises(RuntimeError, match='injected inference failure'):
        state.compute_actions()


def test_pvd_uses_current_physics_state():
    from simulator_view_asyn import RobotSimulator
    simulator = RobotSimulator.__new__(RobotSimulator)
    simulator.model = mujoco.MjModel.from_xml_path(str(ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml'))
    simulator.data = mujoco.MjData(simulator.model)
    joint = simulator.model.joint('knee_pitch_l_joint')
    simulator.data.qpos[joint.qposadr[0]] = 0.3
    simulator.data.qvel[joint.dofadr[0]] = 0.2
    # No published snapshot: PVD must read qpos/qvel, not stale 100 Hz feedback.
    assert simulator.pvd_controller('knee_pitch_l_joint', 0.4, 0, 2, 100, 5) == pytest.approx(11)


def test_stale_imu_is_rejected():
    import time
    from common.robot_data import RobotData
    from common.robot_interface import RobotInterfaceImpl
    from common.peekqueue import PeekableQueue
    from bodyctrl_msgs.msg import Imu
    interface = RobotInterfaceImpl.__new__(RobotInterfaceImpl)
    interface.robot_data_ = RobotData()
    interface.queue_imu_xsens = PeekableQueue(maxsize=1)
    interface.queue_imu_xsens.put(Imu())
    interface.last_received = {'imu': time.monotonic()-1}
    interface.feedback_timeout = 0.5
    with pytest.raises(TimeoutError):
        interface.get_imu_data()


def test_missing_position_actuator_does_not_disable_last_motor():
    from simulator_view_asyn import RobotSimulator
    simulator = RobotSimulator.__new__(RobotSimulator)
    simulator.model = mujoco.MjModel.from_xml_path(str(ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml'))
    simulator.pos_actuator_indices_map = {'absent': -1}
    simulator.control_mode = 1
    gains = simulator.model.actuator_gainprm.copy()
    simulator.init_controller()
    np.testing.assert_array_equal(simulator.model.actuator_gainprm, gains)

