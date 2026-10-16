from pathlib import Path
import runpy
from types import SimpleNamespace

import pytest
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODULE = runpy.run_path(str(ROOT / 'Deploy_Tienkung/policy/mimic/out/sim2sim_dex.py'))


def test_mimic_debug_defaults_use_shared_model_and_400_100_hz():
    args = MODULE['build_argparser']().parse_args([])
    assert args.sim_dt == 0.0025
    assert args.control_dt == 0.01
    assert args.mjcf == ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml'
    assert args.policy.is_file()


@pytest.mark.parametrize('npz', [False, True])
def test_100hz_mimic_debug_keeps_50fps_reference_duration(npz):
    sim = MODULE['MujocoSim2Sim'].__new__(MODULE['MujocoSim2Sim'])
    sim.control_dt = 0.01
    sim.teleop_stream = SimpleNamespace(total_steps=2, fps=50, reference_at=lambda i: i) if npz else None
    sim.motion_library = None
    sim.metadata = SimpleNamespace(total_steps=2)
    sim.reference_outputs = [0, 1]
    sim._normalize_reference_output = lambda x: x
    assert sim._total_steps == 4
    assert [sim._reference_output(i) for i in range(4)] == [0, 0, 1, 1]


def test_mimic_debug_reset_uses_approved_initial_height():
    sim = MODULE['MujocoSim2Sim'].__new__(MODULE['MujocoSim2Sim'])
    sim.model = mujoco.MjModel.from_xml_path(str(ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml'))
    sim.data = mujoco.MjData(sim.model)
    sim.last_action = np.zeros(29)
    sim.teleop_stream = sim.motion_library = None
    sim.joint_qpos_indices = sim.model.jnt_qposadr[sim.model.actuator_trnid[:, 0]]
    sim.joint_qvel_indices = sim.model.jnt_dofadr[sim.model.actuator_trnid[:, 0]]
    sim.actuator_index_array = np.arange(29)
    sim.reference_outputs = [{'body_pos_w': np.array([[0, 0, 0.9]]),
                              'body_quat_w': np.array([[1, 0, 0, 0]]),
                              'body_lin_vel_w': np.zeros((1, 3)),
                              'body_ang_vel_w': np.zeros((1, 3)),
                              'joint_pos': np.zeros(29), 'joint_vel': np.zeros(29)}]
    sim.reset()
    assert sim.data.qpos[2] == 0.995
    assert sim.reference_outputs[0]['body_pos_w'][0, 2] == 0.9
    legacy = mujoco.MjModel.from_xml_path(str(ROOT / 'Deploy_Tienkung/policy/mimic/out/dex_evt/urdf/evt2.xml'))
    assert legacy.qpos0[2] == 0.995
