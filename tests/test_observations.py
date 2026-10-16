import importlib.util
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'TienKung-Lab/legged_lab/scripts'))


def test_independent_and_deployment_observations_match():
    from sim2sim import MujocoRunner
    from common.robot_data import RobotData
    from common.control_flag import ControlFlag
    from common.BasicFunction import gait_phase
    from policy.walk_amp.fsm_walkamp import FSMStateWALKAMP
    plant = MujocoRunner(ROOT / 'Deploy_Tienkung/policy/walk_amp/model/policy.onnx')
    robot = RobotData()
    controller = FSMStateWALKAMP(robot)
    for frame in range(12):
        euler = [0.1 + frame*0.001, -0.2, 0.3]
        xyzw = Rotation.from_euler('xyz', euler).as_quat()
        plant.data.qpos[3:7] = xyzw[[3, 0, 1, 2]]
        plant.data.qpos[plant.qidx] = plant.default + np.sin(np.arange(23)+frame)*0.01
        plant.data.qvel[plant.vidx] = np.cos(np.arange(23)+frame)*0.1
        mujoco.mj_forward(plant.model, plant.data)
        command = [0.4, -0.1, 0.2]
        last_action = np.linspace(-0.5, 0.5, 23, dtype=np.float32)
        plant.action[:] = last_action
        controller.last_actions_[:] = last_action
        robot.q_a_[6+controller.lab2mj] = plant.data.qpos[plant.qidx]
        robot.q_dot_a_[6+controller.lab2mj] = plant.data.qvel[plant.vidx]
        robot.imu_data_[:6] = [euler[2], euler[1], euler[0], *plant.data.sensor('angular-velocity').data]
        robot.walk_cmd_ = command
        plant.steps = frame
        gait = gait_phase(frame*plant.period, 0.85, 0.38, 0.88, 0.38, 0.38)
        controller.compute_observation(ControlFlag(), gait)
        np.testing.assert_allclose(plant.get_obs(command), controller.observations_, atol=1e-6)

