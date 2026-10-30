"""WALKAMP keeps policy-excluded arm joints at their entry positions."""
import numpy as np
import pytest
from types import SimpleNamespace

from common.control_flag import ControlFlag
from common.robot_data import RobotData
from policy.walk_amp.fsm_walkamp import FSMStateWALKAMP


def test_walk_holds_six_arm_joints_without_changing_policy_gains(monkeypatch):
    robot = RobotData()
    state = FSMStateWALKAMP(robot)
    hold = np.array([19, 20, 21, 26, 27, 28])
    np.testing.assert_array_equal(state.hold_indices, hold)
    entry = np.array([0.12, -0.08, 0.05, -0.11, 0.07, -0.04])
    robot.q_a_[6 + hold] = entry
    state.on_enter()
    monkeypatch.setattr(state, 'compute_observation', lambda *args: None)
    monkeypatch.setattr(state, 'compute_actions', lambda: state.actions_.fill(0.2))

    for step in range(state._warm_start_steps + 2):
        robot.time_now_ = step * state.dt_
        robot.q_a_[6 + hold] = entry + 0.2
        state.run(ControlFlag())
        np.testing.assert_allclose(robot.q_d_[6 + hold], entry, atol=1e-7)
        np.testing.assert_array_equal(robot.q_dot_d_[6 + hold], 0)
        np.testing.assert_array_equal(robot.tau_d_[6 + hold], 0)
        np.testing.assert_array_equal(robot.joint_kp_p_[hold], 80)
        np.testing.assert_array_equal(robot.joint_kd_p_[hold], 5)

    np.testing.assert_allclose(robot.joint_kp_p_[state.lab2mj], state.stiffness_array_seq)
    np.testing.assert_allclose(robot.joint_kd_p_[state.lab2mj], state.damping_array_seq)
    np.testing.assert_allclose(robot.q_d_[6 + state.lab2mj],
                               state.default_angles_lab + 0.2 * state.action_scale_lab,
                               atol=1e-7)

    # Exercise the real publishing path and verify the deployed motor IDs.
    from builtin_interfaces.msg import Time
    from common.body_id_map import BodyServoIdMap
    from common.robot_interface import RobotInterfaceImpl
    from FSM.fsm_base import FSMStateName
    interface = RobotInterfaceImpl.__new__(RobotInterfaceImpl)
    interface.id_map = BodyServoIdMap()
    interface.id_map.body_can_id_map_init()
    interface.robot_data_ = robot
    interface.floating_base_dof = 6
    interface.current_state = FSMStateName.WALKAMP
    interface.legs_control_status = [FSMStateName.STOP]
    interface.waist_control_status = [FSMStateName.STOP]
    interface.arms_control_status = [FSMStateName.WALKAMP]
    interface.left_arm_only_status = []
    interface.right_arm_only_status = []
    interface.sim = False
    interface.clip_actions = False
    interface.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())))
    published = []
    interface.pub_arm_motor_cmd = SimpleNamespace(publish=published.append)
    interface.publish_motor_commands(ControlFlag())
    commands = {cmd.name: cmd for cmd in published[0].cmds}
    for motor_id, target in zip((15, 16, 17, 25, 26, 27), entry):
        cmd = commands[motor_id]
        assert cmd.pos == pytest.approx(target)
        assert (cmd.kp, cmd.kd, cmd.spd, cmd.tor) == (80, 5, 0, 0)

    # A new transition captures the new measured pose, not the old target.
    state.on_enter()
    state.run(ControlFlag())
    np.testing.assert_allclose(robot.q_d_[6 + hold], entry + 0.2, atol=1e-7)


def test_walk_rejects_nonfinite_entry_feedback():
    robot = RobotData()
    robot.q_a_[6 + 20] = np.nan
    state = FSMStateWALKAMP(robot)
    with pytest.raises(ValueError, match='finite joint feedback'):
        state.on_enter()
