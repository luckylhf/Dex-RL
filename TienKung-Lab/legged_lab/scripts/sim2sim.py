"""Dex V3 sim2sim, using the same plant and nominal gains as ROS deployment."""
import argparse
import ast
from collections import deque
import json
from pathlib import Path
import time
import re

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[3]


def _delete(spec, element):
  """Delete one element from an MjSpec on every mujoco build.

  mujoco 3.3.x exposes removal on the element itself (`MjsJoint.delete()`,
  `MjsActuator.delete()`, ...); newer builds also accept `MjSpec.delete(obj)`.
  """
  delete = getattr(spec, 'delete', None)
  if delete is not None:
    delete(element)
  else:
    element.delete()


def _load_model(model_path, joint_names):
  """Load the MJCF and weld every joint the policy does not command.

  The training model keeps the elbow-yaw and wrist joints fixed; this file
  keeps them as free hinges, so leaving them unactuated makes the forearms
  flop. Deleting the joint together with the motors and sensors bound to it
  reproduces the training model (23 actuated joints) exactly.
  """
  try:
    spec = mujoco.MjSpec.from_file(str(model_path))
  except Exception as error:
    print(f'[sim2sim] MjSpec load failed ({error}); keeping all joints movable')
    return mujoco.MjModel.from_xml_path(str(model_path)), []
  locked = [joint.name for joint in spec.joints
            if joint.name not in joint_names
            and joint.type != mujoco.mjtJoint.mjJNT_FREE]
  for name in locked:
    for actuator in [a for a in spec.actuators if a.name == 'motor_' + name]:
      _delete(spec, actuator)
    for sensor in [s for s in spec.sensors if s.objname == name]:
      _delete(spec, sensor)
    _delete(spec, spec.joint(name))
  return spec.compile(), locked


class MujocoRunner:
    def __init__(self, policy_path, model_path=None, task='dex_walk'):
        self.cfg = yaml.safe_load((ROOT / 'Deploy_Tienkung/policy/walk_amp/config/walk_amp.yaml').read_text())
        self.gait_cycle = 0.85
        self.phase_offsets = np.array([0.38, 0.88])
        self.phase_ratios = [0.38, 0.38]
        if task in ['dex_run', 'run']:
            tree = ast.parse((ROOT / 'TienKung-Lab/legged_lab/assets/Tienkung_Dex/run.py').read_text())
            config = next(n.value for n in tree.body if isinstance(n, ast.Assign)
                          and any(isinstance(t, ast.Name) and t.id == 'DEX_V3_CFG' for t in n.targets))
            initial = next(k.value for k in config.keywords if k.arg == 'init_state')
            angles = ast.literal_eval(next(k.value for k in initial.keywords if k.arg == 'joint_pos'))
            self.cfg['init_state']['default_joint_angles'] = [angles[n] for n in self.cfg['joint_names']]
            actuators = next(k.value for k in config.keywords if k.arg == 'actuators')
            for field, gain in [('stiffness', 'kp'), ('damping', 'kd')]:
                for actuator in actuators.values:
                    values = ast.literal_eval(next(k.value for k in actuator.keywords if k.arg == field))
                    for i, name in enumerate(self.cfg['joint_names']):
                        for pattern, value in values.items():
                            if re.fullmatch(pattern, name):
                                self.cfg['gains'][gain][i] = value
            self.gait_cycle = 0.64
            self.phase_offsets = np.array([0.6, 0.1])
            self.phase_ratios = [0.6, 0.6]
        model_path = model_path or ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml'
        self.names = self.cfg['joint_names']
        self.model, self.locked_joints = _load_model(model_path, self.names)
        if self.locked_joints:
            print(f'[sim2sim] locked (not commanded by policy): {", ".join(self.locked_joints)}')
        self.data = mujoco.MjData(self.model)
        self.qidx = np.array([self.model.joint(n).qposadr[0] for n in self.names])
        self.vidx = np.array([self.model.joint(n).dofadr[0] for n in self.names])
        self.aidx = np.array([self.model.actuator('motor_' + n).id for n in self.names])
        self.default = np.array(self.cfg['init_state']['default_joint_angles'])
        self.kp = np.array(self.cfg['gains']['kp'])
        self.kd = np.array(self.cfg['gains']['kd'])
        self.limits = self.model.actuator_ctrlrange[self.aidx]
        self.model.actuator_biastype[self.aidx] = mujoco.mjtBias.mjBIAS_AFFINE
        self.model.actuator_biasprm[self.aidx, 1] = -self.kp
        self.model.actuator_biasprm[self.aidx, 2] = -self.kd
        self.model.actuator_ctrllimited[self.aidx] = False
        self.model.actuator_forcelimited[self.aidx] = True
        self.model.actuator_forcerange[self.aidx] = self.limits
        self.data.qpos[:7] = [0, 0, 0.995, 1, 0, 0, 0]
        self.data.qpos[self.qidx] = self.default
        for i in range(self.model.nu):
            if self.model.actuator(i).name.startswith('pos_'):
                self.model.actuator_gainprm[i] = 0
                self.model.actuator_biasprm[i] = 0
        mujoco.mj_forward(self.model, self.data)
        self.action = np.zeros(23, dtype=np.float32)
        self.history = np.zeros(840, dtype=np.float32)
        self.first_obs = True
        self.steps = 0
        self.period = self.cfg['dt'] * self.cfg['control']['decimation']
        self.decimation = round(self.period / self.model.opt.timestep)
        if not np.isclose(self.decimation * self.model.opt.timestep, self.period):
            raise ValueError('Physics timestep must divide the policy period')
        self.policy_path = Path(policy_path)
        if self.policy_path.suffix == '.pt':
            import torch
            torch.set_num_threads(1)
            self.policy = torch.jit.load(str(policy_path), map_location='cpu').eval()
        elif self.policy_path.suffix == '.onnx':
            import onnxruntime as ort
            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            self.policy = ort.InferenceSession(str(policy_path), options, providers=['CPUExecutionProvider'])
            if self.policy.get_inputs()[0].shape != [1, 840] or self.policy.get_outputs()[0].shape != [1, 23]:
                raise ValueError('Expected a 840-input / 23-output Dex policy')
        else:
            raise ValueError('Policy must be a TorchScript .pt or an .onnx file')

    def get_obs(self, command):
        mujoco.mj_forward(self.model, self.data)
        phase = (self.steps * self.period / self.gait_cycle + self.phase_offsets) % 1
        q = self.data.sensor('orientation').data
        gravity = Rotation.from_quat(q[[1, 2, 3, 0]]).inv().apply([0, 0, -1])
        obs = np.concatenate([self.data.sensor('angular-velocity').data, gravity, command,
                              self.data.qpos[self.qidx] - self.default, self.data.qvel[self.vidx],
                              self.action, np.sin(2*np.pi*phase), np.cos(2*np.pi*phase), self.phase_ratios]).astype(np.float32)
        if obs.shape != (84,) or not np.isfinite(obs).all():
            raise ValueError('Invalid Dex observation')
        if self.first_obs:
            self.history[:] = np.tile(obs, 10)
            self.first_obs = False
        else:
            self.history[:-84] = self.history[84:]
            self.history[-84:] = obs
        return np.clip(self.history, -100, 100)

    def step(self, command):
        obs = self.get_obs(command)[None]
        if self.policy_path.suffix == '.pt':
            import torch
            with torch.inference_mode():
                action = self.policy(torch.from_numpy(obs)).numpy()[0]
        else:
            action = self.policy.run(None, {self.policy.get_inputs()[0].name: obs})[0][0]
        if action.shape != (23,) or not np.isfinite(action).all():
            raise ValueError('Invalid Dex action')
        self.action = np.clip(action, -100, 100)
        target = self.default + self.cfg['control']['action_scale'] * self.action
        self.data.ctrl[self.aidx] = self.kp * target
        for _ in range(self.decimation):
            mujoco.mj_step(self.model, self.data)
        self.steps += 1
        return float(self.data.qpos[2])

    def reset(self):
        """Restore the initial pose, velocity and observation history."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:7] = [0, 0, 0.995, 1, 0, 0, 0]
        self.data.qpos[self.qidx] = self.default
        self.action[:] = 0
        self.history[:] = 0
        self.first_obs = True
        self.steps = 0
        mujoco.mj_forward(self.model, self.data)


class Teleop:
    """Keyboard velocity command, mirroring the Isaac Lab play keyboard bindings."""

    VX_STEP, VX_LIMITS = 0.2, (-0.6, 1.2)
    VY_STEP, VY_LIMITS = 0.1, (-0.5, 0.5)
    WZ_STEP, WZ_LIMITS = 0.2, (-1.57, 1.57)

    def __init__(self, command):
        # `mujoco.glfw` 子模块只在 mujoco.gl_context 走默认(GLFV)分支时被顺带导入；当
        # MUJOCO_GL=egl/disable，或平台是 macOS(Darwin) 时不会自动导入，直接访问
        # mujoco.glfw 会抛 AttributeError。显式导入子模块；只放在 GUI 路径上，避免
        # 无头运行平白依赖 libglfw。
        import mujoco.glfw
        self.glfw = mujoco.glfw.glfw
        self.command = [float(value) for value in command]
        self.keys = deque(maxlen=64)
        self.reset_requested = False

    def key_callback(self, key):
        """Called from the viewer thread for every key pressed in the sim window."""
        self.keys.append(key)

    def _bump(self, index, step, limits, sign):
        self.command[index] = round(min(max(self.command[index] + sign * step, limits[0]), limits[1]), 2)

    def update(self):
        """Apply the keys seen since the last policy step; returns True on reset."""
        glfw = self.glfw
        before = list(self.command)
        while self.keys:
            key = self.keys.popleft()
            if key in (glfw.KEY_W, glfw.KEY_UP):
                self._bump(0, self.VX_STEP, self.VX_LIMITS, 1)
            elif key in (glfw.KEY_S, glfw.KEY_DOWN):
                self._bump(0, self.VX_STEP, self.VX_LIMITS, -1)
            elif key in (glfw.KEY_A, glfw.KEY_LEFT):
                self._bump(1, self.VY_STEP, self.VY_LIMITS, 1)
            elif key in (glfw.KEY_D, glfw.KEY_RIGHT):
                self._bump(1, self.VY_STEP, self.VY_LIMITS, -1)
            elif key == glfw.KEY_Q:
                self._bump(2, self.WZ_STEP, self.WZ_LIMITS, 1)
            elif key == glfw.KEY_E:
                self._bump(2, self.WZ_STEP, self.WZ_LIMITS, -1)
            elif key in (glfw.KEY_C, glfw.KEY_X):
                self.command = [0.0, 0.0, 0.0]
            elif key == glfw.KEY_R:
                self.reset_requested = True
        reset = self.reset_requested
        self.reset_requested = False
        if reset or self.command != before:
            state = f'vx={self.command[0]:.2f} vy={self.command[1]:.2f} wz={self.command[2]:.2f}'
            print(f'[TELEOP] {"reset | " if reset else ""}command {state}')
        return reset

    def overlay(self, runner):
        lines = [
            'Teleop keys',
            'W/UP forward +0.2   S/DOWN -0.2    [-0.6, 1.2]',
            'A/LEFT lateral +0.1 D/RIGHT -0.1   [-0.5, 0.5]',
            'Q yaw +0.2          E yaw -0.2     [-1.57, 1.57]',
            'C or X stop         R reset',
            f'cmd vx {self.command[0]:+.2f}  vy {self.command[1]:+.2f}  wz {self.command[2]:+.2f}',
            f'pelvis {runner.data.qpos[2]:.3f} m   sim {runner.data.time:.1f} s',
        ]
        return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', choices=['dex_walk', 'walk', 'dex_run', 'run'], default='dex_walk')
    parser.add_argument('--policy', type=Path)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--command', type=float, nargs=3, default=[0, 0, 0])
    parser.add_argument('--sequence', choices=['constant', 'stand_walk_stop'], default='constant')
    parser.add_argument('--no-teleop', action='store_true', help='hide the keyboard command window and ignore its keys')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.policy is None:
        if args.task in ['dex_run', 'run']:
            parser.error('run requires an explicit matching --policy')
        args.policy = ROOT / 'Deploy_Tienkung/policy/walk_amp/model/policy.onnx'
    runner = MujocoRunner(args.policy, args.model, args.task)
    teleop = None
    if not args.headless and not args.no_teleop and args.sequence == 'constant':
        teleop = Teleop(args.command)
    viewer = None
    if not args.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(
            runner.model, runner.data,
            key_callback=None if teleop is None else teleop.key_callback)
    heights = []
    positions = []
    started = time.monotonic()
    try:
        for i in range(round(args.duration / runner.period)):
            command = args.command
            if args.sequence == 'stand_walk_stop':
                command = [0.4, 0, 0] if args.duration/3 <= i*runner.period < args.duration*2/3 else [0, 0, 0]
            elif teleop is not None:
                if teleop.update():
                    runner.reset()
                command = teleop.command
            heights.append(runner.step(command))
            positions.append(runner.data.qpos[:3].copy().tolist())
            if viewer:
                if not viewer.is_running():
                    break
                if teleop is not None:
                    viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100,
                                       mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                       teleop.overlay(runner), '')])
                viewer.sync()
                remaining = started + runner.steps * runner.period - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
    finally:
        if viewer:
            viewer.close()
    report = {'policy': str(args.policy), 'steps': runner.steps, 'simulation_seconds': float(runner.data.time),
              'min_pelvis_height': min(heights), 'final_position': positions[-1],
              'finite': bool(np.isfinite(runner.data.qpos).all()), 'collapsed': min(heights) < 0.5,
              'sequence': args.sequence}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if not report['finite'] or report['collapsed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
