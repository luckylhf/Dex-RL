import ast
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_dex_shell_rejects_deleted_lite_robot(tmp_path):
    python = tmp_path / 'python'
    python.write_text('#!/bin/sh\nexit 0\n')
    python.chmod(0o755)
    result = subprocess.run(['bash', str(ROOT / 'TienKung-Lab/shell/sim2sim.sh'), 'lite', 'walk'],
                            cwd=ROOT / 'TienKung-Lab', capture_output=True, text=True,
                            env=dict(os.environ, PATH=str(tmp_path) + os.pathsep + os.environ['PATH']))
    assert result.returncode == 1
    assert 'Unsupported robot' in result.stderr


@pytest.mark.parametrize('folder', ['Deploy_Tienkung', 'xSIM_MUJOCO-main', 'sim_joy'])
def test_ros_packages_declare_imported_ros_dependencies(folder):
    package = ROOT / folder
    known = {'rclpy', 'sensor_msgs', 'geometry_msgs', 'std_msgs', 'bodyctrl_msgs',
             'ament_index_python', 'launch', 'launch_ros'}
    used = set()
    for path in package.rglob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.ImportFrom) and node.module:
                used.add(node.module.split('.')[0])
            elif isinstance(node, ast.Import):
                used.update(a.name.split('.')[0] for a in node.names)
    declared = {e.text for e in ET.parse(package / 'package.xml').getroot()
                if e.tag.endswith('depend')}
    assert used & known <= declared


def test_installed_control_default_config_uses_package_share(tmp_path, monkeypatch):
    import ament_index_python.packages
    import rl_control_node
    share = tmp_path / 'share/rl_control'
    (share / 'config').mkdir(parents=True)
    config = {'motor_num': 29, 'dt': 0.01, 'sim': True, 'control_tool': 'manual'}
    path = share / 'config/dex_config_sim.yaml'
    path.write_text(yaml.safe_dump(config))
    monkeypatch.setattr(rl_control_node, '__file__', str(tmp_path / 'site-packages/rl_control_node.py'))
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory', lambda name: str(share))
    node = SimpleNamespace(sim_override=True, control_tool_override=None)
    node.declare_parameter = lambda name, default: setattr(node, 'default', default)
    node.get_parameter = lambda name: SimpleNamespace(get_parameter_value=lambda: SimpleNamespace(string_value=node.default))
    rl_control_node.XMIGCSControlNode._load_config(node)
    assert node.config_file == str(path)
    assert node.config == config


def test_ros_simulator_rejects_partial_protocol(monkeypatch):
    import sys
    import simulator_view_asyn
    def forbid_ros_start():
        raise AssertionError('partial protocol must be rejected before ROS starts')
    monkeypatch.setattr(sys, 'argv', ['simulator_view_asyn.py', '--config', '21'])
    monkeypatch.setattr(simulator_view_asyn.rclpy, 'init', forbid_ros_start)
    with pytest.raises(SystemExit) as error:
        simulator_view_asyn.main()
    assert error.value.code == 2


@pytest.mark.parametrize('ros_args', [
    ['--ros-args', '-r', '__node:=mujoco_simulator_dex'],
    ['--ros-args', '-p', 'use_sim_time:=false'],
])
def test_ros_simulator_accepts_launch_arguments(monkeypatch, ros_args):
    import sys
    import simulator_view_asyn
    argv = ['simulator_view_asyn.py', '--model', 'dex_v3', '--config', 'full',
            '--headless', '--no-band', '--duration', '1'] + ros_args
    def reached_ros_init():
        assert sys.argv == argv
        raise StopIteration
    monkeypatch.setattr(sys, 'argv', argv)
    monkeypatch.setattr(simulator_view_asyn.rclpy, 'init', reached_ros_init)
    with pytest.raises(StopIteration):
        simulator_view_asyn.main()


def test_ros_simulator_rejects_unknown_application_argument(monkeypatch):
    import sys
    import simulator_view_asyn
    def forbid_ros_start():
        raise AssertionError('unknown arguments must be rejected before ROS starts')
    monkeypatch.setattr(sys, 'argv', ['simulator_view_asyn.py', '--duraton', '1',
                                    '--ros-args', '-r', '__node:=mujoco_simulator_dex'])
    monkeypatch.setattr(simulator_view_asyn.rclpy, 'init', forbid_ros_start)
    with pytest.raises(SystemExit) as error:
        simulator_view_asyn.main()
    assert error.value.code == 2
