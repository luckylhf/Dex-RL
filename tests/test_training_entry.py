import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def cli():
    path = ROOT / 'TienKung-Lab/legged_lab/utils/cli_args.py'
    spec = importlib.util.spec_from_file_location('training_cli_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('arguments,initial,expected', [
    ([], True, True),
    (['--resume', 'False'], True, False),
    (['--resume', 'false'], True, False),
    (['--resume', 'True'], False, True),
    (['--resume', 'true'], False, True),
    (['--resume'], False, True),
])
def test_resume_cli_sets_the_requested_boolean(cli, arguments, initial, expected):
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int)
    cli.add_rsl_rl_args(parser)
    config = SimpleNamespace(resume=initial, logger='tensorboard')
    cli.update_rsl_rl_cfg(config, parser.parse_args(arguments))
    assert config.resume is expected


def test_resume_rejects_misspelled_boolean(cli):
    parser = argparse.ArgumentParser()
    cli.add_rsl_rl_args(parser)
    with pytest.raises(SystemExit) as error:
        parser.parse_args(['--resume', 'flase'])
    assert error.value.code == 2


def test_training_entry_uses_v4_training_root_and_preserves_arguments(tmp_path):
    scripts = tmp_path / 'v4/scripts'
    training = tmp_path / 'v4/TienKung-Lab'
    scripts.mkdir(parents=True)
    (training / 'legged_lab/scripts').mkdir(parents=True)
    shutil.copy2(ROOT / 'scripts/ubuntu_train.sh', scripts / 'ubuntu_train.sh')
    (scripts / 'ubuntu_env.sh').write_text('#!/bin/bash\ncd "$(dirname "$0")/.."\nexec "$@"\n')
    (training / 'legged_lab/scripts/train.py').write_text(
        'import json,os,sys\nprint(json.dumps({"cwd":os.getcwd(),"args":sys.argv[1:]}))\n')
    args = ['--task=dex_walk', '--run_name', 'with space', '--num_envs', '8000', '--max_iterations', '20000']
    result = subprocess.run(['bash', str(scripts / 'ubuntu_train.sh'), *args], cwd=tmp_path,
                            capture_output=True, text=True,
                            env=dict(os.environ, PATH=str(Path(sys.executable).parent)+os.pathsep+os.environ['PATH']))
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout) == {'cwd': str(training), 'args': args}


def test_default_training_analyzer_finds_dex_walk_logs(tmp_path, monkeypatch):
    path = ROOT / 'TienKung-Lab/legged_lab/scripts/analyze_training_log.py'
    spec = importlib.util.spec_from_file_location('training_analyzer_test', path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    event = tmp_path / 'logs/dex_walk/run/events.out.tfevents.test'
    event.parent.mkdir(parents=True)
    event.write_text('test')
    monkeypatch.setattr(module, 'WORKSPACE_ROOT', tmp_path)
    selected = []
    monkeypatch.setattr(module, 'run_once', lambda path, *args: selected.append(path))
    monkeypatch.setattr(sys, 'argv', ['analyze_training_log.py'])
    module.main()
    assert selected == [event]


def test_environment_entry_does_not_reactivate_the_active_training_env():
    shell = '''
source() {
    case "$1" in
        /opt/ros/humble/setup.bash|*/install/setup.bash) return 0 ;;
        *) echo 'conda must not be activated twice' >&2; return 73 ;;
    esac
}
export -f source
exec bash "$1" "$2" -c 'print("ACTIVE_ENV_OK")'
'''
    result = subprocess.run(['bash', '-c', shell, 'bash', str(ROOT/'scripts/ubuntu_env.sh'), sys.executable],
        env=dict(os.environ, CONDA_PREFIX=str(Path.home()/'miniconda3/envs/env_isaaclab')),
        capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == 'ACTIVE_ENV_OK'


def test_ubuntu_deployment_pins_match_the_verified_base_environment():
    pins = []
    for name in ['requirements-humble.txt', 'Deploy_Tienkung/requirements_22.txt']:
        pins.append(dict(line.strip().split('==', 1) for line in (ROOT/name).read_text().splitlines()
                         if '==' in line and not line.lstrip().startswith('#')))
    assert {name: (version, pins[1][name]) for name, version in pins[0].items()
            if name in pins[1] and version != pins[1][name]} == {}


@pytest.mark.parametrize('failure,exit_code', [(False, 0), (True, 1)])
def test_training_main_closes_isaac_and_reports_the_real_status(failure, exit_code):
    import ast
    import traceback
    path = ROOT/'TienKung-Lab/legged_lab/scripts/train.py'
    tree = ast.parse(path.read_text())
    main = next(node for node in tree.body if isinstance(node, ast.If)
                and ast.unparse(node.test) == "__name__ == '__main__'")
    events = []
    context_active = [True]
    requested_status = [0]
    def train():
        events.append('train')
        if failure:
            raise RuntimeError('injected training failure')
    def launch(args, fast_shutdown=True):
        def close():
            assert not context_active[0], 'unsubscribe the timeline before closing Isaac'
            events.append('close')
            if fast_shutdown:
                raise SystemExit(requested_status[0])
        def post_quit(code):
            requested_status[0] = code
            events.append('quit')
        return SimpleNamespace(app=SimpleNamespace(close=close, app=SimpleNamespace(post_quit=post_quit)))
    setup = [node for node in tree.body if isinstance(node, ast.Assign)
             and any(isinstance(target, ast.Name) and target.id in ['app_launcher', 'simulation_app']
                     for target in node.targets)]
    def clear_instance():
        events.append('clear')
        context_active[0] = False
    namespace = {'__name__': '__main__', 'train': train, 'sys': sys, 'traceback': traceback,
                 'AppLauncher': launch, 'args_cli': SimpleNamespace(),
                 'SimulationContext': SimpleNamespace(clear_instance=clear_instance)}
    with pytest.raises(SystemExit) as error:
        exec(compile(ast.Module(body=setup+main.body, type_ignores=[]), str(path), 'exec'), namespace)
    assert error.value.code == exit_code
    assert events == (['train', 'quit', 'clear', 'close'] if failure else ['train', 'clear', 'close'])
