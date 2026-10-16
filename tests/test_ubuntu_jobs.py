import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def launcher(tmp_path):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    shutil.copy2(ROOT / 'scripts/ubuntu_job.py', scripts / 'ubuntu_job.py')
    (scripts / 'ubuntu_env.sh').write_text('#!/usr/bin/env bash\nexec "$@"\n')
    return scripts / 'ubuntu_job.py'


@pytest.mark.parametrize('exit_code', [0, 7])
def test_detached_job_keeps_running_and_records_output_and_exit_code(launcher, exit_code):
    child = f'import sys,time; time.sleep(0.5); print("remote stdout"); print("remote stderr",file=sys.stderr); sys.exit({exit_code})'
    parent = subprocess.run([sys.executable, str(launcher), 'smoke', sys.executable, '-c', child],
                            capture_output=True, text=True, timeout=5)
    assert parent.returncode == 0, parent.stderr
    job = json.loads(parent.stdout)
    directory = Path(job['directory'])
    assert job['pid'] > 0 and (directory / 'job.json').is_file()
    assert not (directory / 'result.json').exists()  # launcher has returned before its job finishes
    deadline = time.monotonic() + 5
    while not (directory / 'result.json').exists():
        assert time.monotonic() < deadline
        time.sleep(0.02)
    result = json.loads((directory / 'result.json').read_text())
    assert result['returncode'] == exit_code
    assert result['finished_at']
    log = (directory / 'output.log').read_text()
    assert 'remote stdout' in log and 'remote stderr' in log


@pytest.mark.parametrize('label', ['../escape', '/absolute', 'with space'])
def test_job_label_cannot_escape_the_run_directory(launcher, label):
    result = subprocess.run([sys.executable, str(launcher), label, 'true'], capture_output=True, text=True)
    assert result.returncode == 2
    assert 'label' in result.stderr
    assert not (launcher.parents[1] / 'artifacts').exists()
