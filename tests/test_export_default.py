import hashlib
import json
from pathlib import Path
import runpy
import sys

import pytest
import torch

ROOT = Path(__file__).resolve().parents[1]


def test_export_default_uses_selected_duan_pt(monkeypatch):
    selected = []
    def capture(path, **kwargs):
        selected.append(Path(path))
        raise StopIteration
    monkeypatch.setattr(torch.jit, 'load', capture)
    monkeypatch.setattr(sys, 'argv', ['export_policy.py'])
    with pytest.raises(StopIteration):
        runpy.run_path(str(ROOT / 'scripts/export_policy.py'), run_name='__main__')
    baseline = json.loads((ROOT / 'tests/fixtures/duan_baseline.json').read_text())
    assert hashlib.sha256(selected[0].read_bytes()).hexdigest() == baseline['source_files']['policy.pt']['sha256']


def test_run_export_requires_explicit_policy(monkeypatch):
    def forbid_loading(*args, **kwargs):
        raise AssertionError('run must not silently load the walk policy')
    monkeypatch.setattr(torch.jit, 'load', forbid_loading)
    monkeypatch.setattr(sys, 'argv', ['export_policy.py', '--task', 'dex_run'])
    with pytest.raises(SystemExit) as error:
        runpy.run_path(str(ROOT / 'scripts/export_policy.py'), run_name='__main__')
    assert error.value.code == 2


def test_run_export_accepts_explicit_policy(monkeypatch, tmp_path):
    selected = []
    def capture(path, **kwargs):
        selected.append(Path(path))
        raise StopIteration
    path = tmp_path / 'run.pt'
    monkeypatch.setattr(torch.jit, 'load', capture)
    monkeypatch.setattr(sys, 'argv', ['export_policy.py', '--task', 'dex_run', '--policy', str(path)])
    with pytest.raises(StopIteration):
        runpy.run_path(str(ROOT / 'scripts/export_policy.py'), run_name='__main__')
    assert selected == [path]
