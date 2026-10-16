from pathlib import Path
import runpy
import shutil

import mujoco
import setuptools

ROOT = Path(__file__).resolve().parents[1]


def test_installed_dex_model_loads_without_training_checkout(tmp_path, monkeypatch):
    package = ROOT / 'xSIM_MUJOCO-main'
    captured = {}
    monkeypatch.setattr(setuptools, 'setup', lambda **kwargs: captured.update(kwargs))
    monkeypatch.chdir(package)
    runpy.run_path(str(package / 'setup.py'))
    for destination, files in captured['data_files']:
        if destination.startswith('share/mujoco_node/resources/dex_v3'):
            target = tmp_path / destination
            target.mkdir(parents=True, exist_ok=True)
            for source in files:
                shutil.copy2(source, target / Path(source).name)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / 'share/mujoco_node/resources/dex_v3/urdf/dex_v3.xml'))
    assert model.nv == 35
    assert model.opt.timestep == 0.0025
