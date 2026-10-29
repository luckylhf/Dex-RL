"""Check that regeneration preserves the shipped MuJoCo model in isolation."""
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSET = Path('TienKung-Lab/legged_lab/assets/Tienkung_Dex')
MODEL = Path('xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml')


def xml_signature(element):
    children = list(element)
    if element.tag == 'asset':
        # Mesh ordering is not semantically significant and may vary by MuJoCo version.
        children.sort(key=lambda child: (child.tag, child.get('name', '')))
    return (element.tag, tuple(sorted(element.attrib.items())),
            tuple(xml_signature(child) for child in children))


def test_sync_reproduces_shipped_mujoco_model(tmp_path):
    for relative in (Path('scripts/sync_dex.py'), ASSET / 'urdf/tiangong3.urdf',
                     ASSET / 'walk.py', Path('Deploy_Tienkung/config/dex_config.yaml'),
                     Path('Deploy_Tienkung/policy/walk_amp/config/walk_amp.yaml')):
        target = tmp_path / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / relative, target)
    shutil.copytree(ROOT / ASSET / 'meshes', tmp_path / ASSET / 'meshes')
    imported = tmp_path / ASSET / 'mujoco/meshes_import'
    imported.mkdir(parents=True)
    for name in ('waist_pitch_link_lower.obj', 'waist_pitch_link_upper.obj'):
        shutil.copy2(ROOT / ASSET / 'mujoco/meshes_import' / name, imported / name)

    subprocess.run([sys.executable, str(tmp_path / 'scripts/sync_dex.py')], check=True)
    generated = tmp_path / MODEL
    assert xml_signature(ET.parse(generated).getroot()) == xml_signature(ET.parse(ROOT / MODEL).getroot())

    # Use the same relative resource layout as the shipped model.
    resource = generated.parent.parent
    (resource / 'meshes').symlink_to(tmp_path / ASSET / 'meshes', target_is_directory=True)
    (resource / 'mujoco').mkdir()
    (resource / 'mujoco/meshes_import').symlink_to(imported, target_is_directory=True)
    shipped = mujoco.MjModel.from_xml_path(str(ROOT / MODEL))
    rebuilt = mujoco.MjModel.from_xml_path(str(generated))
    for field in ('body_mass', 'body_inertia', 'jnt_range', 'actuator_ctrlrange',
                  'geom_type', 'geom_contype', 'geom_conaffinity', 'geom_rgba'):
        np.testing.assert_allclose(getattr(rebuilt, field), getattr(shipped, field), rtol=1e-7, atol=1e-9)
    assert (rebuilt.nq, rebuilt.nv, rebuilt.nu, rebuilt.nsensor) == (shipped.nq, shipped.nv, shipped.nu, shipped.nsensor)
    original_data, rebuilt_data = mujoco.MjData(shipped), mujoco.MjData(rebuilt)
    for _ in range(100):
        mujoco.mj_step(shipped, original_data)
        mujoco.mj_step(rebuilt, rebuilt_data)
    np.testing.assert_allclose(rebuilt_data.qpos, original_data.qpos, rtol=1e-7, atol=1e-9)
    assert rebuilt_data.ncon == original_data.ncon
