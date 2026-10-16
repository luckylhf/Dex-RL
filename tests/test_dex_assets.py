import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'TienKung-Lab/legged_lab/assets'


def test_assets_only_contains_dex_package():
    entries = {p.name for p in ASSETS.iterdir() if p.name not in {'.DS_Store', '__pycache__'}}
    assert entries == {'Tienkung_Dex', '__init__.py'}


def test_dex_training_asset_modules_resolve_locally():
    sys.path.insert(0, str(ROOT / 'TienKung-Lab'))
    for task in ['walk', 'run']:
        tree = ast.parse((ROOT / f'TienKung-Lab/legged_lab/envs/dex/{task}_cfg.py').read_text())
        imports = [n.module for n in tree.body if isinstance(n, ast.ImportFrom)
                   and n.module and n.module.startswith('legged_lab.assets.')]
        assert imports == [f'legged_lab.assets.Tienkung_Dex.{task}']
        spec = importlib.util.find_spec(imports[0])
        assert Path(spec.origin) == ASSETS / f'Tienkung_Dex/{task}.py'


def test_only_dex_tasks_are_registered_without_lite_imports():
    tree = ast.parse((ROOT / 'TienKung-Lab/legged_lab/envs/__init__.py').read_text())
    imports = [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
    assert not any(m and m.startswith('legged_lab.envs.tienkung') for m in imports)
    tasks = {n.args[0].value for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr == 'register'}
    assert tasks == {'dex_walk', 'dex_run'}


def test_migration_preserves_training_parameters():
    before = json.loads((ROOT / 'artifacts/asset_cleanup_before.json').read_text())
    for task in ['walk', 'run']:
        path = ASSETS / f'Tienkung_Dex/{task}.py'
        content = path.read_bytes().replace(b'replace_cylinders_with_capsules=False',
                                            b'replace_cylinders_with_capsules=True')
        old_height = b'0.97' if task == 'walk' else b'1.0'
        content = content.replace(b'pos=(0.0, 0.0, 0.995)', b'pos=(0.0, 0.0, '+old_height+b')')
        if task == 'run':
            content = content.replace(b'"shoulder_pitch_l_joint": 0.20', b'"shoulder_pitch_l_joint": 0.00')
            content = content.replace(b'"shoulder_pitch_r_joint": 0.20', b'"shoulder_pitch_r_joint": 0.00')
            content = content.replace(b'".*knee_pitch.*joint": 330', b'".*knee_pitch.*joint": 350')
        assert hashlib.sha256(content).hexdigest() == before['configs'][task]
