import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_reward_type_hints_resolve_defined_env_classes():
    tree = ast.parse((ROOT / 'TienKung-Lab/legged_lab/mdp/rewards.py').read_text())
    block = next(n for n in tree.body if isinstance(n, ast.If)
                 and isinstance(n.test, ast.Name) and n.test.id == 'TYPE_CHECKING')
    for node in block.body:
        if isinstance(node, ast.ImportFrom):
            path = ROOT / 'TienKung-Lab' / (node.module.replace('.', '/') + '.py')
            classes = {n.name for n in ast.walk(ast.parse(path.read_text())) if isinstance(n, ast.ClassDef)}
            for alias in node.names:
                assert alias.name in classes, f'{node.module} does not define {alias.name}'


@pytest.mark.parametrize('old_present,new_present', [(True, False), (False, True), (True, True), (False, False)])
def test_actual_reward_alias_keeps_old_and_new_api_behavior(old_present, new_present):
    tree = ast.parse((ROOT / 'TienKung-Lab/legged_lab/mdp/rewards.py').read_text())
    block = next(n for n in tree.body if isinstance(n, ast.If)
                 and 'quat_rotate_inverse' in ast.unparse(n.test))
    old = lambda q, v: ('old', q, v)
    new = lambda q, v: ('new', q, v)
    math = SimpleNamespace()
    if old_present:
        math.quat_rotate_inverse = old
    if new_present:
        math.quat_apply_inverse = new
    exec(compile(ast.Module(body=[block], type_ignores=[]), 'reward_compatibility', 'exec'), {'math_utils': math})
    if new_present:
        assert math.quat_apply_inverse is new
    elif old_present:
        assert math.quat_apply_inverse('q', 'v') == ('old', 'q', 'v')
    else:
        assert not hasattr(math, 'quat_apply_inverse')
