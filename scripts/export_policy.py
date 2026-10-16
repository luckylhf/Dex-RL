"""Export one Dex TorchScript policy and verify its ONNX numerical equivalence."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
import onnx
import onnxruntime as ort
import torch

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--task', choices=['dex_walk', 'dex_run'], default='dex_walk')
parser.add_argument('--policy', type=Path)
parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/exported_policy/policy.onnx')
args = parser.parse_args()
if args.policy is None:
    if args.task == 'dex_run':
        parser.error('run requires an explicit matching --policy')
    args.policy = ROOT / 'Deploy_Tienkung/policy/walk_amp/model/policy.pt'
torch.set_num_threads(1)
policy = torch.jit.load(str(args.policy), map_location='cpu').eval()
samples = np.random.default_rng(42).normal(0, 1, (32, 840)).astype(np.float32)
with torch.inference_mode():
    reference = policy(torch.from_numpy(samples)).numpy()
if reference.shape != (32, 23) or not np.isfinite(reference).all():
    raise ValueError('Expected a finite 840-input / 23-output Dex policy')
args.output.parent.mkdir(parents=True, exist_ok=True)
if args.output.exists() and not args.output.with_name('supplied_policy.onnx').exists():
    shutil.copy2(args.output, args.output.with_name('supplied_policy.onnx'))
torch.onnx.export(policy, torch.zeros(1, 840), str(args.output), input_names=['obs'],
                  output_names=['actions'], opset_version=17)
model = onnx.load(str(args.output))
metadata = {
    'expected_task': args.task,
    'policy_sha256': hashlib.sha256(args.policy.read_bytes()).hexdigest(),
    'urdf_sha256': hashlib.sha256((ROOT / 'TienKung-Lab/legged_lab/assets/Tienkung_Dex/urdf/tiangong3_locomotion.urdf').read_bytes()).hexdigest(),
    'deploy_config_sha256': hashlib.sha256((ROOT / 'Deploy_Tienkung/policy/walk_amp/config/walk_amp.yaml').read_bytes()).hexdigest(),
}
onnx.helper.set_model_props(model, metadata)
onnx.checker.check_model(model)
onnx.save(model, str(args.output))
options = ort.SessionOptions()
options.intra_op_num_threads = 1
session = ort.InferenceSession(str(args.output), options, providers=['CPUExecutionProvider'])
result = np.concatenate([session.run(None, {'obs': row[None]})[0] for row in samples])
np.testing.assert_allclose(result, reference, rtol=1e-4, atol=1e-5)
report = dict(metadata, samples=32, max_abs_error=float(np.max(np.abs(result-reference))),
              pt=str(args.policy.relative_to(ROOT)) if args.policy.is_relative_to(ROOT) else str(args.policy),
              onnx=str(args.output))
(ROOT / 'artifacts').mkdir(exist_ok=True)
(ROOT / 'artifacts/policy_equivalence.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))

