"""Start one detached Ubuntu job; retain its command, PID, output and exit code."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

if len(sys.argv) == 3 and sys.argv[1] == '--worker':
    directory = Path(sys.argv[2])
    job = json.loads((directory / 'job.json').read_text())
    completed = subprocess.run(['bash', str(ROOT / 'scripts/ubuntu_env.sh'), *job['command']], cwd=ROOT)
    result = {'returncode': completed.returncode,
              'finished_at': datetime.now(timezone.utc).isoformat()}
    temporary = directory / 'result.json.tmp'
    temporary.write_text(json.dumps(result, indent=2)+'\n')
    temporary.replace(directory / 'result.json')
    raise SystemExit(completed.returncode)

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('label')
parser.add_argument('command', nargs=argparse.REMAINDER)
args = parser.parse_args()
if not re.fullmatch(r'[A-Za-z0-9_-]+', args.label):
    parser.error('label must contain only letters, digits, underscores or hyphens')
if not args.command:
    parser.error('a command is required')
started = datetime.now(timezone.utc)
job_id = started.strftime('%Y%m%dT%H%M%S%fZ') + '_' + args.label
directory = ROOT / 'artifacts/remote_runs' / job_id
directory.mkdir(parents=True)
job = {'job_id': job_id, 'started_at': started.isoformat(), 'cwd': str(ROOT),
       'environment': 'env_isaaclab', 'command': args.command}
(directory / 'job.json').write_text(json.dumps(job, indent=2)+'\n')
with (directory / 'output.log').open('wb') as output:
    worker = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--worker', str(directory)],
                              cwd=ROOT, stdin=subprocess.DEVNULL, stdout=output,
                              stderr=subprocess.STDOUT, start_new_session=True)
(directory / 'pid.txt').write_text(str(worker.pid)+'\n')
print(json.dumps({'job_id': job_id, 'directory': str(directory), 'pid': worker.pid}))
