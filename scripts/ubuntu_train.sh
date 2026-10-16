#!/usr/bin/env bash
# Use the V4 environment, then the training root required by the AMP data paths.
set -eo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$script_dir/ubuntu_env.sh" bash -c '
cd TienKung-Lab
exec python legged_lab/scripts/train.py "$@"
' bash "$@"
