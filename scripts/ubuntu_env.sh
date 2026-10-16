#!/usr/bin/env bash
# Run from this V4 checkout using the existing Ubuntu environment.
set -eo pipefail
if [ "$#" -eq 0 ]; then
    echo 'Usage: bash scripts/ubuntu_env.sh COMMAND [ARG ...]' >&2
    exit 2
fi
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${CONDA_PREFIX:-}" != "$HOME/miniconda3/envs/env_isaaclab" ]]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
    conda activate env_isaaclab
fi
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONUNBUFFERED=1
# Training-only checkout: the ROS workspace (ros_ws/install) is no longer part of this tree.
cd "$project_root"
exec "$@"
