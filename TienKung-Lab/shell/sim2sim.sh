#!/usr/bin/env bash

set -euo pipefail

robot="${1:-dex}"
gait="${2:-walk}"
duration="${DURATION:-1000}"
policy_override="${POLICY:-}"
model_override="${MODEL:-}"

case "$robot" in
    dex)
        task="dex_${gait}"
        default_model="../xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml"
        case "$gait" in
            walk)
                default_policy="../Deploy_Tienkung/policy/walk_amp/model/policy.pt"
                ;;
            run)
                default_policy=""
                ;;
            *)
                echo "Unsupported gait: $gait" >&2
                echo "Usage: $0 [dex] [walk|run]" >&2
                exit 1
                ;;
        esac
        ;;
    *)
        echo "Unsupported robot: $robot" >&2
        echo "Usage: $0 [dex] [walk|run]" >&2
        exit 1
        ;;
esac

policy="${policy_override:-$default_policy}"
model="${model_override:-$default_model}"

if [[ -z "$policy" ]]; then
    echo "No default policy configured for $task." >&2
    echo "Set POLICY=/path/to/policy.pt and rerun, for example:" >&2
    echo "  POLICY=logs/${task}/.../exported/policy.pt $0 $robot $gait" >&2
    exit 1
fi

python legged_lab/scripts/sim2sim.py \
    --task "$task" \
    --policy "$policy" \
    --duration "$duration" \
    --model "$model"
