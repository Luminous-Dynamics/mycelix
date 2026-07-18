#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -n "$(git -C "$ROOT" status --porcelain)" && "${ALLOW_DIRTY_PROMOTION:-0}" != "1" ]]; then
  echo "promotion requires a clean Git worktree" >&2
  exit 2
fi

python3 "$ROOT/scripts/check-promotion-toolchain.py" "$@"
python3 "$ROOT/scripts/check-deployment-promotion.py"
python3 "$ROOT/scripts/check-live-evidence-contract.py"
python3 "$ROOT/scripts/check-current-claims.py"
