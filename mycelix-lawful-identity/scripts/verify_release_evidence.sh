#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
adapter="${MYCELIX_REAL_CONDUCTOR_ADAPTER:-$root/tests/conductor/real_conductor_adapter.py}"
evidence_root="${MYCELIX_REAL_CONDUCTOR_EVIDENCE_ROOT:-}"

if [[ ! -x "$adapter" ]]; then
  echo "real-conductor adapter is not executable: $adapter" >&2
  exit 2
fi
if [[ -z "$evidence_root" ]]; then
  echo "MYCELIX_REAL_CONDUCTOR_EVIDENCE_ROOT must name an empty evidence directory" >&2
  exit 2
fi
if [[ ! -d "$evidence_root" ]]; then
  echo "real-conductor evidence root is not a directory: $evidence_root" >&2
  exit 2
fi
if [[ -L "$evidence_root" ]]; then
  echo "real-conductor evidence root may not be a symlink: $evidence_root" >&2
  exit 2
fi
if find "$evidence_root" -mindepth 1 -print -quit | grep -q .; then
  echo "real-conductor evidence root must start empty: $evidence_root" >&2
  exit 2
fi

resolved_evidence_root="$(cd "$evidence_root" && pwd -P)"
if [[ -n "${MYCELIX_CONDUCTOR_EVIDENCE_ROOT:-}" ]] \
  && [[ "$(cd "$MYCELIX_CONDUCTOR_EVIDENCE_ROOT" && pwd -P)" != "$resolved_evidence_root" ]]; then
  echo "MYCELIX_CONDUCTOR_EVIDENCE_ROOT disagrees with the release evidence root" >&2
  exit 2
fi
export MYCELIX_CONDUCTOR_EVIDENCE_ROOT="$resolved_evidence_root"

cd "$root"
scripts/verify_model_evidence.sh
python3 tests/conductor/run_scenarios.py \
  --adapter "$adapter" \
  --require-real \
  --evidence-root "$resolved_evidence_root"
python3 tests/conductor/release_evidence_index.py \
  --evidence-root "$resolved_evidence_root"
python3 tests/conductor/release_evidence_index.py \
  --evidence-root "$resolved_evidence_root" \
  --verify

echo "lawful-identity release evidence gate: PASS"
echo "retained_evidence_root=$resolved_evidence_root"
