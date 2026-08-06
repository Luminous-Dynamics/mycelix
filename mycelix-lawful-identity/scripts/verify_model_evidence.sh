#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

python3 scripts/verify_source_consistency.py
python3 conformance/verify_scoped_verifier_run_v1.py
python3 tests/protocol_model/settlement_conflict_model.py
PYTHONOPTIMIZE=2 python3 tests/protocol_model/settlement_conflict_model.py
python3 tests/conductor/validate_scenarios.py
python3 tests/conductor/run_scenarios.py --adapter tests/conductor/model_adapter.py
optimized_state="$(PYTHONOPTIMIZE=2 python3 tests/conductor/model_adapter.py \
  --scenario policy-lease-expiry)"
python3 - "$optimized_state" <<'PY_CHECK'
import json
import sys
value = json.loads(sys.argv[1])
if value.get("observed_state") != "PolicyExpired":
    raise SystemExit("optimized model adapter lost policy-expiry state")
PY_CHECK
python3 -m unittest \
  tests/conductor/test_evidence_manifest.py \
  tests/conductor/test_real_conductor_adapter.py \
  tests/conductor/test_release_evidence_index.py

echo "lawful-identity model evidence gate: PASS"
