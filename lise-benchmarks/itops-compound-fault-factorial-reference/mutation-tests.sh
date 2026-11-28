#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$ROOT/itops-compound-fault-factorial-reference"
CORPUS="$DIR/itops-compound-fault-factorial-v0.1.json"
TMP="$(mktemp)"; trap 'rm -f "$TMP"' EXIT
expect_rejected() {
  local name="$1" filter="$2"
  jq "$filter" "$CORPUS" > "$TMP"
  if ITOPS_COMPOUND_CORPUS_PATH="$TMP" bash "$DIR/validate.sh" >/dev/null 2>&1; then
    echo "FAIL: validator accepted mutation: $name" >&2; exit 1
  fi
  echo "PASS: rejected mutation: $name"
}
expect_rejected "remove one factorial arm" '(.groups[0].arms) |= map(select(.arm_id != "A1B0"))'
expect_rejected "falsify compound interaction contrast" '(.groups[0].arms[] | select(.arm_id == "A1B1") | .terminal.unrecovered_degradation_minutes) = 0'
expect_rejected "change expected contrast to a false value" '(.groups[0].expected_interaction_contrast) = 0'
expect_rejected "mismatch workload seed in one arm" '(.groups[0].arms[] | select(.arm_id == "A1B1") | .matched_profile.seed) = 7'
expect_rejected "expose factor assignments to agent" '(.groups[0].agent_payload_policy.exposes_factor_assignments) = true'
expect_rejected "remove one required causal factor" '(.groups[0].expected_causal_factor_ids) |= map(select(. != "cpu_capacity_constraint"))'
expect_rejected "hide metastable persistence on joint arm" '(.groups[0].arms[] | select(.arm_id == "A1B1") | .terminal.fault_persists_after_trigger_removed) = false'
expect_rejected "relabel synergistic group as additive" '(.groups[0].classification) = "additive_control"'
expect_rejected "remove required controlled restart remedy" '(.groups[0].required_remediation_action_ids) |= map(select(. != "controlled_restart"))'
echo "PASS: all 9 compound-interaction mutation controls were rejected."
