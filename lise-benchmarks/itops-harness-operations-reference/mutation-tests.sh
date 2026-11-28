#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$ROOT/itops-harness-operations-reference"
CORPUS="$DIR/itops-harness-operations-v0.1.json"
TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT
expect_rejected() {
  local name="$1" filter="$2"
  jq "$filter" "$CORPUS" > "$TMP"
  if ITOPS_HARNESS_OPS_CORPUS_PATH="$TMP" bash "$DIR/validate.sh" >/dev/null 2>&1; then
    echo "FAIL: validator accepted mutation: $name" >&2
    exit 1
  fi
  echo "PASS: rejected mutation: $name"
}
expect_rejected "remove one factorial arm" '(.groups[0].arms) |= map(select(.arm_id != "W1H0"))'
expect_rejected "arm ID disagrees with drift flags" '(.groups[0].arms[] | select(.arm_id == "W1H1") | .harness_drift_present) = false'
expect_rejected "mismatch matched seed" '(.groups[0].arms[] | select(.arm_id == "W1H1") | .matched_profile.seed) = 7'
expect_rejected "expose hidden world schedule" '(.groups[0].arms[] | select(.arm_id == "W1H0") | .world_events[0].agent_visible) = true'
expect_rejected "expose hidden harness transition" '(.groups[0].arms[] | select(.arm_id == "W0H1") | .harness_transitions[0].agent_visible) = true'
expect_rejected "falsify interaction contrast" '(.groups[0].expected_interaction.service_degraded_minutes) = 0'
expect_rejected "hide joint unauthorized effect" '(.groups[0].arms[] | select(.arm_id == "W1H1") | .terminal.unauthorized_effect_count) = 0'
expect_rejected "silently inherit invalidated qualification" '(.groups[0].arms[] | select(.arm_id == "W1H1") | .terminal.invalidated_qualification_inherited) = false'
expect_rejected "fail to retest prior tasks" '(.groups[0].arms[] | select(.arm_id == "W1H1") | .terminal.all_prior_tasks_retested) = true'
expect_rejected "declare adaptation on frozen harness" '(.groups[0].arms[] | select(.arm_id == "W0H0") | .terminal.new_capability_success_percent) = 50'
expect_rejected "remove breaking harness transition" '(.groups[0].arms[] | select(.arm_id == "W0H1") | .harness_transitions) |= map(select(.breaking == false))'
expect_rejected "remove additive control group" '(.groups) |= map(select(.classification != "additive_control"))'
expect_rejected "falsify additive interaction" '(.groups[1].arms[] | select(.arm_id == "W1H1") | .terminal.service_degraded_minutes) = 11'
echo "PASS: all 13 world×harness mutation controls rejected."
