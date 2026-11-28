#!/usr/bin/env bash
# Mutation controls for the ITOPS continuous scenario validator.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$ROOT/itops-continuous-reference"
CORPUS="$DIR/itops-continuous-scenario-v0.1.json"
TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT

expect_rejected() {
  local name="$1" filter="$2"
  jq "$filter" "$CORPUS" > "$TMP"
  if ITOPS_CONTINUOUS_CORPUS_PATH="$TMP" bash "$DIR/validate.sh" >/dev/null 2>&1; then
    echo "FAIL: validator accepted mutation: $name" >&2
    exit 1
  fi
  echo "PASS: rejected mutation: $name"
}

expect_rejected "fault schedule disclosed to agent" \
  '(.scenario.fault_schedule[0].agent_visible) = true'

expect_rejected "fault targets unknown service" \
  '(.scenario.fault_schedule[1].target_service_id) = "unregistered-service"'

expect_rejected "fault extends beyond scenario horizon" \
  '(.scenario.fault_schedule[0].start_seconds) = 7190'

expect_rejected "remove required service substrate" \
  '(.scenario.services) |= map(select(.service_id != "payments"))'

expect_rejected "remove required injected-event population" \
  '(.scenario.fault_schedule) = []'

expect_rejected "remove independent terminal proposition" \
  '(.scenario.terminal_propositions[0].independent_verification_required) = false'

echo "PASS: all 6 ITOPS continuous mutation controls were rejected."
