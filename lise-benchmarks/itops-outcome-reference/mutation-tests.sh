#!/usr/bin/env bash
# Mutation checks: the public fixture validator must reject altered claims.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DIR="$ROOT/lise-benchmarks/itops-outcome-reference"
CORPUS="$DIR/itops-outcome-cases-v0.1.json"
TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT

expect_rejected() {
  local name="$1"
  local filter="$2"
  jq "$filter" "$CORPUS" > "$TMP"
  if ITOPS_CORPUS_PATH="$TMP" bash "$DIR/validate.sh" >/dev/null 2>&1; then
    echo "FAIL: validator accepted mutation: $name" >&2
    exit 1
  fi
  echo "PASS: rejected mutation: $name"
}

# Removing a required action must not shrink the expected-operation denominator.
expect_rejected "missing expected action" \
  '(.cases[] | select(.case_id == "ITOPS-OR-006") | .actions) |= map(select(.operation_id != "op-006-b"))'

# A terminal functional regression invalidates AcceptedRecovery.
expect_rejected "accepted recovery with failed functional check" \
  '(.cases[] | select(.case_id == "ITOPS-OR-002") | .terminal_observation.functional_checks_pass) = false'

# A shared dependency cannot be promoted to independent verification.
expect_rejected "shared observer promoted to independent evidence" \
  '(.cases[] | select(.case_id == "ITOPS-OR-002") | .evidence[0].independence) = "shared_dependency"'

# Action evidence must support the exact operation it purports to verify.
expect_rejected "evidence detached from operation" \
  '(.cases[] | select(.case_id == "ITOPS-OR-002") | .evidence[0].operation_ids) = []'

# An out-of-scope effect cannot be relabelled as an accepted recovery.
expect_rejected "unsafe mutation relabelled as success" \
  '(.cases[] | select(.case_id == "ITOPS-OR-010") | .expected_disposition) = "AcceptedRecovery"'

# A coordinated scope shrink must not make per-operation checks vacuously true.
expect_rejected "accepted recovery with an empty work denominator" '(.cases[] | select(.case_id == "ITOPS-OR-002") | .expected_operation_ids) = [] | (.cases[] | select(.case_id == "ITOPS-OR-002") | .actions) = [] | (.cases[] | select(.case_id == "ITOPS-OR-002") | .evidence[].operation_ids) = []'

echo "PASS: all 6 mutation controls were rejected."
