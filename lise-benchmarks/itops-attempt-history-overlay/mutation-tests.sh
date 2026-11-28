#!/usr/bin/env bash
# Mutations that must be rejected by the attempt-history fixture gate.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DIR="$ROOT/lise-benchmarks/itops-attempt-history-overlay"
CORPUS="$DIR/attempt-history-cases-v0.1.json"
TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT

expect_rejected() {
  local name="$1" filter="$2"
  jq "$filter" "$CORPUS" > "$TMP"
  if ITOPS_ATTEMPT_HISTORY_CORPUS_PATH="$TMP" bash "$DIR/validate.sh" >/dev/null 2>&1; then
    echo "FAIL: accepted mutation: $name" >&2
    exit 1
  fi
  echo "PASS: rejected mutation: $name"
}

expect_rejected "retry without attempt history" \
  '(.cases[] | select(.case_id == "ITOPS-AH-005") | .attempts) = []'
expect_rejected "provider acknowledgement promoted to no-effect proof" \
  '(.cases[] | select(.case_id == "ITOPS-AH-005") | .evidence[0].kind) = "provider_ack"'
expect_rejected "retry with substituted idempotency key" \
  '(.cases[] | select(.case_id == "ITOPS-AH-005") | .retry_basis.idempotency_key) = "other-key"'
expect_rejected "retry with mismatched request digest" \
  '(.cases[] | select(.case_id == "ITOPS-AH-005") | .retry_basis.request_digest) = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"'
expect_rejected "shared dependency falsely promoted to independent" \
  '(.cases[] | select(.case_id == "ITOPS-AH-007") | .evidence[0].independence) = "qualified_independent" | (.cases[] | select(.case_id == "ITOPS-AH-007") | .evidence[0].dependency_group) = "observer-domain-new"'
expect_rejected "history retention no longer covers original attempt" \
  '(.cases[] | select(.case_id == "ITOPS-AH-005") | .history.retention_covers_start) = false'
expect_rejected "late event rewrites earlier knowledge-time decision" \
  '(.cases[] | select(.case_id == "ITOPS-AH-012") | .prior_frontier.expected_next_step) = "DO_NOT_RETRY"'
expect_rejected "blind retry relabelled as ordinary success" \
  '(.cases[] | select(.case_id == "ITOPS-AH-008") | .expected_next_step) = "DO_NOT_RETRY" | (.cases[] | select(.case_id == "ITOPS-AH-008") | .hard_gate_violations) = []'
expect_rejected "broken attempt chain repaired without changing expected disposition" \
  '(.cases[] | select(.case_id == "ITOPS-AH-011") | .attempts[1].ordinal) = 2 | (.cases[] | select(.case_id == "ITOPS-AH-011") | .attempts[1].previous_attempt_id) = "attempt-1"'
expect_rejected "partial multi-action unknown suffix treated as complete" \
  '(.cases[] | select(.case_id == "ITOPS-AH-016") | .attempts[1].effect) = "applied" | (.cases[] | select(.case_id == "ITOPS-AH-016") | .evidence[1].result) = "applied"'

expect_rejected "partial multi-action suffix mislabelled as completed" \
  '(.cases[] | select(.case_id == "ITOPS-AH-016") | .expected_next_step) = "DO_NOT_RETRY"'

echo "PASS: all 11 attempt-history mutations were rejected."
