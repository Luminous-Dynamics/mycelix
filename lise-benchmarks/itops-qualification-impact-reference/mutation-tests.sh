#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$ROOT/itops-qualification-impact-reference"
CORPUS="$DIR/itops-qualification-impact-v0.1.json"
TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT
expect_rejected() {
  local name="$1" filter="$2"
  jq "$filter" "$CORPUS" > "$TMP"
  if ITOPS_QUAL_IMPACT_CORPUS_PATH="$TMP" bash "$DIR/validate.sh" >/dev/null 2>&1; then
    echo "FAIL: validator accepted mutation: $name" >&2
    exit 1
  fi
  echo "PASS: rejected mutation: $name"
}
expect_rejected "tool change incorrectly marked unaffected" '(.cases[] | select(.case_id == "ITOPS-QI-TOOL-SCHEMA-001") | .claims[] | select(.claim_id == "repair_payments") | .expected_qualification_disposition) = "NOT_AFFECTED"'
expect_rejected "unrelated capability unnecessarily requalified" '(.cases[] | select(.case_id == "ITOPS-QI-TOOL-SCHEMA-001") | .claims[] | select(.claim_id == "read_catalog") | .expected_qualification_disposition) = "REQUALIFICATION_REQUIRED"'
expect_rejected "revoked operation still authorized" '(.cases[] | select(.case_id == "ITOPS-QI-PERMISSION-REVOKE-001") | .effect_authority.expected_decision) = "AUTHORIZED"'
expect_rejected "permission revocation falsely invalidates competence" '(.cases[] | select(.case_id == "ITOPS-QI-PERMISSION-REVOKE-001") | .claims[] | select(.claim_id == "repair_payments") | .expected_qualification_disposition) = "REQUALIFICATION_REQUIRED"'
expect_rejected "observer gap promoted to current recovery evidence" '(.cases[] | select(.case_id == "ITOPS-QI-OBSERVER-DRIFT-001") | .outcome_evidence.expected_status) = "CURRENT"'
expect_rejected "evaluator change scores pooled as comparable" '(.cases[] | select(.case_id == "ITOPS-QI-EVALUATOR-CHANGE-001") | .benchmark_score.expected_comparability) = "COMPARABLE"'
expect_rejected "incomplete change coverage treated as unaffected" '(.cases[] | select(.case_id == "ITOPS-QI-INCOMPLETE-COVERAGE-001") | .claims[] | select(.claim_id == "delegated_payments_repair") | .expected_qualification_disposition) = "NOT_AFFECTED"'
expect_rejected "new capability inherits qualification automatically" '(.cases[] | select(.case_id == "ITOPS-QI-COMPATIBLE-ADD-001") | .claims[] | select(.claim_id == "query_orders") | .expected_qualification_disposition) = "NOT_AFFECTED"'
expect_rejected "score protocol digest silently rewritten" '(.cases[] | select(.case_id == "ITOPS-QI-EVALUATOR-CHANGE-001") | .benchmark_score.current_protocol_digest) = "sha256:itops-eval-v0.1"'
expect_rejected "duplicate claim identity" '(.cases[] | select(.case_id == "ITOPS-QI-TOOL-SCHEMA-001") | .claims[1].claim_id) = "repair_payments"'
expect_rejected "required observer identity removed" '(.cases[] | select(.case_id == "ITOPS-QI-OBSERVER-DRIFT-001") | .outcome_evidence.observer_ref) = null'
expect_rejected "applicable authority operation missing" '(.cases[] | select(.case_id == "ITOPS-QI-PERMISSION-REVOKE-001") | .effect_authority.operation_ref) = null'
expect_rejected "scored result missing protocol identity" '(.cases[] | select(.case_id == "ITOPS-QI-EVALUATOR-CHANGE-001") | .benchmark_score.current_protocol_digest) = null'
expect_rejected "known dependency change hidden by incomplete feed" '(.cases[] | select(.case_id == "ITOPS-QI-INCOMPLETE-COVERAGE-001") | .claims[] | select(.claim_id == "repair_payments") | .expected_qualification_disposition) = "UNDER_INVESTIGATION"
expect_rejected "unknown change coverage accepted as complete" '(.cases[] | select(.case_id == "ITOPS-QI-INCOMPLETE-COVERAGE-001") | .change_feed_complete) = true'
expect_rejected "unknown impact incorrectly treated as unaffected" '(.cases[] | select(.case_id == "ITOPS-QI-INCOMPLETE-COVERAGE-001") | .claims[] | select(.claim_id == "delegated_payments_repair") | .expected_qualification_disposition) = "NOT_AFFECTED"'
expect_rejected "observer change event points to a different observer" '(.cases[] | select(.case_id == "ITOPS-QI-OBSERVER-DRIFT-001") | .outcome_evidence.observer_ref) = "observer:wrong"'
expect_rejected "observer coverage and change both erased to promote evidence" '(.cases[] | select(.case_id == "ITOPS-QI-OBSERVER-DRIFT-001") | .outcome_evidence) |= (.observer_changed = false | .coverage_complete = true)'
echo "PASS: all 18 orthogonal-impact mutation controls rejected."
