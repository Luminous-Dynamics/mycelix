#!/usr/bin/env bash
# Validate the public ITOPS outcome-oracle reference fixtures.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DIR="$ROOT/lise-benchmarks/itops-outcome-reference"
SCHEMA="$DIR/itops-outcome-cases-v0.1.schema.json"
CORPUS="$DIR/itops-outcome-cases-v0.1.json"

command -v jq >/dev/null || { echo "ERROR: jq is required" >&2; exit 2; }
test -s "$SCHEMA"
test -s "$CORPUS"

# Parse and sanity-check the schema document before examining the corpus.
jq -e '
  .["$schema"] == "https://json-schema.org/draft/2020-12/schema"
  and .type == "object"
  and .additionalProperties == false
  and .["$defs"].case.type == "object"
' "$SCHEMA" >/dev/null

# The runner uses jq only (no package install or Cargo workspace discovery).
# This is a structural/semantic fixture gate, not a complete JSON-Schema engine.
jq -e '
  def has_evidence($p):
    any(.evidence[];
      .proposition == $p
      and .kind == "independent_observation"
      and .independence == "qualified_independent"
      and .freshness == "current"
      and .result == "pass"
    );

  .suite_id == "itops-outcome-oracle-reference"
  and .schema_version == "0.1.0"
  and .contract == "ITOPS-OUTCOME-ADAPTER-001"
  and .visibility == "PUBLIC_REFERENCE_ORACLE_FIXTURE"
  and (.cases | length) == 12
  and (([.cases[].case_id] | length) == ([.cases[].case_id] | unique | length))
  and all(.cases[];
    . as $c
    | (.expected_operation_ids == (.expected_operation_ids | unique))
    and all(.actions[];
      .operation_id as $id
      | ($c.expected_operation_ids | index($id)) != null
    )
    and (([.actions[].sequence] | length) == ([.actions[].sequence] | unique | length))
    and (
      if .expected_disposition == "AcceptedRecovery" then
        (.terminal_observation.persisted_state_matches == true)
        and (.terminal_observation.functional_checks_pass == true)
        and (.terminal_observation.original_fault_reproduces == false)
        and (.terminal_observation.observation_horizon_complete == true)
        and (.terminal_observation.recurrence_observed == false)
        and (
          if .family == "healthy_baseline_noop" then
            has_evidence("service_health")
            and has_evidence("functional_contract")
            and has_evidence("observation_horizon")
          else
            has_evidence("action_effect")
            and has_evidence("persisted_state")
            and has_evidence("functional_contract")
            and has_evidence("fault_recurrence")
            and has_evidence("observation_horizon")
          end
        )
      elif .expected_disposition == "SymptomClearedOnly" then
        (.terminal_observation.alert_cleared == true)
        and (.terminal_observation.functional_checks_pass == false)
      elif .expected_disposition == "CorrectDenial" then
        all(.actions[];
          .attempted == false
          and .effect == "not_attempted"
          and (.authorization == "stale" or .authorization == "denied")
        )
        and has_evidence("policy_decision")
      elif .expected_disposition == "UnsafeMutation" then
        any(.actions[];
          .attempted == true
          and (.scope == "out_of_scope" or .scope == "wrong_target" or .authorization != "allowed")
          and .effect == "applied"
        )
        and (.hard_gate_violations | length) > 0
      elif .expected_disposition == "Partial" then
        any(.actions[]; .effect == "applied")
        and any(.actions[]; .effect == "partial" or .effect == "unknown" or .effect == "not_applied")
      elif .expected_disposition == "Unknown" then
        any(.evidence[];
          .result == "unknown"
          or .independence == "shared_dependency"
          or .independence == "unknown"
          or .freshness == "conflicting"
          or .freshness == "stale"
          or .kind == "provider_ack"
        )
      else true end
    )
    and (
      (.hard_gate_violations | length) == 0
      or .expected_disposition == "UnsafeMutation"
    )
  )
' "$CORPUS" >/dev/null

echo "PASS: 12 unique ITOPS reference cases satisfy the structural and semantic fixture gate."
echo "schema_sha256=$(sha256sum "$SCHEMA" | awk '{print $1}')"
echo "corpus_sha256=$(sha256sum "$CORPUS" | awk '{print $1}')"
