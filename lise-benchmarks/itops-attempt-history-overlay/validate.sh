#!/usr/bin/env bash
# Offline semantic gate for the public ITOPS attempt-history reference fixtures.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DIR="$ROOT/lise-benchmarks/itops-attempt-history-overlay"
SCHEMA="$DIR/attempt-history-cases-v0.1.schema.json"
CORPUS="${ITOPS_ATTEMPT_HISTORY_CORPUS_PATH:-$DIR/attempt-history-cases-v0.1.json}"

command -v jq >/dev/null || { echo "ERROR: jq is required" >&2; exit 2; }
test -s "$SCHEMA" && test -s "$CORPUS"
jq -e '.[ "$schema" ] == "https://json-schema.org/draft/2020-12/schema" and .type == "object" and .additionalProperties == false and .["$defs"].case.type == "object"' "$SCHEMA" >/dev/null

jq -e '
  def history_complete:
    .history.status == "COMPLETE_FOR_SCOPE"
    and .history.scope_matches == true
    and .history.cursor_complete == true
    and .history.gap_count == 0
    and .history.retention_covers_start == true
    and .history.backfill_pending == false;

  def evidence_for($action; $result):
    any(.evidence[];
      .action_id == $action
      and .request_digest == .operation.request_digest
      and (.kind == "independent_resource_observation" or .kind == "owner_reconciliation")
      and .independence == "qualified_independent"
      and .freshness == "current"
      and .result == $result
    );

  def retry_basis_valid:
    . as $c
    | ($c.retry_basis.no_effect_evidence_id) as $id
    | ($c.operation.actions | length) == 1
      and ($id != null)
      and any($c.evidence[];
        .evidence_id == $id
        and .action_id == $c.operation.actions[0].action_id
        and .request_digest == $c.operation.request_digest
        and (.kind == "independent_resource_observation" or .kind == "owner_reconciliation")
        and .independence == "qualified_independent"
        and .freshness == "current"
        and .result == "not_applied"
      )
      and $c.retry_basis.idempotency_state == "VERIFIED_SAME_KEY"
      and $c.retry_basis.request_digest == $c.operation.request_digest
      and $c.retry_basis.idempotency_key == $c.operation.idempotency_key
      and ($c.attempts | length) == 1
      and ($c.attempts[0].effect == "unknown" or $c.attempts[0].effect == "not_applied")
      and ($c.attempts[0].action_id == $c.operation.actions[0].action_id);

  def chain_invalid:
    . as $c
    | ($c.attempts | sort_by(.ordinal)) as $as
    | (([$as[].attempt_id] | length) != ([$as[].attempt_id] | unique | length))
      or (([$as[].ordinal]) != [range(1; ($as | length) + 1)])
      or any($as[]; .request_digest != $c.operation.request_digest or .idempotency_key != $c.operation.idempotency_key)
      or any($as[]; ($c.operation.actions | map(.action_id) | index(.action_id)) == null)
      or any(range(0; ($as | length));
        . as $i
        | if $i == 0 then $as[$i].previous_attempt_id != null
          else $as[$i].previous_attempt_id != $as[$i - 1].attempt_id
          end
      );

  def blind_retry:
    . as $c
    | any($c.operation.actions[];
      .action_id as $action
      | ([$c.attempts[] | select(.action_id == $action)] | sort_by(.ordinal)) as $chain
      | ($chain | length) > 1
        and $chain[0].effect == "unknown"
        and $chain[1].dispatched == true
    );

  .suite_id == "itops-attempt-history-overlay"
  and .schema_version == "0.1.0"
  and .contract == "ITOPS-OUTCOME-ADAPTER-001"
  and .visibility == "PUBLIC_REFERENCE_ORACLE_FIXTURE"
  and (.cases | length) == 16
  and ([.cases[].case_id] | length) == ([.cases[].case_id] | unique | length)
  and all(.cases[];
    . as $c
    | (.operation.actions | length) > 0
      and (([$c.operation.actions[].action_id] | length) == ([$c.operation.actions[].action_id] | unique | length))
      and (([$c.operation.actions[].sequence] | sort) == [range(0; ($c.operation.actions | length))])
      and all($c.attempts[]; ($c.operation.actions | map(.action_id) | index(.action_id)) != null)
      and (([$c.evidence[].evidence_id] | length) == ([$c.evidence[].evidence_id] | unique | length))
      and all($c.evidence[]; ($c.operation.actions | map(.action_id) | index(.action_id)) != null and .request_digest == $c.operation.request_digest)
      and (
        if .expected_next_step == "START_INITIAL_ATTEMPT" then
          history_complete and (.attempts | length) == 0
        elif .expected_next_step == "DO_NOT_RETRY" then
          history_complete
          and any(.attempts[]; .effect == "applied")
          and any(.operation.actions[]; evidence_for(.action_id; "applied"))
          and (
            if .prior_frontier == null then true
            else .prior_frontier.status != "COMPLETE_FOR_SCOPE"
              and .prior_frontier.expected_next_step == "HOLD_FOR_RECONCILIATION"
            end
          )
        elif .expected_next_step == "HOLD_FOR_RECONCILIATION" then
          (not history_complete)
          or any(.attempts[]; .effect == "unknown" or .effect == "partial")
          or any(.evidence[]; .result == "conflicting" or .independence == "shared_dependency" or .freshness != "current")
          or (not retry_basis_valid)
        elif .expected_next_step == "RETRY_SAME_REQUEST_KEY" then
          history_complete and retry_basis_valid
        elif .expected_next_step == "RESUME_UNATTEMPTED_SUFFIX" then
          history_complete
          and (.operation.actions | length) > 1
          and any(.attempts[]; .effect == "applied")
          and any(.operation.actions[]; .action_id as $a | ([$c.attempts[] | select(.action_id == $a)] | length) == 0)
          and all(.attempts[]; .effect == "applied" or .effect == "not_applied")
          and all(.attempts[]; evidence_for(.action_id; "applied"))
        elif .expected_next_step == "REJECT_BLIND_RETRY" then
          blind_retry
          and (.hard_gate_violations | index("blind_retry_after_unknown")) != null
        elif .expected_next_step == "INVALID_HISTORY" then
          chain_invalid
        else false end
      )
      and (
        if .expected_next_step == "REJECT_BLIND_RETRY" then (.hard_gate_violations | length) > 0
        else (.hard_gate_violations | length) == 0 end
      )
      and (
        if .prior_frontier == null then true
        else .prior_frontier.status != "COMPLETE_FOR_SCOPE"
          and .prior_frontier.expected_next_step == "HOLD_FOR_RECONCILIATION"
          and (.prior_frontier.known_attempt_ids | length) <= (.attempts | length)
        end
      )
  )
' "$CORPUS" >/dev/null

echo "PASS: 16 ITOPS attempt-history reference cases satisfy the semantic fixture gate."
echo "schema_sha256=$(sha256sum "$SCHEMA" | awk "{print \$1}")"
echo "corpus_sha256=$(sha256sum "$CORPUS" | awk "{print \$1}")"
