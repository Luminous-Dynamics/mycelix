#!/usr/bin/env bash
# Offline semantic gate for the public ITOPS attempt-history reference fixtures.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DIR="$ROOT/lise-benchmarks/itops-attempt-history-overlay"
SCHEMA="$DIR/attempt-history-cases-v0.1.schema.json"
CORPUS="${ITOPS_ATTEMPT_HISTORY_CORPUS_PATH:-$DIR/attempt-history-cases-v0.1.json}"

command -v jq >/dev/null || { echo "ERROR: jq is required" >&2; exit 2; }
test -s "$SCHEMA" && test -s "$CORPUS"
jq -e '.["$schema"] == "https://json-schema.org/draft/2020-12/schema" and .type == "object" and .additionalProperties == false and .["$defs"].case.type == "object"' "$SCHEMA" >/dev/null
jq -e . "$CORPUS" >/dev/null

jq -e '
  def history_complete:
    .history.status == "COMPLETE_FOR_SCOPE"
    and .history.scope_matches == true
    and .history.cursor_complete == true
    and .history.gap_count == 0
    and .history.retention_covers_start == true
    and .history.backfill_pending == false;

  def evidence_for($action; $attempt_id; $result):
    . as $c
    | any($c.evidence[];
        .action_id == $action
        and .attempt_id == $attempt_id
        and .request_digest == $c.operation.request_digest
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
        and .attempt_id == $c.attempts[0].attempt_id
        and .result == "not_applied"
      )
      and $c.retry_basis.idempotency_state == "VERIFIED_SAME_KEY"
      and $c.retry_basis.request_digest == $c.operation.request_digest
      and $c.retry_basis.idempotency_key == $c.operation.idempotency_key
      and ($c.attempts | length) == 1
      and ($c.attempts[0].effect == "unknown" or $c.attempts[0].effect == "not_applied")
      and ($c.attempts[0].action_id == $c.operation.actions[0].action_id)
      and ($c.history.status == "COMPLETE_FOR_SCOPE")
      and ($c.history.scope_matches == true)
      and ($c.history.cursor_complete == true)
      and ($c.history.gap_count == 0)
      and ($c.history.retention_covers_start == true)
      and ($c.history.backfill_pending == false);

  def chain_invalid:
    . as $c
    | ($c.attempts | sort_by(.ordinal)) as $as
    | (([$as[].attempt_id] | length) != ([$as[].attempt_id] | unique | length))
      or (([$as[].ordinal]) != [range(1; ($as | length) + 1)])
      or any($as[];
        . as $attempt
        | ($c.operation.actions | map(.action_id) | index($attempt.action_id)) == null
      )
      or any($as[];
        .request_digest != $c.operation.request_digest
        or .idempotency_key != $c.operation.idempotency_key
      )
      or any(range(0; ($as | length));
        . as $i
        | if $i == 0 then $as[$i].previous_attempt_id != null
          else $as[$i].previous_attempt_id != $as[$i - 1].attempt_id
          end
      );

  def duplicate_effect_retry:
    . as $c
    | any($c.operation.actions[];
        .action_id as $action
        | ([$c.attempts[] | select(.action_id == $action)] | sort_by(.ordinal)) as $chain
        | ($chain | length) > 1
          and $chain[0].effect == "applied"
          and $chain[1].dispatched == true
          and $chain[1].effect == "applied"
          and ($c | evidence_for($action; $chain[0].attempt_id; "applied"))
          and ($c | evidence_for($action; $chain[1].attempt_id; "applied"))
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
  and (.cases | length) == 17
  and ([.cases[].case_id] | length) == ([.cases[].case_id] | unique | length)
  and all(.cases[];
    . as $c
    | ($c.operation.actions | length) > 0
      and (([$c.operation.actions[].action_id] | length) == ([$c.operation.actions[].action_id] | unique | length))
      and (([$c.operation.actions[].sequence] | sort) == [range(0; ($c.operation.actions | length))])
      and all($c.attempts[];
        . as $attempt
        | ($c.operation.actions | map(.action_id) | index($attempt.action_id)) != null
      )
      and (([$c.evidence[].evidence_id] | length) == ([$c.evidence[].evidence_id] | unique | length))
      and all($c.evidence[];
        . as $ev
        | ($c.operation.actions | map(.action_id) | index($ev.action_id)) != null
          and $ev.request_digest == $c.operation.request_digest
          and any($c.attempts[]; .attempt_id == $ev.attempt_id and .action_id == $ev.action_id)
          and $ev.event_at <= $ev.observed_at
          and $ev.observed_at <= $c.history.as_of
      )
      and all($c.attempts[];
        .attempted_at <= .recorded_at
        and .recorded_at <= $c.history.as_of
      )
      and (
        if $c.expected_next_step == "START_INITIAL_ATTEMPT" then
          ($c | history_complete) and ($c.attempts | length) == 0
        elif $c.expected_next_step == "DO_NOT_RETRY" then
          ($c | history_complete)
          and all($c.operation.actions[];
            .action_id as $action
            | any($c.attempts[];
                . as $attempt
                | $attempt.action_id == $action
                  and $attempt.effect == "applied"
                  and ($c | evidence_for($action; $attempt.attempt_id; "applied"))
              )
          )
          and all($c.attempts[];
            .effect != "unknown" and .effect != "partial"
          )
          and (not ($c | blind_retry))
          and (not ($c | duplicate_effect_retry))
          and (
            if $c.prior_frontier == null then true
            else $c.prior_frontier.status != "COMPLETE_FOR_SCOPE"
              and $c.prior_frontier.expected_next_step == "HOLD_FOR_RECONCILIATION"
            end
          )
        elif $c.expected_next_step == "HOLD_FOR_RECONCILIATION" then
          (not ($c | history_complete))
          or any($c.attempts[]; .effect == "partial")
          or any($c.attempts[]; .effect == "unknown") and (not ($c | retry_basis_valid))
          or any($c.evidence[];
            .result == "conflicting"
            or .independence == "shared_dependency"
            or .independence == "unknown"
            or .freshness != "current"
            or .kind == "provider_ack"
          )
          or ($c.retry_basis.no_effect_evidence_id != null and not ($c | retry_basis_valid))
        elif $c.expected_next_step == "RETRY_SAME_REQUEST_KEY" then
          ($c | history_complete) and ($c | retry_basis_valid)
        elif $c.expected_next_step == "RESUME_UNATTEMPTED_SUFFIX" then
          ($c | history_complete)
          and ($c.operation.actions | length) > 1
          and any($c.attempts[]; .effect == "applied")
          and any($c.operation.actions[];
            .action_id as $action
            | ([$c.attempts[] | select(.action_id == $action)] | length) == 0
          )
          and all($c.attempts[]; .effect == "applied" or .effect == "not_applied")
          and all($c.attempts[];
            . as $attempt
            | $attempt.effect == "applied"
              and ($c | evidence_for($attempt.action_id; $attempt.attempt_id; "applied"))
          )
        elif $c.expected_next_step == "REJECT_BLIND_RETRY" then
          (
            (($c | blind_retry) and ($c.hard_gate_violations | index("blind_retry_after_unknown")) != null)
            or
            (($c | duplicate_effect_retry) and ($c.hard_gate_violations | index("duplicate_effect")) != null)
          )
        elif $c.expected_next_step == "INVALID_HISTORY" then
          ($c | chain_invalid)
        else false end
      )
      and (
        if $c.expected_next_step == "REJECT_BLIND_RETRY" then ($c.hard_gate_violations | length) > 0
        else ($c.hard_gate_violations | length) == 0 end
      )
      and (
        if $c.expected_next_step == "INVALID_HISTORY" then ($c | chain_invalid)
        else (not ($c | chain_invalid)) end
      )
      and (
        if $c.prior_frontier == null then true
        else $c.prior_frontier.status != "COMPLETE_FOR_SCOPE"
          and $c.prior_frontier.expected_next_step == "HOLD_FOR_RECONCILIATION"
          and $c.prior_frontier.as_of <= $c.history.as_of
          and (($c.prior_frontier.known_attempt_ids) == ([$c.attempts[] | select(.recorded_at <= $c.prior_frontier.as_of) | .attempt_id]))
          and (($c.prior_frontier.known_evidence_ids) == ([$c.evidence[] | select(.observed_at <= $c.prior_frontier.as_of) | .evidence_id]))
        end
      )
  )
' "$CORPUS" >/dev/null

echo "PASS: 16 ITOPS attempt-history reference cases satisfy the semantic fixture gate."
echo "schema_sha256=$(sha256sum "$SCHEMA" | cut -d' ' -f1)"
echo "corpus_sha256=$(sha256sum "$CORPUS" | cut -d' ' -f1)"
