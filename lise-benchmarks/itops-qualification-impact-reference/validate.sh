#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$ROOT/itops-qualification-impact-reference"
SCHEMA="$DIR/itops-qualification-impact-v0.1.schema.json"
CORPUS="${ITOPS_QUAL_IMPACT_CORPUS_PATH:-$DIR/itops-qualification-impact-v0.1.json}"
command -v jq >/dev/null || { echo "ERROR: jq is required" >&2; exit 2; }
command -v python3 >/dev/null || { echo "ERROR: python3 is required" >&2; exit 2; }
test -s "$SCHEMA"
test -s "$CORPUS"
python3 "$DIR/validate-json-schema.py" "$SCHEMA" "$CORPUS"
jq -e '
  def intersects($a; $b): any($a[]?; . as $item | any($b[]?; . == $item));
  def expected_qualification($c; $claim):
    if $claim.new_capability then "NEW_CAPABILITY_UNQUALIFIED"
    elif intersects($claim.dependency_artifact_refs; $c.changed_artifact_refs) then "REQUALIFICATION_REQUIRED"
    elif (($c.change_feed_complete | not) and $claim.exposed_to_unknown_change) then "UNDER_INVESTIGATION"
    else "NOT_AFFECTED" end;
  def expected_authority($c):
    if ($c.effect_authority.applicable | not) then "NOT_APPLICABLE"
    elif $c.effect_authority.permission_revoked then "DENIED"
    else "AUTHORIZED" end;
  def expected_observation($c):
    if ($c.outcome_evidence.required | not) then "NOT_APPLICABLE"
    elif ($c.outcome_evidence.observer_changed or ($c.outcome_evidence.coverage_complete | not)) then "INDETERMINATE"
    else "CURRENT" end;
  def expected_score($c):
    if ($c.benchmark_score.scored | not) then "NOT_APPLICABLE"
    elif $c.benchmark_score.baseline_protocol_digest != $c.benchmark_score.current_protocol_digest then "INCOMPARABLE"
    else "COMPARABLE" end;
  . as $root
  | $root.suite_id == "itops-qualification-impact-reference"
  and $root.schema_version == "0.1.0"
  and $root.contract == "ITOPS-QUAL-IMPACT-001"
  and $root.visibility == "PUBLIC_REFERENCE_ORACLE_FIXTURE"
  and ($root.cases | length) == 8
  and ([ $root.cases[].case_id ] | length) == ([ $root.cases[].case_id ] | unique | length)
  and all($root.cases[];
    . as $c
    | (if $c.effect_authority.applicable then
         (($c.effect_authority.operation_ref | type) == "string" and ($c.effect_authority.operation_ref | length) > 0)
       else
         ($c.effect_authority.operation_ref == null and $c.effect_authority.permission_revoked == false)
       end)
    and (if $c.outcome_evidence.required then
           (($c.outcome_evidence.observer_ref | type) == "string" and ($c.outcome_evidence.observer_ref | length) > 0)
         else
           ($c.outcome_evidence.observer_ref == null and $c.outcome_evidence.observer_changed == false)
         end)
    and (if $c.benchmark_score.scored then
           (($c.benchmark_score.baseline_protocol_digest | type) == "string" and ($c.benchmark_score.baseline_protocol_digest | length) > 0
            and ($c.benchmark_score.current_protocol_digest | type) == "string" and ($c.benchmark_score.current_protocol_digest | length) > 0)
         else
           ($c.benchmark_score.baseline_protocol_digest == null and $c.benchmark_score.current_protocol_digest == null)
         end)
    and ($c.claims | length) > 0
    and ([ $c.claims[].claim_id ] | length) == ([ $c.claims[].claim_id ] | unique | length)
    and all($c.claims[]; . as $claim | $claim.expected_qualification_disposition == expected_qualification($c; $claim))
    and $c.effect_authority.expected_decision == expected_authority($c)
    and $c.outcome_evidence.expected_status == expected_observation($c)
    and (if $c.outcome_evidence.observer_changed then
           any($c.changed_artifact_refs[]; . == $c.outcome_evidence.observer_ref)
         else true end)
    and $c.benchmark_score.expected_comparability == expected_score($c)
  )
  and ([ $root.cases[] | select(.case_id == "ITOPS-QI-PERMISSION-REVOKE-001") ][0]
    | .effect_authority.expected_decision == "DENIED"
    and all(.claims[]; .expected_qualification_disposition == "NOT_AFFECTED"))
  and ([ $root.cases[] | select(.case_id == "ITOPS-QI-OBSERVER-DRIFT-001") ][0]
    | .outcome_evidence.expected_status == "INDETERMINATE"
    and all(.claims[]; .expected_qualification_disposition == "NOT_AFFECTED"))
  and ([ $root.cases[] | select(.case_id == "ITOPS-QI-EVALUATOR-CHANGE-001") ][0]
    | .benchmark_score.expected_comparability == "INCOMPARABLE"
    and all(.claims[]; .expected_qualification_disposition == "NOT_AFFECTED"))
  and ([ $root.cases[] | select(.case_id == "ITOPS-QI-INCOMPLETE-COVERAGE-001") ][0]
    | any(.claims[]; .expected_qualification_disposition == "UNDER_INVESTIGATION")
    and .outcome_evidence.expected_status == "INDETERMINATE")
' "$CORPUS" >/dev/null
echo "PASS: eight orthogonal impact cases preserve qualification, authority, observer evidence and score comparability as separate outcomes."
echo "schema_sha256=$(sha256sum "$SCHEMA" | awk '{print $1}')"
echo "corpus_sha256=$(sha256sum "$CORPUS" | awk '{print $1}')"
