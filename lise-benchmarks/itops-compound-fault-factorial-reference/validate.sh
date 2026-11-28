#!/usr/bin/env bash
# Validate matched 2x2 causal-interaction known-answer reference groups.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$ROOT/itops-compound-fault-factorial-reference"
SCHEMA="$DIR/itops-compound-fault-factorial-v0.1.schema.json"
CORPUS="${ITOPS_COMPOUND_CORPUS_PATH:-$DIR/itops-compound-fault-factorial-v0.1.json}"

command -v jq >/dev/null || { echo "ERROR: jq is required" >&2; exit 2; }
test -s "$SCHEMA"
test -s "$CORPUS"

jq -e '
  .["$schema"] == "https://json-schema.org/draft/2020-12/schema"
  and .type == "object"
  and .additionalProperties == false
  and .["$defs"].group.type == "object"
' "$SCHEMA" >/dev/null

jq -e '
  . as $root
  | $root.suite_id == "itops-compound-fault-factorial-reference"
  and $root.schema_version == "0.1.0"
  and $root.contract == "ITOPS-CONTINUOUS-001"
  and $root.visibility == "PUBLIC_REFERENCE_ORACLE_FIXTURE"
  and ($root.groups | length) == 2
  and ([ $root.groups[].group_id ] | length) == ([ $root.groups[].group_id ] | unique | length)
  and all($root.groups[];
    . as $g
    | ($g.arms | length) == 4
    and ([ $g.arms[].arm_id ] | sort) == ["A0B0","A0B1","A1B0","A1B1"]
    and ([ $g.arms[] | "\(.factor_a_present):\(.factor_b_present)" ] | sort)
        == ["false:false","false:true","true:false","true:true"]
    and all($g.arms[]; .matched_profile == $g.arms[0].matched_profile)
    and ($g.agent_payload_policy.exposes_factor_assignments == false)
    and ($g.agent_payload_policy.exposes_terminal_oracle == false)
    and ($g.expected_causal_factor_ids | length) == 2
    and ($g.required_remediation_action_ids | length) >= 2
    and (
      ([ $g.arms[] | select(.factor_a_present == false and .factor_b_present == false) | .terminal.unrecovered_degradation_minutes ][0])
      as $y00
      | ([ $g.arms[] | select(.factor_a_present == true and .factor_b_present == false) | .terminal.unrecovered_degradation_minutes ][0])
      as $y10
      | ([ $g.arms[] | select(.factor_a_present == false and .factor_b_present == true) | .terminal.unrecovered_degradation_minutes ][0])
      as $y01
      | ([ $g.arms[] | select(.factor_a_present == true and .factor_b_present == true) | .terminal.unrecovered_degradation_minutes ][0])
      as $y11
      | ($y11 - $y10 - $y01 + $y00) == $g.expected_interaction_contrast
    )
    and (
      if $g.classification == "synergistic_interaction" then
        $g.expected_interaction_contrast > 0
        and any($g.arms[]; .factor_a_present == true and .factor_b_present == true and .terminal.fault_persists_after_trigger_removed == true and .terminal.functional_contract_pass == false)
        and all($g.arms[] | select(.factor_a_present != true or .factor_b_present != true); .terminal.fault_persists_after_trigger_removed == false)
      else
        $g.expected_interaction_contrast == 0
        and all($g.arms[]; .terminal.fault_persists_after_trigger_removed == false)
      end
    )
  )
' "$CORPUS" >/dev/null

echo "PASS: matched 2x2 compound-fault factorial controls satisfy their known-answer contract."
