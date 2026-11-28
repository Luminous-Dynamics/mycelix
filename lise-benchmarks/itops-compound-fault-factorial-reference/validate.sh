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
  def arm($g; $id):
    [ $g.arms[] | select(.arm_id == $id) ][0];
  def y($g; $id):
    arm($g; $id).terminal.service_degraded_minutes;

  . as $root
  | $root.suite_id == "itops-compound-fault-factorial-reference"
  and $root.schema_version == "0.1.0"
  and $root.contract == "ITOPS-CONTINUOUS-001"
  and $root.visibility == "PUBLIC_REFERENCE_ORACLE_FIXTURE"
  and ($root.groups | length) == 2
  and ([ $root.groups[].group_id ] | sort)
      == ["ITOPS-CAUSAL-ADDITIVE-CONTROL-001", "ITOPS-CAUSAL-METASTABLE-001"]
  and ([ $root.groups[].classification ] | sort)
      == ["additive_control", "synergistic_interaction"]
  and all($root.groups[];
    . as $g
    | ($g.arms | length) == 4
    and ([ $g.arms[].arm_id ] | sort) == ["A0B0","A0B1","A1B0","A1B1"]
    and ([ $g.arms[] | "\(.factor_a_present):\(.factor_b_present)" ] | sort)
        == ["false:false","false:true","true:false","true:true"]
    and all($g.arms[];
      . as $arm
      | $arm.arm_id ==
        ((if $arm.factor_a_present then "A1" else "A0" end)
         + (if $arm.factor_b_present then "B1" else "B0" end))
    )
    and all($g.arms[]; .matched_profile == $g.arms[0].matched_profile)
    and ($g.factor_a.factor_id != $g.factor_b.factor_id)
    and ([ $g.expected_causal_factor_ids[] ] | sort)
        == ([ $g.factor_a.factor_id, $g.factor_b.factor_id ] | sort)
    and ($g.agent_payload_policy.exposes_factor_assignments == false)
    and ($g.agent_payload_policy.exposes_terminal_oracle == false)
    and ($g.interaction_metric == "service_degraded_minutes")
    and (y($g;"A1B1") - y($g;"A1B0") - y($g;"A0B1") + y($g;"A0B0"))
        == $g.expected_interaction_contrast
    and (
      if $g.classification == "synergistic_interaction" then
        $g.group_id == "ITOPS-CAUSAL-METASTABLE-001"
        and $g.expected_interaction_contrast > 0
        and arm($g;"A0B0").terminal.fault_persists_after_trigger_removed == false
        and arm($g;"A1B0").terminal.fault_persists_after_trigger_removed == false
        and arm($g;"A0B1").terminal.fault_persists_after_trigger_removed == false
        and arm($g;"A1B1").terminal.fault_persists_after_trigger_removed == true
        and arm($g;"A1B1").terminal.functional_contract_pass == false
        and all(["A0B0","A1B0","A0B1"][];
          . as $id | arm($g;$id).terminal.functional_contract_pass == true
        )
        and ([ $g.required_remediation_action_ids[] ] | sort)
            == ["controlled_restart","raise_cpu_quota","restore_retry_policy"]
      else
        $g.group_id == "ITOPS-CAUSAL-ADDITIVE-CONTROL-001"
        and $g.expected_interaction_contrast == 0
        and all($g.arms[];
          .terminal.fault_persists_after_trigger_removed == false
          and .terminal.functional_contract_pass == true
        )
        and ([ $g.required_remediation_action_ids[] ] | sort)
            == ["restore_cache_capacity","restore_dns_latency"]
      end
    )
  )
' "$CORPUS" >/dev/null

echo "PASS: matched 2x2 compound-fault known-answer groups satisfy causal-interaction, arm, remedy and terminal-state invariants."
