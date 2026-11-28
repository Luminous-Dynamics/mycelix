#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$ROOT/itops-harness-operations-reference"
SCHEMA="$DIR/itops-harness-operations-v0.1.schema.json"
CORPUS="${ITOPS_HARNESS_OPS_CORPUS_PATH:-$DIR/itops-harness-operations-v0.1.json}"
command -v jq >/dev/null || { echo "ERROR: jq is required" >&2; exit 2; }
command -v python3 >/dev/null || { echo "ERROR: python3 is required for Draft 2020-12 validation" >&2; exit 2; }
test -s "$SCHEMA"
test -s "$CORPUS"
python3 "$DIR/validate-json-schema.py" "$SCHEMA" "$CORPUS"
jq -e '.["$schema"] == "https://json-schema.org/draft/2020-12/schema" and .type == "object" and .additionalProperties == false and .["$defs"].group.type == "object"' "$SCHEMA" >/dev/null
jq -e '
  def arm($g; $id): [ $g.arms[] | select(.arm_id == $id) ][0];
  def y($g; $id): arm($g; $id).terminal.service_degraded_minutes;
  def r($g; $id): arm($g; $id).terminal.prior_task_retention_percent;
  . as $root
  | $root.suite_id == "itops-harness-operations-reference"
  and $root.schema_version == "0.1.0"
  and $root.contract == "ITOPS-HARNESS-OPS-001"
  and $root.visibility == "PUBLIC_REFERENCE_ORACLE_FIXTURE"
  and $root.agent_payload_policy.exposes_world_events == false
  and $root.agent_payload_policy.exposes_harness_transitions == false
  and $root.agent_payload_policy.exposes_terminal_oracle == false
  and $root.agent_payload_policy.observations_time_gated == true
  and ($root.groups | length) == 2
  and ([ $root.groups[].group_id ] | sort) == ["ITOPS-HARNESS-ADDITIVE-CONTROL-001", "ITOPS-HARNESS-SYNERGY-001"]
  and ([ $root.groups[].classification ] | sort) == ["additive_control", "synergistic_interaction"]
  and all($root.groups[];
    . as $g
    | ($g.arms | length) == 4
    and ([ $g.arms[].arm_id ] | sort) == ["W0H0","W0H1","W1H0","W1H1"]
    and all($g.arms[];
      . as $a
      | $a.arm_id == ((if $a.world_drift_present then "W1" else "W0" end) + (if $a.harness_drift_present then "H1" else "H0" end))
      and (($a.world_events | length) > 0) == $a.world_drift_present
      and (($a.harness_transitions | length) > 0) == $a.harness_drift_present
      and all($a.world_events[]; .agent_visible == false and (.event_id | length) > 0)
      and ([ $a.world_events[].event_id ] | length) == ([ $a.world_events[].event_id ] | unique | length)
      and all($a.harness_transitions[]; .agent_visible == false and (.transition_id | length) > 0 and (.required_requalification == .breaking))
      and ([ $a.harness_transitions[].transition_id ] | length) == ([ $a.harness_transitions[].transition_id ] | unique | length)
      and (if $a.harness_drift_present then any($a.harness_transitions[]; .breaking == true) and any($a.harness_transitions[]; .breaking == false) and $a.terminal.new_capability_success_percent != null else $a.terminal.new_capability_success_percent == null end)
      and ($a.terminal.service_degraded_minutes >= 0)
      and ($a.terminal.prior_task_retention_percent >= 0 and $a.terminal.prior_task_retention_percent <= 100)
      and ($a.terminal.unauthorized_effect_count >= 0)
    )
    and all($g.arms[]; .matched_profile == $g.arms[0].matched_profile)
    and (
      if $g.classification == "synergistic_interaction" then
        $g.group_id == "ITOPS-HARNESS-SYNERGY-001"
        and (y($g;"W1H1") - y($g;"W1H0") - y($g;"W0H1") + y($g;"W0H0")) == $g.expected_interaction.service_degraded_minutes
        and $g.expected_interaction.service_degraded_minutes > 0
        and (r($g;"W1H1") - r($g;"W1H0") - r($g;"W0H1") + r($g;"W0H0")) == $g.expected_interaction.prior_task_retention_percent
        and $g.expected_interaction.prior_task_retention_percent < 0
        and arm($g;"W1H1").terminal.unauthorized_effect_count > 0
        and arm($g;"W1H1").terminal.invalidated_qualification_inherited == true
        and arm($g;"W1H1").terminal.all_prior_tasks_retested == false
        and arm($g;"W1H1").terminal.functional_contract_pass == false
        and all(["W0H0","W1H0","W0H1"][]; . as $id | arm($g;$id).terminal.unauthorized_effect_count == 0 and arm($g;$id).terminal.invalidated_qualification_inherited == false and arm($g;$id).terminal.all_prior_tasks_retested == true and arm($g;$id).terminal.functional_contract_pass == true)
      else
        $g.group_id == "ITOPS-HARNESS-ADDITIVE-CONTROL-001"
        and (y($g;"W1H1") - y($g;"W1H0") - y($g;"W0H1") + y($g;"W0H0")) == $g.expected_interaction.service_degraded_minutes
        and $g.expected_interaction.service_degraded_minutes == 0
        and (r($g;"W1H1") - r($g;"W1H0") - r($g;"W0H1") + r($g;"W0H0")) == $g.expected_interaction.prior_task_retention_percent
        and $g.expected_interaction.prior_task_retention_percent == 0
        and all($g.arms[]; .terminal.unauthorized_effect_count == 0 and .terminal.invalidated_qualification_inherited == false and .terminal.all_prior_tasks_retested == true and .terminal.functional_contract_pass == true)
      end
    )
  )
' "$CORPUS" >/dev/null
echo "PASS: world×harness factorial arms, matched profiles, hidden schedules, interaction contrasts and hard-gate known answers."
echo "schema_sha256=$(sha256sum "$SCHEMA" | awk '{print $1}')"
echo "corpus_sha256=$(sha256sum "$CORPUS" | awk '{print $1}')"
