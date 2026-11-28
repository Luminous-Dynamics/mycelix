#!/usr/bin/env bash
# Dependency-light semantic validator for the public ITOPS continuous scenario.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$ROOT/itops-continuous-reference"
SCHEMA="$DIR/itops-continuous-scenario-v0.1.schema.json"
CORPUS="${ITOPS_CONTINUOUS_CORPUS_PATH:-$DIR/itops-continuous-scenario-v0.1.json}"

command -v jq >/dev/null || { echo "ERROR: jq is required" >&2; exit 2; }
test -s "$SCHEMA"
test -s "$CORPUS"

jq -e '
  .["$schema"] == "https://json-schema.org/draft/2020-12/schema"
  and .type == "object"
  and .additionalProperties == false
  and .["$defs"].scenario.type == "object"
' "$SCHEMA" >/dev/null

jq -e '
  .suite_id == "itops-continuous-scenario-reference"
  and .schema_version == "0.1.0"
  and .contract == "ITOPS-CONTINUOUS-001"
  and .visibility == "PUBLIC_REFERENCE_SCENARIO"
  and (.scenario.scenario_id | length) > 0
  and (.scenario.time_profile.horizon_seconds >= 3600)
  and (.scenario.time_profile.tick_seconds > 0)
  and (.scenario.time_profile.seed >= 0)
  and (.scenario.services | length) >= 3
  and ([.scenario.services[].service_id] | length) == ([.scenario.services[].service_id] | unique | length)
  and ([.scenario.operator_observations[].observation_id] | length) == ([.scenario.operator_observations[].observation_id] | unique | length)
  and ([.scenario.fault_schedule[].event_id] | length) == ([.scenario.fault_schedule[].event_id] | unique | length)
  and ([.scenario.terminal_propositions[].proposition_id] | length) == ([.scenario.terminal_propositions[].proposition_id] | unique | length)
  and (.scenario.fault_schedule | length) >= 8
  and ([.scenario.fault_schedule[].family] | unique | length) >= 6
  and all(.scenario.operator_observations[]; .agent_visible == true and .observed_at_seconds <= .scenario.time_profile.horizon_seconds)
  and all(.scenario.fault_schedule[];
    .agent_visible == false
    and .start_seconds >= 0
    and .duration_seconds > 0
    and (.start_seconds + .duration_seconds) <= .scenario.time_profile.horizon_seconds
  )
  and all(.scenario.fault_schedule[].target_service_id as $target; any(.scenario.services[]; .service_id == $target))
  and all(.scenario.dependencies[];
    any(.scenario.services[]; .service_id == .from_service)
    and any(.scenario.services[]; .service_id == .to_service)
  )
  and all(.scenario.operator_observations[].target_service_id as $target; any(.scenario.services[]; .service_id == $target))
  and all(.scenario.fault_schedule[].affected_observation_ids[] as $obs; any(.scenario.operator_observations[]; .observation_id == $obs))
  and any(.scenario.fault_schedule[] as $a;
    any(.scenario.fault_schedule[];
      .event_id != $a.event_id
      and .start_seconds < ($a.start_seconds + $a.duration_seconds)
      and $a.start_seconds < (.start_seconds + .duration_seconds)
    )
  )
  and .scenario.agent_payload_policy.exposes_fault_schedule == false
  and .scenario.agent_payload_policy.exposes_oracle == false
  and .scenario.agent_payload_policy.observations_available_only_after_observed_time == true
  and all(.scenario.terminal_propositions[]; .independent_verification_required == true)
' "$CORPUS" >/dev/null

echo "PASS: ITOPS continuous public scenario satisfies structural and temporal invariants."
echo "schema_sha256=$(sha256sum "$SCHEMA" | awk '{print $1}')"
echo "corpus_sha256=$(sha256sum "$CORPUS" | awk '{print $1}')"
