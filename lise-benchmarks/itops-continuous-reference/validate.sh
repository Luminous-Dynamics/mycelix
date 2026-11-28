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
  . as $root
  | $root.suite_id == "itops-continuous-scenario-reference"
  and $root.schema_version == "0.1.0"
  and $root.contract == "ITOPS-CONTINUOUS-001"
  and $root.visibility == "PUBLIC_REFERENCE_SCENARIO"
  and ($root.scenario.scenario_id | length) > 0
  and ($root.scenario.time_profile.horizon_seconds >= 3600)
  and ($root.scenario.time_profile.tick_seconds > 0)
  and ($root.scenario.time_profile.seed >= 0)
  and ($root.scenario.services | length) >= 3
  and ([ $root.scenario.services[].service_id ] | length) == ([ $root.scenario.services[].service_id ] | unique | length)
  and ([ $root.scenario.operator_observations[].observation_id ] | length) == ([ $root.scenario.operator_observations[].observation_id ] | unique | length)
  and ([ $root.scenario.fault_schedule[].event_id ] | length) == ([ $root.scenario.fault_schedule[].event_id ] | unique | length)
  and ([ $root.scenario.terminal_propositions[].proposition_id ] | length) == ([ $root.scenario.terminal_propositions[].proposition_id ] | unique | length)
  and ($root.scenario.fault_schedule | length) >= 8
  and ([ $root.scenario.fault_schedule[].family ] | unique | length) >= 6
  and all($root.scenario.operator_observations[];
    .agent_visible == true
    and .observed_at_seconds <= $root.scenario.time_profile.horizon_seconds
  )
  and all($root.scenario.fault_schedule[];
    .agent_visible == false
    and .start_seconds >= 0
    and .duration_seconds > 0
    and (.start_seconds + .duration_seconds) <= $root.scenario.time_profile.horizon_seconds
  )
  and all($root.scenario.fault_schedule[];
    . as $event
    | any($root.scenario.services[]; .service_id == $event.target_service_id)
    and ($event.affected_observation_ids | length) > 0
    and all($event.affected_observation_ids[];
      . as $obs
      | any($root.scenario.operator_observations[]; .observation_id == $obs)
    )
  )
  and all($root.scenario.dependencies[];
    . as $edge
    | $edge.from_service != $edge.to_service
    and any($root.scenario.services[]; .service_id == $edge.from_service)
    and any($root.scenario.services[]; .service_id == $edge.to_service)
  )
  and ([ $root.scenario.dependencies[] | "\(.from_service)->\(.to_service):\(.dependency_kind)" ] | length)
      == ([ $root.scenario.dependencies[] | "\(.from_service)->\(.to_service):\(.dependency_kind)" ] | unique | length)
  and all($root.scenario.services[];
    . as $service
    | all($service.depends_on[];
      . as $dependency
      | any($root.scenario.services[]; .service_id == $dependency)
      and any($root.scenario.dependencies[];
        .from_service == $service.service_id and .to_service == $dependency
      )
    )
  )
  and all($root.scenario.operator_observations[];
    . as $obs
    | any($root.scenario.services[]; .service_id == $obs.target_service_id)
  )
  and any($root.scenario.fault_schedule[];
    . as $a
    | any($root.scenario.fault_schedule[];
      . as $b
      | $b.event_id != $a.event_id
      and $b.start_seconds < ($a.start_seconds + $a.duration_seconds)
      and $a.start_seconds < ($b.start_seconds + $b.duration_seconds)
    )
  )
  and $root.scenario.agent_payload_policy.exposes_fault_schedule == false
  and $root.scenario.agent_payload_policy.exposes_oracle == false
  and $root.scenario.agent_payload_policy.observations_available_only_after_observed_time == true
  and all($root.scenario.terminal_propositions[];
    .independent_verification_required == true
  )
  and all(["term-availability", "term-latency", "term-unauthorized", "term-unresolved"][];
    . as $required
    | any($root.scenario.terminal_propositions[]; .proposition_id == $required)
  )
' "$CORPUS" >/dev/null

echo "PASS: ITOPS continuous public scenario satisfies structural and temporal invariants."
echo "schema_sha256=$(sha256sum "$SCHEMA" | awk '{print $1}')"
echo "corpus_sha256=$(sha256sum "$CORPUS" | awk '{print $1}')"
