#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="$ROOT/manifest.json"
CASES="$ROOT/cases.json"
INTEGRATION_MAP="$ROOT/integration-map.json"

command -v jq >/dev/null 2>&1 || { echo "jq is required" >&2; exit 127; }

if ! jq -e '
  .benchmark_id == "CRAFT-WATER-SERVICE-REPAIR-V1"
  and .version == "1.0.0-draft"
  and .data_classification == "synthetic-only"
  and .no_real_personal_data == true
  and ((.public_baseline_sources | type) == "array")
  and ((.public_baseline_sources | length) == 5)
  and ((.claims_not_established | type) == "array")
  and ((.claims_not_established | length) >= 4)
' "$MANIFEST" >/dev/null; then
  echo "Manifest validation failed: identity, classification, source count, or claim boundary." >&2
  exit 1
fi

if ! jq -e '
  .benchmark_id == "CRAFT-WATER-SERVICE-REPAIR-V1"
  and .schema_version == "1.0.0"
  and .synthetic_only == true
  and ((.cases | type) == "array")
  and ((.cases | length) == 16)
  and (([.cases[].case_id] | unique | length) == 16)
  and all(.cases[];
    ((.case_id | type) == "string")
    and (.case_id | test("^WSR-[0-9]{3}$"))
    and ((.title | type) == "string")
    and ((.title | length) > 0)
    and ((.input | type) == "object")
    and ((.expected | type) == "object")
    and (.expected.disposition as $d | [
      "not_eligible",
      "eligible_for_local_review",
      "revalidation_required",
      "execution_or_closure_blocked",
      "new_workflow_required",
      "alternative_service_inadequate",
      "drinking_water_incident_response_required",
      "extended_alternative_supply_monitoring_required",
      "measurement_reconciliation_required"
    ] | index($d) != null)
    and ((.expected.reason_codes | type) == "array")
    and ((.expected.reason_codes | length) > 0)
    and (.input.context_digest | type == "string" and test("^[0-9a-f]{64}$"))
    and ((.input.expected_actions // 0) >= (.input.observed_actions // 0))
    and all((.input.evidence // [])[];
      ((.source_id | type) == "string")
      and ((.source_id | length) > 0)
      and (.digest | type == "string" and test("^[0-9a-f]{64}$"))
      and (.context_digest | type == "string" and test("^[0-9a-f]{64}$"))
      and ((.independence_group | type) == "string")
      and ((.independence_group | length) > 0)
    )
    and (if .case_id == "WSR-010" then
      (.input.authorized_work_scope.scope_id == "WORK-SCOPE-SYN-010")
      and (.input.authorized_work_scope.authorization_state == "approved")
      and (.input.authorized_work_scope.expected_action_count == .input.expected_actions)
      and (.input.authorized_work_scope.scope_digest_kind == "synthetic_test_placeholder")
      and (.input.authorized_work_scope.scope_digest | type == "string" and test("^[0-9a-f]{64}$"))
    else true end)
  )
' "$CASES" >/dev/null; then
  echo "Case validation failed: one or more of the 16 frozen WSR scenarios violate the schema." >&2
  jq -r '
    .cases[]
    | select(
        ((.case_id | type) != "string")
        or ((.case_id | test("^WSR-[0-9]{3}$")) | not)
        or ((.input.context_digest | type) != "string")
        or ((.input.context_digest | test("^[0-9a-f]{64}$")) | not)
        or ((.expected.reason_codes | type) != "array")
        or ((.expected.reason_codes | length) == 0)
        or ((.input.expected_actions // 0) < (.input.observed_actions // 0))
      )
    | .case_id
  ' "$CASES" >&2
  exit 1
fi

if ! jq -n -e --slurpfile cases "$CASES" --slurpfile mapping "$INTEGRATION_MAP" '
  $mapping[0].benchmark_id == "CRAFT-WATER-SERVICE-REPAIR-V1-INTEGRATION-MAP"
  and $mapping[0].schema_version == "1.0.0"
  and (($mapping[0].mappings | type) == "array")
  and (($mapping[0].mappings | length) == 16)
  and (($mapping[0].mappings | map(.case_id) | sort) == ($cases[0].cases | map(.case_id) | sort))
  and all($mapping[0].mappings[];
    ((.owner_issues | type) == "array")
    and ((.owner_issues | length) > 0)
    and all(.owner_issues[]; (type == "string") and test("^#[0-9]+$"))
    and ((.checks | type) == "array")
    and ((.checks | length) > 0)
    and ((.owner_boundary | type) == "string")
    and ((.owner_boundary | length) > 0)
  )
' >/dev/null; then
  echo "Integration-map validation failed: each scenario must map to its exact owner issue and boundary." >&2
  exit 1
fi

bash "$ROOT/validate-metamorphic.sh"

echo "Water Service Repair V1: manifest, 16 synthetic cases, 16 cross-core owner mappings, and 20 proposed metamorphic vectors structurally valid; cross-core execution remains pending."
