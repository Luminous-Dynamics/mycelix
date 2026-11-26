#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="$ROOT/manifest.json"
CASES="$ROOT/cases.json"
INTEGRATION_MAP="$ROOT/integration-map.json"

command -v jq >/dev/null 2>&1 || { echo "jq is required" >&2; exit 127; }
jq -e . "$MANIFEST" >/dev/null
jq -e . "$CASES" >/dev/null
jq -e . "$INTEGRATION_MAP" >/dev/null

jq -e '
  .benchmark_id == "CRAFT-WATER-SERVICE-REPAIR-V1"
  and .version == "1.0.0-draft"
  and .data_classification == "synthetic-only"
  and .no_real_personal_data == true
  and (.public_baseline_sources | length == 5)
  and (.claims_not_established | length >= 4)
' "$MANIFEST" >/dev/null

jq -e '
  .benchmark_id == "CRAFT-WATER-SERVICE-REPAIR-V1"
  and .schema_version == "1.0.0"
  and .synthetic_only == true
  and (.cases | length == 15)
  and (([.cases[].case_id] | unique | length) == 15)
  and all(.cases[];
    (.case_id | type == "string" and test("^WSR-[0-9]{3}$"))
    and (.title | type == "string" and length > 0)
    and (.input | type == "object")
    and (.expected | type == "object")
    and (.expected.disposition as $d | [
      "not_eligible",
      "eligible_for_local_review",
      "revalidation_required",
      "execution_or_closure_blocked",
      "new_workflow_required",
      "alternative_service_inadequate",
      "drinking_water_incident_response_required",
      "extended_alternative_supply_monitoring_required"
    ] | index($d) != null)
    and (.expected.reason_codes | type == "array" and length > 0)
    and (.input.context_digest | type == "string" and test("^[0-9a-f]{64}$"))
    and ((.input.expected_actions // 0) >= (.input.observed_actions // 0))
    and (all((.input.evidence // [])[];
      (.source_id | type == "string" and length > 0)
      and (.digest | type == "string" and test("^[0-9a-f]{64}$"))
      and (.context_digest | type == "string" and test("^[0-9a-f]{64}$"))
      and (.independence_group | type == "string" and length > 0)
    ))
  )
' "$CASES" >/dev/null


jq -n -e --slurpfile cases "$CASES" --slurpfile mapping "$INTEGRATION_MAP" '
  $mapping[0].benchmark_id == "CRAFT-WATER-SERVICE-REPAIR-V1-INTEGRATION-MAP"
  and $mapping[0].schema_version == "1.0.0"
  and ($mapping[0].mappings | length == 15)
  and (($mapping[0].mappings | map(.case_id) | sort) == ($cases[0].cases | map(.case_id) | sort))
  and all($mapping[0].mappings[];
    (.owner_issues | type == "array" and length > 0)
    and all(.owner_issues[]; type == "string" and test("^#[0-9]+$"))
    and (.checks | type == "array" and length > 0)
    and (.owner_boundary | type == "string" and length > 0)
  )
' >/dev/null

echo "Water Service Repair V1: manifest, 15 unique synthetic cases, and 15 cross-core owner mappings valid."
