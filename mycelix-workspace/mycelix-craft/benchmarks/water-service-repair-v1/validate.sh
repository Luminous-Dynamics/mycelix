#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="$ROOT/manifest.json"
CASES="$ROOT/cases.json"

command -v jq >/dev/null 2>&1 || { echo "jq is required" >&2; exit 127; }
jq -e . "$MANIFEST" >/dev/null
jq -e . "$CASES" >/dev/null

jq -e '
  .benchmark_id == "CRAFT-WATER-SERVICE-REPAIR-V1"
  and .version == "1.0.0-draft"
  and .data_classification == "synthetic-only"
  and .no_real_personal_data == true
  and (.public_baseline_sources | length == 3)
  and (.claims_not_established | length >= 4)
' "$MANIFEST" >/dev/null

jq -e '
  .benchmark_id == "CRAFT-WATER-SERVICE-REPAIR-V1"
  and .schema_version == "1.0.0"
  and .synthetic_only == true
  and (.cases | length == 12)
  and (([.cases[].case_id] | unique | length) == 12)
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
      "new_workflow_required"
    ] | index($d) != null)
    and (.expected.reason_codes | type == "array" and length > 0)
    and (.input.context_digest | type == "string" and test("^[0-9a-f]{64}$"))
    and ((.input.expected_actions // 0) >= (.input.observed_actions // 0))
    and (all((.input.evidence // [])[];
      (.evidence_id | type == "string" and length > 0)
      and (.digest | type == "string" and test("^[0-9a-f]{64}$"))
      and (.context_digest | type == "string" and test("^[0-9a-f]{64}$"))
      and (.independence_group | type == "string" and length > 0)
    ))
  )
' "$CASES" >/dev/null

echo "Water Service Repair V1: manifest valid; 12 unique synthetic cases valid."
