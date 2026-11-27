#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="$ROOT/manifest.json"
SEEDS="$ROOT/seeds.json"
RELATIONS="$ROOT/relations.json"
MUTATIONS="$ROOT/mutations.json"
SCORECARD="$ROOT/scorecard.json"
COMPARATORS="$ROOT/comparators.json"

command -v jq >/dev/null 2>&1 || { echo "jq is required" >&2; exit 127; }

if ! jq -e '
  .benchmark_id == "CRAFT-EVIDENCE-ASSURANCE-METAMORPHIC-V1"
  and .version == "1.0.0-draft"
  and .data_classification == "synthetic-only"
  and .no_real_personal_data == true
  and .execution_status == "protocol-and-structural-validation-only"
  and .source_benchmark == "CRAFT-WATER-SERVICE-REPAIR-V1"
  and .relation_count == 16
  and .adversarial_operator_count == 30
  and .negative_control_count == 16
  and .scorecard_metric_count == 15
  and .comparator_count == 9
  and ((.prohibited_claims | type) == "array")
  and ((.prohibited_claims | length) >= 5)
' "$MANIFEST" >/dev/null; then
  echo "Manifest identity, scope, execution boundary, or declared counts failed." >&2
  exit 1
fi

if ! jq -e '
  .benchmark_id == "CRAFT-EVIDENCE-ASSURANCE-METAMORPHIC-V1"
  and .schema_version == "1.0.0"
  and .synthetic_only == true
  and ((.seeds | type) == "array")
  and ((.seeds | length) >= 8)
  and (([.seeds[].seed_id] | unique | length) == (.seeds | length))
  and all(.seeds[];
    ((.seed_id | type) == "string")
    and (.seed_id | test("^SEED-WSR-[0-9]{3}$"))
    and (.source_benchmark == "CRAFT-WATER-SERVICE-REPAIR-V1")
    and (.source_case_id | test("^WSR-[0-9]{3}$"))
    and (.synthetic_only == true)
  )
' "$SEEDS" >/dev/null; then
  echo "Seed registry validation failed." >&2
  exit 1
fi

if ! jq -e '
  .benchmark_id == "CRAFT-EVIDENCE-ASSURANCE-METAMORPHIC-V1"
  and .schema_version == "1.0.0"
  and .synthetic_only == true
  and ((.relations | type) == "array")
  and ((.relations | length) == 16)
  and (([.relations[].relation_id] | unique | length) == 16)
  and all(.relations[];
    (.relation_id | type == "string" and test("^MR-[0-9]{3}$"))
    and (.name | type == "string" and length > 0)
    and (.risk_tier as $risk | (["critical", "high", "medium"] | index($risk) != null))
    and (.seed_ids | type == "array" and length > 0)
    and (.mutation_ids | type == "array" and length > 0)
    and (.negative_control_ids | type == "array" and length > 0)
    and (.invariant | type == "string" and length > 0)
    and (.expected_relation | type == "string" and length > 0)
    and (.oracle | type == "string" and length > 0)
  )
' "$RELATIONS" >/dev/null; then
  echo "Metamorphic relation schema validation failed." >&2
  exit 1
fi

if ! jq -e '
  .benchmark_id == "CRAFT-EVIDENCE-ASSURANCE-METAMORPHIC-V1"
  and .schema_version == "1.0.0"
  and ((.comparators | type) == "array")
  and ((.comparators | length) == 9)
  and (([.comparators[].comparator_id] | unique | length) == 9)
  and all(.comparators[];
    (.comparator_id | type == "string" and test("^BASE-[A-Z0-9-]+$"))
    and (.name | type == "string" and length > 0)
    and (.category | type == "string" and length > 0)
    and (.configuration_requirements | type == "array" and length > 0)
    and (.required_outputs | type == "array" and length > 0)
    and (.interpretation_guard | type == "string" and length > 0)
    and (if .category == "commercial_alm" then (.official_reference | type == "string" and startswith("https://")) else true end)
  )
  and (.non_comparable_policy.no_penalty_for_missing_interface == true)
  and (.non_comparable_policy.no_claim_of_absence_without_admin_attestation == true)
' "$COMPARATORS" >/dev/null; then
  echo "Comparator registry validation failed: missing baseline controls, outputs, or fair-comparison policy." >&2
  exit 1
fi

if ! jq -e '
  .benchmark_id == "CRAFT-EVIDENCE-ASSURANCE-METAMORPHIC-V1"
  and .schema_version == "1.0.0"
  and ((.metrics | type) == "array")
  and ((.metrics | length) == 15)
  and (([.metrics[].metric_id] | unique | length) == 15)
  and all(.metrics[];
    (.metric_id | type == "string" and test("^[A-Z]+-[0-9]{3}$"))
    and (.name | type == "string" and length > 0)
    and (.category | type == "string" and length > 0)
    and (.numerator | type == "string" and length > 0)
    and (.denominator | type == "string" and length > 0)
    and (.unit | type == "string" and length > 0)
    and (.direction | type == "string" and length > 0)
    and (.target | type == "object")
    and (.target.operator | type == "string" and length > 0)
    and (.gate | type == "boolean")
    and (.requires_field_data | type == "boolean")
    and (.report_by | type == "array" and length > 0)
  )
  and all(.metrics[] | select(.gate == true); .requires_field_data == false)
' "$SCORECARD" >/dev/null; then
  echo "Scorecard validation failed: metric IDs, definitions, or gate/field-data separation." >&2
  exit 1
fi

if ! jq -e '
  .benchmark_id == "CRAFT-EVIDENCE-ASSURANCE-METAMORPHIC-V1"
  and .schema_version == "1.0.0"
  and .synthetic_only == true
  and ((.mutations | type) == "array")
  and ((.mutations | length) == 46)
  and (([.mutations[].mutation_id] | unique | length) == 46)
  and (([.mutations[] | select(.kind == "adversarial")] | length) == 30)
  and (([.mutations[] | select(.kind == "negative_control")] | length) == 16)
  and all(.mutations[];
    (.mutation_id | type == "string")
    and (.mutation_id | test("^[MN]-[A-Z0-9-]+$"))
    and (.kind as $kind | (["adversarial", "negative_control"] | index($kind) != null))
    and (.target_path | type == "string" and length > 0)
    and (.transformation | type == "string" and length > 0)
  )
' "$MUTATIONS" >/dev/null; then
  echo "Mutation registry schema or count validation failed." >&2
  exit 1
fi

if ! jq -n -e --slurpfile seeds "$SEEDS" --slurpfile relations "$RELATIONS" --slurpfile mutations "$MUTATIONS" '
  ($seeds[0].seeds | map(.seed_id)) as $seed_ids
  | ($mutations[0].mutations | map(.mutation_id)) as $mutation_ids
  | ($mutations[0].mutations | map(select(.kind == "adversarial") | .mutation_id)) as $adversarial_ids
  | ($mutations[0].mutations | map(select(.kind == "negative_control") | .mutation_id)) as $control_ids
  | all($relations[0].relations[];
      all(.seed_ids[]; . as $id | ($seed_ids | index($id)) != null)
      and all(.mutation_ids[]; . as $id | (($adversarial_ids | index($id)) != null))
      and all(.negative_control_ids[]; . as $id | (($control_ids | index($id)) != null))
    )
  and all($mutation_ids[];
    . as $id
    | any($relations[0].relations[];
        ((.mutation_ids | index($id)) != null)
        or ((.negative_control_ids | index($id)) != null)
      )
    )
' >/dev/null; then
  echo "Cross-file relation, seed, mutation, or negative-control reference integrity failed." >&2
  exit 1
fi

echo "Evidence Assurance Metamorphic V1: 16 relations, 30 adversarial operators, 16 negative controls, and 8 synthetic seed references valid."
