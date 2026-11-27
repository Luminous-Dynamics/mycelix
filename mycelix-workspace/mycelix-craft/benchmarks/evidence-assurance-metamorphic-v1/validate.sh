#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="$ROOT/manifest.json"
SEEDS="$ROOT/seeds.json"
RELATIONS="$ROOT/relations.json"
MUTATIONS="$ROOT/mutations.json"
SCORECARD="$ROOT/scorecard.json"
COMPARATORS="$ROOT/comparators.json"
OWNER_INPUTS="$ROOT/owner-inputs.json"
MUTATION_SPECS="$ROOT/mutation-specs.json"
ORACLE_CONTRACT="$ROOT/oracle-contract.json"
WATER_CASES="$ROOT/../water-service-repair-v1/cases.json"

command -v jq >/dev/null 2>&1 || { echo "jq is required" >&2; exit 127; }

if ! jq -e '
  .benchmark_id == "CRAFT-EVIDENCE-ASSURANCE-METAMORPHIC-V1"
  and .version == "1.0.0-draft"
  and .data_classification == "synthetic-only"
  and .no_real_personal_data == true
  and .execution_status == "protocol-and-structural-validation-only"
  and .source_benchmark == "CRAFT-WATER-SERVICE-REPAIR-V1"
  and .relation_count == 18
  and .adversarial_operator_count == 34
  and .negative_control_count == 20
  and .scorecard_metric_count == 17
  and .seed_count == 10
  and .comparator_count == 9
  and .mutation_spec_count == 54
  and .owner_projection_count == 10
  and .oracle_class_count == 5
  and .execution_contract_status == "declarative-specification-only"
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
  and ((.seeds | length) == 10)
  and (([.seeds[].seed_id] | unique | length) == (.seeds | length))
  and all(.seeds[];
    ((.seed_id | type) == "string")
    and (.seed_id | test("^SEED-[A-Z0-9-]+$"))
    and (.source_benchmark as $source | (["CRAFT-WATER-SERVICE-REPAIR-V1", "CRAFT-NEED-INTAKE-V1"] | index($source) != null))
    and (.source_case_id | test("^(WSR-[0-9]{3}|NIV1-[A-Z0-9-]+)$"))
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
  and ((.relations | length) == 18)
  and (([.relations[].relation_id] | unique | length) == 18)
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
  and ((.metrics | length) == 17)
  and (([.metrics[].metric_id] | unique | length) == 17)
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
  and ((.mutations | length) == 54)
  and (([.mutations[].mutation_id] | unique | length) == 54)
  and (([.mutations[] | select(.kind == "adversarial")] | length) == 34)
  and (([.mutations[] | select(.kind == "negative_control")] | length) == 20)
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

if ! jq -e '
  .benchmark_id == "CRAFT-EVIDENCE-ASSURANCE-METAMORPHIC-V1"
  and .schema_version == "1.0.0"
  and .synthetic_only == true
  and ((.seeds | type) == "array")
  and ((.seeds | length) == 10)
  and (([.seeds[].seed_id] | unique | length) == 10)
  and all(.seeds[];
    (.seed_id | type == "string" and test("^SEED-[A-Z0-9-]+$"))
    and (.source_benchmark | type == "string" and length > 0)
    and (.source_case_id | type == "string" and length > 0)
    and (.owner_views | type == "object")
    and (if .source_benchmark == "CRAFT-WATER-SERVICE-REPAIR-V1" then
      (.source_fixture.immutable == true)
      and (.source_fixture.blob_sha | type == "string" and test("^[0-9a-f]{40}$"))
      and (.owner_views.source_case.immutable == true)
      and (.owner_views.source_case.source_blob_sha == .source_fixture.blob_sha)
      and (.owner_views.source_case.source_case_id == .source_case_id)
    else true end)
  )
' "$OWNER_INPUTS" >/dev/null; then
  echo "Owner-input projection validation failed." >&2
  exit 1
fi

if ! jq -e '
  .benchmark_id == "CRAFT-EVIDENCE-ASSURANCE-METAMORPHIC-V1"
  and .schema_version == "1.0.0"
  and .execution_status == "declarative-specification-only"
  and .source_registry_count == 54
  and ((.mutation_specs | type) == "array")
  and ((.mutation_specs | length) == 54)
  and (([.mutation_specs[].mutation_id] | unique | length) == 54)
  and all(.mutation_specs[];
    (.mutation_id | type == "string" and test("^[MN]-[A-Z0-9-]+$"))
    and (["adversarial", "negative_control"] | index(.kind) != null)
    and (["critical", "high", "medium"] | index(.risk_tier) != null)
    and (.applicable_seed_ids | type == "array" and length > 0)
    and (.registry_target_path | type == "string" and length > 0)
    and (.owner_view | type == "string" and length > 0)
    and (.target_path | type == "string" and length > 0)
    and (.selector | type == "object")
    and (.selector.mode | type == "string" and length > 0)
    and (.selector.ambiguity_policy | type == "string" and length > 0)
    and (.preconditions | type == "array" and length > 0)
    and (.operation | type == "object")
    and (.operation.kind | type == "string" and length > 0)
    and (.operation.parameters | type == "object")
    and (.preserve_policy.all_non_target_fields == true)
    and (.preserve_policy.source_fixture_immutable == true)
    and (.postconditions | type == "array" and length > 0)
    and (.owner_evaluator | type == "string" and length > 0)
    and (.independent_oracle | type == "string" and length > 0)
    and (.semantic_validity.required == true)
    and (.semantic_validity.invalid_outcome == "invalid_mutation_excluded_from_detection_denominator")
    and (.semantic_validity.ambiguous_selection_outcome == "invalid_mutation_excluded_from_detection_denominator")
  )
' "$MUTATION_SPECS" >/dev/null; then
  echo "Typed mutation specification validation failed." >&2
  exit 1
fi

if ! jq -e '
  .benchmark_id == "CRAFT-EVIDENCE-ASSURANCE-METAMORPHIC-V1"
  and .schema_version == "1.0.0"
  and .execution_status == "oracle-protocol-only_no_owner-adapter-execution"
  and ((.oracle_classes | type) == "array")
  and ((.oracle_classes | length) == 5)
  and (([.oracle_classes[].oracle_id] | unique | length) == 5)
  and all(.oracle_classes[];
    (.oracle_id | type == "string" and length > 0)
    and (.kind | type == "string" and length > 0)
    and (.description | type == "string" and length > 0)
    and (.independence_requirements | type == "array" and length > 0)
  )
  and all(.risk_requirements[];
    (.minimum_distinct_oracle_classes >= 2)
    and (.required_classes | index("OWNER-EVALUATOR") != null)
    and (.required_classes | index("INDEPENDENT-REFERENCE") != null)
    and (.trace_required == true)
  )
  and (.common_cause_policy.distinct_ids_do_not_prove_independence == true)
  and (.semantic_validity_gate.translation_policy | type == "string")
  and ((.outcome_taxonomy | length) >= 8)
  and ((.receipt_required_fields | index("source_fixture_digest")) != null)
  and ((.receipt_required_fields | index("independent_oracle_output_digest")) != null)
' "$ORACLE_CONTRACT" >/dev/null; then
  echo "Independent oracle and semantic validity contract validation failed." >&2
  exit 1
fi

if ! jq -n -e --slurpfile seeds "$SEEDS" --slurpfile relations "$RELATIONS" --slurpfile mutations "$MUTATIONS" --slurpfile owner_inputs "$OWNER_INPUTS" --slurpfile mutation_specs "$MUTATION_SPECS" --slurpfile water_cases "$WATER_CASES" '
  ($seeds[0].seeds | map(.seed_id)) as $seed_ids
  | ($owner_inputs[0].seeds | map(.seed_id)) as $owner_seed_ids
  | ($mutations[0].mutations | map(.mutation_id)) as $mutation_ids
  | ($mutations[0].mutations | map(select(.kind == "adversarial") | .mutation_id)) as $adversarial_ids
  | ($mutations[0].mutations | map(select(.kind == "negative_control") | .mutation_id)) as $control_ids
  | ($mutation_specs[0].mutation_specs) as $specs
  | all($relations[0].relations[];
      all(.seed_ids[]; . as $id | ($seed_ids | index($id)) != null)
      and all(.mutation_ids[]; . as $id | ($adversarial_ids | index($id)) != null)
      and all(.negative_control_ids[]; . as $id | ($control_ids | index($id)) != null)
      and all(.mutation_ids[]; . as $id |
        ($specs | map(select(.mutation_id == $id)) | length) == 1
      )
      and all(.negative_control_ids[]; . as $id |
        ($specs | map(select(.mutation_id == $id)) | length) == 1
      )
    )
  and all($mutation_ids[];
    . as $id
    | any($relations[0].relations[];
        ((.mutation_ids | index($id)) != null)
        or ((.negative_control_ids | index($id)) != null)
      )
    )
  and all($specs[];
    . as $spec
    | any($mutations[0].mutations[]; .mutation_id == $spec.mutation_id and .kind == $spec.kind and .target_path == $spec.registry_target_path)
    and all($spec.applicable_seed_ids[];
      . as $sid
      | ($seed_ids | index($sid)) != null
      and ($owner_seed_ids | index($sid)) != null
      and any($owner_inputs[0].seeds[];
        .seed_id == $sid
        and (
          if $spec.owner_view == "source_case" then
            (.owner_views.source_case.immutable == true)
            and any($water_cases[0].cases[]; .case_id == .source_case_id and ($spec.target_path | split(".") as $parts | getpath($parts)) != null)
          else
            (.owner_views | has($spec.owner_view))
            and ((["owner_views", $spec.owner_view] + ($spec.target_path | split("."))) as $parts | getpath($parts)) != null
          end
        )
      )
    )
    and any($relations[0].relations[];
      ((.mutation_ids + .negative_control_ids) | index($spec.mutation_id)) != null
      and any(.seed_ids[]; . as $sid | ($spec.applicable_seed_ids | index($sid)) != null)
    )
  )
' >/dev/null; then
  echo "Cross-file relation, seed, mutation, or negative-control reference integrity failed." >&2
  exit 1
fi

echo "Evidence Assurance Metamorphic V1: 18 relations, 34 adversarial operators, 20 negative controls, 17 metrics, 9 comparators, 10 owner projections, 54 typed mutation specs, and 5 oracle classes valid."
