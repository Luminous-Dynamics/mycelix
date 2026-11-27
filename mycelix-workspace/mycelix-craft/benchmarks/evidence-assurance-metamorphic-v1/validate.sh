#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="$ROOT/manifest.json"
SEEDS="$ROOT/seeds.json"
RELATIONS="$ROOT/relations.json"
MUTATIONS="$ROOT/mutations.json"

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
    and (["critical", "high", "medium"] | index(.risk_tier) != null)
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
  and .synthetic_only == true
  and ((.mutations | type) == "array")
  and ((.mutations | length) == 46)
  and (([.mutations[].mutation_id] | unique | length) == 46)
  and (([.mutations[] | select(.kind == "adversarial")] | length) == 30)
  and (([.mutations[] | select(.kind == "negative_control")] | length) == 16)
  and all(.mutations[];
    (.mutation_id | type == "string")
    and (.mutation_id | test("^[MN]-[A-Z0-9-]+$"))
    and (["adversarial", "negative_control"] | index(.kind) != null)
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
