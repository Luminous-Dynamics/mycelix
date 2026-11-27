#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CASES="$ROOT/cases.json"
MUTATIONS="$ROOT/metamorphic-cases.json"

command -v jq >/dev/null 2>&1 || { echo "jq is required" >&2; exit 127; }

if ! jq -n -e --slurpfile cases "$CASES" --slurpfile mutations "$MUTATIONS" '
  $cases[0].benchmark_id == "CRAFT-WATER-SERVICE-REPAIR-V1"
  and $mutations[0].benchmark_id == "CRAFT-WATER-SERVICE-REPAIR-V1-METAMORPHIC"
  and $mutations[0].schema_version == "1.0.0"
  and $mutations[0].base_corpus == "CRAFT-WATER-SERVICE-REPAIR-V1"
  and $mutations[0].data_classification == "synthetic-only"
  and $mutations[0].no_real_personal_data == true
  and $mutations[0].status == "proposed_metamorphic_vectors"
  and (($mutations[0].mutations | type) == "array")
  and (($mutations[0].mutations | length) == 20)
  and (($mutations[0].mutations | map(.mutation_id) | unique | length) == 20)
  and all($mutations[0].mutations[];
    (.mutation_id | test("^WSRM-[0-9]{3}$"))
    and ((.seed_case_id as $id | [$cases[0].cases[].case_id] | index($id)) != null)
    and (.family | type == "string" and length > 0)
    and (.operation == "replace")
    and (.path | type == "string" and test("^(input|metric)\\."))
    and (.from != .to)
    and (.expected_invariant | type == "string" and length > 0)
    and (.expected_effect | type == "string" and length > 0)
    and (.risk_direction as $risk | ["weaken","boundary","invalid"] | index($risk) != null)
    and (.execution_status == "vector_defined_not_executed")
    and (.requires == "cross-core-adapter-and-independent-oracle")
  )
' >/dev/null; then
  echo "Metamorphic validation failed: identity, count, unique IDs, seed references or mutation contract." >&2
  exit 1
fi

echo "Water Service Repair: 20 proposed metamorphic vectors structurally valid; none is claimed executed against the Rust cores."
