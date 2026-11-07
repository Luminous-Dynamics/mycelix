#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FILE="$ROOT/research/polar-living-routines/routines-v0.json"
SELF_TEST=0
if [[ "$#" -gt 0 ]]; then
  if [[ "$1" == "--self-test" ]]; then SELF_TEST=1; else FILE="$1"; fi
fi
command -v jq >/dev/null 2>&1 || { echo "ERROR: jq is required" >&2; exit 2; }
[[ -r "$FILE" ]] || { echo "ERROR: cannot read $FILE" >&2; exit 2; }

# Structural smoke-check only. Full JSON Schema validation and community/ethics review are separate.
jq -e '
 .schema_version=="0.1.0"
 and .artifact_kind=="voluntary-polar-routine-catalog"
 and .status=="proposed"
 and .evidence_tier=="S0"
 and .claim_ceiling=="design_hypothesis_only"
 and .deployment_status=="not_for_deployment"
 and (.governing_principles|length>=8)
 and .privacy_defaults.personal_data_collected==false
 and .privacy_defaults.sensitive_health_data_collected==false
 and .privacy_defaults.personal_location_history_collected==false
 and .privacy_defaults.individual_adherence_scoring==false
 and .privacy_defaults.default_storage=="none"
 and .privacy_defaults.distributed_ledger_personal_data==false
 and .privacy_defaults.participation=="voluntary"
 and (.evidence_sources|length>=6)
 and (([.evidence_sources[].id]|unique|length)==(.evidence_sources|length))
 and (.routines|length>=8)
 and (([.routines[].id]|unique|length)==(.routines|length))
 and all(.routines[];
   (.id|type=="string" and test("^POLAR-HABIT-[0-9]{3}$"))
   and (.options|length>=2)
   and (.avoid|length>=2)
   and (.evaluation|length>=2)
   and (.impact_claim=="none")
   and (.evidence_source_ids|length>=1)
   and ((.autonomy_note|length)>=10)
 )
 and all(.routines[]; (.evidence_source_ids | unique | length) == (.evidence_source_ids | length))
 and ([.evidence_sources[].id] as $source_ids
      | all(.routines[]; all(.evidence_source_ids[]; . as $id | $source_ids | index($id) != null)))
' "$FILE" >/dev/null || { echo "ERROR: routine catalog violates structural safeguards" >&2; exit 1; }

if [[ "$SELF_TEST" -eq 0 ]]; then
 echo "catalog structural smoke-check passed; community approval and health effects have not been validated"
 exit 0
fi
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
reject() {
 local name="$1" expr="$2"
 jq "$expr" "$ROOT/research/polar-living-routines/routines-v0.json" > "$TMP/mutated.json"
 if "$0" "$TMP/mutated.json" >/dev/null 2>&1; then
  echo "ERROR: negative control accepted: $name" >&2; exit 1
 fi
 echo "rejected: $name"
}
reject "promotion above S0" '.evidence_tier="S3"'
reject "deployment authorization without governance" '.deployment_status="approved"'
reject "mandatory personal habit" '.privacy_defaults.participation="mandatory"'
reject "personal data collection" '.privacy_defaults.personal_data_collected=true'
reject "sensitive health data collection" '.privacy_defaults.sensitive_health_data_collected=true'
reject "individual adherence scoring" '.privacy_defaults.individual_adherence_scoring=true'
reject "distributed ledger stores personal data" '.privacy_defaults.distributed_ledger_personal_data=true'
reject "remove opt-out guidance" '.routines[0].autonomy_note=""'
reject "routine claims health impact" '.routines[0].impact_claim="improves_sleep"'
reject "duplicate routine ID" '.routines[1].id=.routines[0].id'
reject "unknown evidence source" '.routines[0].evidence_source_ids=["not-a-cited-source"]'
echo "11 negative controls rejected; this is not clinical or community deployment validation"
