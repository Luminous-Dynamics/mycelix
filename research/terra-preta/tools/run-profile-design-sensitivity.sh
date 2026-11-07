#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'
export LC_ALL=C
export CARGO_TERM_COLOR=never

usage() {
  cat <<'USAGE'
Usage:
  bash research/terra-preta/tools/run-profile-design-sensitivity.sh \
    <normalizer-config.json> <raw-one-minute.csv> <new-output-directory>

Environment overrides:
  TERRA_PRETA_CADENCES_SECONDS="300 600 900 1800"
  TERRA_PRETA_RESPONSE_THRESHOLDS="0.01 0.02 0.03 0.05"
  TERRA_PRETA_REFERENCE_CADENCE_SECONDS="300"
  TERRA_PRETA_DEPTH_ABLATION_THRESHOLD="0.01"

Runs the same raw profile through the existing NEON normalizer, profile
response extractor and interval-censored profile-flow auditor. Output paths
must not exist. The selected reference cadence is a high-resolution
comparison point, not ground truth.
USAGE
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
[[ $# -eq 3 ]] || { usage >&2; exit 2; }

CONFIG="$1"
RAW_CSV="$2"
OUTPUT_DIR="$3"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd -P)"
cd "$ROOT_DIR"

[[ "$CONFIG" = /* ]] || CONFIG="$ROOT_DIR/$CONFIG"
[[ "$RAW_CSV" = /* ]] || RAW_CSV="$ROOT_DIR/$RAW_CSV"
[[ "$OUTPUT_DIR" = /* ]] || OUTPUT_DIR="$ROOT_DIR/$OUTPUT_DIR"
[[ -f "$CONFIG" ]] || { echo "normalizer config not found: $CONFIG" >&2; exit 2; }
[[ -f "$RAW_CSV" ]] || { echo "raw one-minute CSV not found: $RAW_CSV" >&2; exit 2; }
[[ ! -e "$OUTPUT_DIR" ]] || { echo "output path already exists; refusing overwrite: $OUTPUT_DIR" >&2; exit 2; }

for command_name in cargo jq sha256sum awk; do
  command -v "$command_name" >/dev/null 2>&1 || {
    echo "missing required command: $command_name" >&2
    exit 2
  }
done

CADENCE_TEXT="${TERRA_PRETA_CADENCES_SECONDS:-300 600 900 1800}"
THRESHOLD_TEXT="${TERRA_PRETA_RESPONSE_THRESHOLDS:-0.01 0.02 0.03 0.05}"
REFERENCE_CADENCE="${TERRA_PRETA_REFERENCE_CADENCE_SECONDS:-300}"
DEPTH_ABLATION_THRESHOLD="${TERRA_PRETA_DEPTH_ABLATION_THRESHOLD:-0.01}"
IFS=' ' read -r -a CADENCES <<< "$CADENCE_TEXT"
IFS=' ' read -r -a THRESHOLDS <<< "$THRESHOLD_TEXT"

contains_value() {
  local needle="$1"
  shift
  local item
  for item in "$@"; do
    [[ "$item" == "$needle" ]] && return 0
  done
  return 1
}

contains_value "$REFERENCE_CADENCE" "${CADENCES[@]}" || {
  echo "reference cadence must be included in TERRA_PRETA_CADENCES_SECONDS" >&2
  exit 2
}
contains_value "$DEPTH_ABLATION_THRESHOLD" "${THRESHOLDS[@]}" || {
  echo "depth-ablation threshold must be included in TERRA_PRETA_RESPONSE_THRESHOLDS" >&2
  exit 2
}

for cadence in "${CADENCES[@]}"; do
  [[ "$cadence" =~ ^[0-9]+$ ]] || { echo "invalid cadence: $cadence" >&2; exit 2; }
  ((cadence >= 300 && cadence <= 1800)) || { echo "cadence must be 300..1800 seconds: $cadence" >&2; exit 2; }
done
for threshold in "${THRESHOLDS[@]}"; do
  jq -e --argjson threshold "$threshold" '($threshold > 0 and $threshold <= 1)' >/dev/null <<< 'null' || {
    echo "invalid response threshold: $threshold" >&2
    exit 2
  }
done

mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd -P)"
CADENCE_JSONL="$OUTPUT_DIR/cadence-runs.jsonl"
ABLATION_JSONL="$OUTPUT_DIR/depth-ablation-runs.jsonl"
: > "$CADENCE_JSONL"
: > "$ABLATION_JSONL"
chmod 600 "$CADENCE_JSONL" "$ABLATION_JSONL"

CONFIG_SHA256="$(sha256sum "$CONFIG" | awk '{print $1}')"
RAW_CSV_SHA256="$(sha256sum "$RAW_CSV" | awk '{print $1}')"
NORMALIZER="research/terra-preta/tools/neon-swc-normalizer/Cargo.toml"
EXTRACTOR="research/terra-preta/tools/profile-response-extractor/Cargo.toml"
PROFILE_AUDIT="research/terra-preta/tools/profile-flow-audit/Cargo.toml"

run_pipeline() {
  local run_config="$1"
  local target_csv="$2"
  local run_directory="$3"
  local threshold="$4"
  local cadence="$5"

  mkdir -p "$run_directory"
  local raw_profile="$run_directory/normalized-raw-profile.json"
  local paper_profile="$run_directory/normalized-paper-profile.json"
  local aggregation="$run_directory/aggregation-report.json"
  local extractor_input="$run_directory/profile-response-input.json"
  local profile_report="$run_directory/profile-flow-report.json"

  cargo run --quiet --manifest-path "$NORMALIZER" -- \
    "$run_config" "$target_csv" "$raw_profile" "$paper_profile" "$aggregation"
  cargo run --quiet --manifest-path "$EXTRACTOR" -- \
    "$raw_profile" "$extractor_input" "$threshold"
  cargo run --quiet --manifest-path "$PROFILE_AUDIT" -- \
    "$extractor_input" "$profile_report"

  jq -n \
    --argjson cadence "$cadence" \
    --argjson threshold "$threshold" \
    --arg config_sha256 "$(sha256sum "$run_config" | awk '{print $1}')" \
    --arg raw_csv_sha256 "$RAW_CSV_SHA256" \
    --arg raw_profile_sha256 "$(sha256sum "$raw_profile" | awk '{print $1}')" \
    --slurpfile aggregation "$aggregation" \
    --slurpfile profile "$profile_report" \
    '{
      target_resolution_seconds: $cadence,
      target_resolution_minutes: ($cadence / 60),
      response_threshold_m3_m3: $threshold,
      normalizer_config_sha256: $config_sha256,
      raw_csv_sha256: $raw_csv_sha256,
      normalized_raw_profile_sha256: $raw_profile_sha256,
      aggregation: {
        observed_bin_count: $aggregation[0].observed_bin_count,
        missing_bin_count: $aggregation[0].missing_bin_count,
        interpolated_bin_count: $aggregation[0].interpolated_bin_count,
        remaining_missing_bin_count: $aggregation[0].remaining_missing_bin_count
      },
      profile_classification: $profile[0].profile_classification,
      profile_coverage_status: $profile[0].profile_coverage_status,
      sensor_count: $profile[0].sensor_count,
      detected_sensor_count: $profile[0].detected_sensor_count,
      not_detected_sensor_count: $profile[0].not_detected_sensor_count,
      indeterminate_sensor_count: $profile[0].indeterminate_sensor_count,
      nsr_pair_count: $profile[0].nsr_pair_count,
      sequential_pair_count: $profile[0].sequential_pair_count,
      tied_pair_count: $profile[0].tied_pair_count,
      ambiguous_pair_count: $profile[0].ambiguous_pair_count,
      nsr_pairs: [
        $profile[0].pair_results[]?
        | select(.ordering == "nonsequential_deeper_first_interval_separated")
        | {
            shallow_sensor_id,
            deep_sensor_id,
            deep_advance_min_seconds,
            deep_advance_max_seconds
          }
      ],
      pair_results: $profile[0].pair_results
    }' > "$run_directory/design-summary.json"
  cat "$run_directory/design-summary.json" >> "$CADENCE_JSONL"
  printf '\n' >> "$CADENCE_JSONL"
  printf 'audited cadence=%s s threshold=%s: %s; NSR pairs=%s; indeterminate sensors=%s\n' \
    "$cadence" "$threshold" \
    "$(jq -r '.profile_classification' "$run_directory/design-summary.json")" \
    "$(jq -r '.nsr_pair_count' "$run_directory/design-summary.json")" \
    "$(jq -r '.indeterminate_sensor_count' "$run_directory/design-summary.json")"
}

for cadence in "${CADENCES[@]}"; do
  for threshold in "${THRESHOLDS[@]}"; do
    label="${threshold//./p}"
    run_directory="$OUTPUT_DIR/cadence_${cadence}_threshold_${label}"
    mkdir -p "$run_directory"
    run_config="$run_directory/config.json"
    jq --argjson cadence "$cadence" --argjson threshold "$threshold" \
      '.target_resolution_seconds = $cadence | .response_threshold_m3_m3 = $threshold' \
      "$CONFIG" > "$run_config"
    run_pipeline "$run_config" "$RAW_CSV" "$run_directory" "$threshold" "$cadence"
  done
done

REFERENCE_THRESHOLD="$DEPTH_ABLATION_THRESHOLD"
REFERENCE_DIRECTORY="$OUTPUT_DIR/cadence_${REFERENCE_CADENCE}_threshold_${REFERENCE_THRESHOLD//./p}"
REFERENCE_PROFILE="$REFERENCE_DIRECTORY/normalized-raw-profile.json"
[[ -s "$REFERENCE_PROFILE" ]] || {
  echo "reference normalized profile missing: $REFERENCE_PROFILE" >&2
  exit 1
}
SENSOR_COUNT="$(jq '.sensors | length' "$REFERENCE_PROFILE")"
((SENSOR_COUNT >= 3)) || {
  echo "leave-one-sensor-out ablation requires at least three declared sensors" >&2
  exit 2
}

while IFS= read -r sensor_id; do
  ablation_label="${sensor_id//[^A-Za-z0-9_-]/_}"
  run_directory="$OUTPUT_DIR/depth_ablation_${ablation_label}"
  mkdir -p "$run_directory"
  subset_profile="$run_directory/subset-profile.json"
  extractor_input="$run_directory/profile-response-input.json"
  profile_report="$run_directory/profile-flow-report.json"

  jq --arg sensor "$sensor_id" '.sensors |= map(select(.sensor_id != $sensor))' \
    "$REFERENCE_PROFILE" > "$subset_profile"
  remaining="$(jq '.sensors | length' "$subset_profile")"
  ((remaining >= 2)) || { echo "ablation left fewer than two sensors" >&2; exit 1; }

  cargo run --quiet --manifest-path "$EXTRACTOR" -- \
    "$subset_profile" "$extractor_input" "$REFERENCE_THRESHOLD"
  cargo run --quiet --manifest-path "$PROFILE_AUDIT" -- \
    "$extractor_input" "$profile_report"

  jq -n \
    --arg removed_sensor_id "$sensor_id" \
    --argjson target_resolution_seconds "$REFERENCE_CADENCE" \
    --argjson response_threshold_m3_m3 "$REFERENCE_THRESHOLD" \
    --argjson remaining_sensor_count "$remaining" \
    --slurpfile profile "$profile_report" \
    '{
      design: "leave_one_sensor_out",
      removed_sensor_id: $removed_sensor_id,
      target_resolution_seconds: $target_resolution_seconds,
      response_threshold_m3_m3: $response_threshold_m3_m3,
      remaining_sensor_count: $remaining_sensor_count,
      profile_classification: $profile[0].profile_classification,
      profile_coverage_status: $profile[0].profile_coverage_status,
      detected_sensor_count: $profile[0].detected_sensor_count,
      indeterminate_sensor_count: $profile[0].indeterminate_sensor_count,
      nsr_pair_count: $profile[0].nsr_pair_count,
      pair_results: $profile[0].pair_results,
      nsr_pairs: [
        $profile[0].pair_results[]?
        | select(.ordering == "nonsequential_deeper_first_interval_separated")
        | {shallow_sensor_id, deep_sensor_id}
      ]
    }' > "$run_directory/design-summary.json"
  cat "$run_directory/design-summary.json" >> "$ABLATION_JSONL"
  printf '\n' >> "$ABLATION_JSONL"
  printf 'depth ablation removed=%s: %s; NSR pairs=%s\n' \
    "$sensor_id" \
    "$(jq -r '.profile_classification' "$run_directory/design-summary.json")" \
    "$(jq -r '.nsr_pair_count' "$run_directory/design-summary.json")"
done < <(jq -r '.sensors[].sensor_id' "$REFERENCE_PROFILE")

SUMMARY="$OUTPUT_DIR/profile-design-sensitivity-report.json"
jq -n \
  --arg raw_csv_sha256 "$RAW_CSV_SHA256" \
  --arg base_config_sha256 "$CONFIG_SHA256" \
  --argjson reference_cadence_seconds "$REFERENCE_CADENCE" \
  --argjson reference_threshold "$REFERENCE_THRESHOLD" \
  --slurpfile cadence_runs "$CADENCE_JSONL" \
  --slurpfile depth_runs "$ABLATION_JSONL" \
  '($cadence_runs // []) as $runs
   | ($depth_runs // []) as $ablations
   | {
       audit_version: "0.1.0",
       raw_csv_sha256: $raw_csv_sha256,
       base_normalizer_config_sha256: $base_config_sha256,
       reference_cadence_seconds: $reference_cadence_seconds,
       reference_threshold_m3_m3: $reference_threshold,
       cadence_runs: $runs,
       cadence_comparisons: [
         $runs[] as $target
         | select($target.target_resolution_seconds != $reference_cadence_seconds)
         | ($runs
            | map(select(
                .target_resolution_seconds == $reference_cadence_seconds
                and .response_threshold_m3_m3 == $target.response_threshold_m3_m3
              ))
            | first
           ) as $reference
         | {
             response_threshold_m3_m3: $target.response_threshold_m3_m3,
             reference_cadence_seconds: $reference.target_resolution_seconds,
             target_cadence_seconds: $target.target_resolution_seconds,
             reference_profile_classification: $reference.profile_classification,
             target_profile_classification: $target.profile_classification,
             classification_changed: ($reference.profile_classification != $target.profile_classification),
             reference_nsr_pair_count: $reference.nsr_pair_count,
             target_nsr_pair_count: $target.nsr_pair_count,
             reference_pairs: [
               $reference.nsr_pairs[] as $pair
               | {
                   shallow_sensor_id: $pair.shallow_sensor_id,
                   deep_sensor_id: $pair.deep_sensor_id,
                   present_at_target: (
                     $target.nsr_pairs
                     | any(.[]; .shallow_sensor_id == $pair.shallow_sensor_id
                                  and .deep_sensor_id == $pair.deep_sensor_id)
                   )
                 }
             ]
           }
       ],
       depth_ablation_runs: $ablations,
       depth_ablation_comparisons: [
         $ablations[] as $ablation
         | (
             $runs
             | map(select(
                 .target_resolution_seconds == $reference_cadence_seconds
                 and .response_threshold_m3_m3 == $reference_threshold
               ))
             | first
           ) as $reference
         | {
             removed_sensor_id: $ablation.removed_sensor_id,
             full_profile_classification: $reference.profile_classification,
             ablated_profile_classification: $ablation.profile_classification,
             classification_changed: ($reference.profile_classification != $ablation.profile_classification),
             full_profile_nsr_pair_count: $reference.nsr_pair_count,
             ablated_nsr_pair_count: $ablation.nsr_pair_count
           }
       ],
       interpretation_boundary: "Cadence and depth ablations quantify algorithmic stability of sensor signatures on the same synthetic/raw profile; they do not establish physical truth. The 5-minute reference is the highest-resolution comparator in this sweep, not ground truth.",
       warnings: [
         "This tool re-runs declared QA/porosity filtering and median aggregation separately at each target cadence; it does not interpolate raw source data.",
         "All cadence comparisons share the same raw CSV, sensor roster, threshold, event and baseline config except target resolution.",
         "Leave-one-sensor-out results show layout sensitivity, not which omitted depth is physically optimal.",
         "Run on independent profiles and tracer-adjudicated events before making detection-accuracy claims."
       ]
     }' > "$SUMMARY"

printf '\nprofile design sensitivity complete\nreport: %s\nraw CSV SHA-256: %s\n' "$SUMMARY" "$RAW_CSV_SHA256"
