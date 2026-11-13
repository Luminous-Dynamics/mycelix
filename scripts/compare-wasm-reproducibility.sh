#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="${1:?usage: compare-wasm-reproducibility.sh ARTIFACT_ROOT}"
command -v jq >/dev/null 2>&1 || { echo "jq is required" >&2; exit 2; }
command -v diff >/dev/null 2>&1 || { echo "diff is required" >&2; exit 2; }

ROOT="$(realpath "$ROOT")"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
SUMMARY="${GITHUB_STEP_SUMMARY:-$WORK/summary.md}"
REPORT="$WORK/wasm-reproducibility-report.json"
printf '{"schema":"luminous.mycelix.wasm-reproducibility.v1","clusters":[]}\n' > "$REPORT"

for cluster in finance governance; do
  mapfile -t metadata_files < <(find "$ROOT" -type f -name metadata.json -path "*mycelix-wasm-repro-$cluster-repeat-*" | sort)
  mapfile -t manifest_files < <(find "$ROOT" -type f -name manifest.jsonl -path "*mycelix-wasm-repro-$cluster-repeat-*" | sort)

  [[ "${#metadata_files[@]}" -eq 2 ]] || {
    echo "Expected exactly two independent metadata receipts for $cluster; found ${#metadata_files[@]}" >&2
    exit 1
  }
  [[ "${#manifest_files[@]}" -eq 2 ]] || {
    echo "Expected exactly two independent artifact manifests for $cluster; found ${#manifest_files[@]}" >&2
    exit 1
  }

  for i in 0 1; do
    meta="${metadata_files[$i]}"
    manifest="${manifest_files[$i]}"
    repeat="$(jq -r '.repeat' "$meta")"
    [[ "$repeat" == "$((i+1))" ]] || {
      echo "Expected $cluster repeat $((i+1)); got $repeat from $meta" >&2
      exit 1
    }
    [[ "$(jq -r '.cluster' "$meta")" == "$cluster" ]]
    [[ "$(jq -r '.target' "$meta")" == "wasm32-unknown-unknown" ]]
    [[ "$(jq -r '.rustc' "$meta")" == rustc\ 1.100.0\ * ]]
    [[ "$(jq -r '.cargo' "$meta")" == cargo\ 1.100.0\ * ]]
    [[ "$(jq -r '.source_commit' "$meta")" != "" ]]
    [[ "$(jq -r '.lock_sha256' "$meta")" =~ ^[0-9a-f]{64}$ ]]
    [[ "$(jq -s 'length' "$manifest")" -eq 2 ]] || {
      echo "Expected exactly two selected zome artifacts in $manifest" >&2
      cat "$manifest" >&2
      exit 1
    }
  done

  meta1="${metadata_files[0]}"
  meta2="${metadata_files[1]}"
  manifest1="${manifest_files[0]}"
  manifest2="${manifest_files[1]}"

  for field in source_commit cluster rustc cargo target lock_sha256 build_flags; do
    a="$(jq -r --arg f "$field" '.[$f]' "$meta1")"
    b="$(jq -r --arg f "$field" '.[$f]' "$meta2")"
    [[ "$a" == "$b" ]] || {
      echo "Independent $cluster builds do not share $field: '$a' vs '$b'" >&2
      exit 1
    }
  done

  jq -S -s 'sort_by(.target) | map({package_id,target,filename_basename:(.filename|split("/")[-1]),size_bytes,sha256})' "$manifest1" > "$WORK/$cluster-1.json"
  jq -S -s 'sort_by(.target) | map({package_id,target,filename_basename:(.filename|split("/")[-1]),size_bytes,sha256})' "$manifest2" > "$WORK/$cluster-2.json"

  if ! cmp -s "$WORK/$cluster-1.json" "$WORK/$cluster-2.json"; then
    echo "FAIL: $cluster WASM output is not bit-for-bit reproducible across two independent runners." >&2
    diff -u "$WORK/$cluster-1.json" "$WORK/$cluster-2.json" || true
    exit 1
  fi

  if [[ "$cluster" == finance ]]; then
    expected_targets="$(printf 'payments\npayments_integrity')"
  else
    expected_targets="$(printf 'voting\nvoting_integrity')"
  fi
  actual_targets="$(jq -r '.[].target' "$WORK/$cluster-1.json" | sort)"
  [[ "$actual_targets" == "$expected_targets" ]] || {
    echo "Unexpected target set for $cluster: $actual_targets" >&2
    exit 1
  }

  size_total="$(jq '[.[].size_bytes] | add' "$WORK/$cluster-1.json")"
  tmp="$WORK/report-$cluster.json"
  jq -n \
    --arg cluster "$cluster" \
    --arg source "$(jq -r '.source_commit' "$meta1")" \
    --arg rustc "$(jq -r '.rustc' "$meta1")" \
    --arg cargo "$(jq -r '.cargo' "$meta1")" \
    --arg lock "$(jq -r '.lock_sha256' "$meta1")" \
    --argjson size "$size_total" \
    --slurpfile targets "$WORK/$cluster-1.json" \
    '{cluster:$cluster,source_commit:$source,rustc:$rustc,cargo:$cargo,lock_sha256:$lock,total_size_bytes:$size,targets:$targets[0],independent_runner_builds:2,bitwise_equal:true}' > "$tmp"

  combined="$WORK/report-combined.json"
  jq --slurpfile c "$tmp" '.clusters += $c' "$REPORT" > "$combined"
  mv "$combined" "$REPORT"

  {
    echo "## $cluster — same-toolchain WASM reproducibility"
    echo
    echo "- Source: $(jq -r '.source_commit' "$meta1")"
    echo "- Compiler: $(jq -r '.rustc' "$meta1")"
    echo "- Cargo: $(jq -r '.cargo' "$meta1")"
    echo "- Lock SHA-256: $(jq -r '.lock_sha256' "$meta1")"
    echo "- Independent runner builds: 2"
    echo "- Total selected artifact bytes: $size_total"
    echo "- Bitwise reproducible: PASS"
    echo
    echo "| Target | Bytes | SHA-256 |"
    echo "|---|---:|---|"
    jq -r '.[] | "| " + .target + " | " + (.size_bytes|tostring) + " | " + .sha256 + " |"' "$WORK/$cluster-1.json"
    echo
  } >> "$SUMMARY"
done

cp "$REPORT" "${REPORT_OUTPUT:-$ROOT/wasm-reproducibility-report.json}"
echo "wasm_reproducibility=PASS"
