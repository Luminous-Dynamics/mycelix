#!/usr/bin/env bash
set -Eeuo pipefail

# Reproducible bounded TLC campaign for the NCF-FEN-1 and NCF-ESCROW-1 safe
# models plus four deliberately unsafe mutants. Each mutant must fail for its named invariant;
# parser/tool failures are not successful negative-control detections.

repo_root="$(git rev-parse --show-toplevel)"
formal_dir="$repo_root/mycelix-workspace/simulations/nested-currency-reference/formal"
jar="${TLA2TOOLS_JAR:-}"
if [[ -z "$jar" || ! -f "$jar" ]]; then
  echo "TLA2TOOLS_JAR must name an existing, locally available tla2tools.jar" >&2
  exit 2
fi
jar="$(realpath "$jar")"

if [[ -n "$(git -C "$repo_root" status --porcelain)" ]]; then
  echo "refusing to qualify a dirty checkout; commit or discard changes first" >&2
  exit 2
fi

source_commit="$(git -C "$repo_root" rev-parse HEAD)"
source_tree="$(git -C "$repo_root" rev-parse 'HEAD^{tree}')"
model="$formal_dir/NCF_FEN_1.tla"
config="$formal_dir/NCF_FEN_1.cfg"
timeout_mutant="$formal_dir/NCF_FEN_1_TimeoutMutant.tla"
timeout_config="$formal_dir/NCF_FEN_1_TimeoutMutant.cfg"
stale_mutant="$formal_dir/NCF_FEN_1_StaleOwnerMutant.tla"
stale_config="$formal_dir/NCF_FEN_1_StaleOwnerMutant.cfg"
escrow_model="$formal_dir/NCF_ESCROW_1.tla"
escrow_config="$formal_dir/NCF_ESCROW_1.cfg"
duplicate_mutant="$formal_dir/NCF_ESCROW_1_DuplicateAllocationMutant.tla"
duplicate_config="$formal_dir/NCF_ESCROW_1_DuplicateAllocationMutant.cfg"
resize_mutant="$formal_dir/NCF_ESCROW_1_PartitionResizeMutant.tla"
resize_config="$formal_dir/NCF_ESCROW_1_PartitionResizeMutant.cfg"

for path in "$model" "$config" "$timeout_mutant" "$timeout_config" "$stale_mutant" "$stale_config" "$escrow_model" "$escrow_config" "$duplicate_mutant" "$duplicate_config" "$resize_mutant" "$resize_config"; do
  [[ -f "$path" ]] || { echo "missing model artifact: $path" >&2; exit 2; }
done

out_dir="${NCF_TLC_OUTPUT_DIR:-${TMPDIR:-/tmp}/ncf-fen-tlc-$source_commit}"
mkdir -p "$out_dir"
safe_log="$out_dir/safe-model.log"
timeout_log="$out_dir/timeout-mutant.log"
stale_log="$out_dir/stale-owner-mutant.log"
escrow_log="$out_dir/escrow-model.log"
duplicate_log="$out_dir/duplicate-allocation-mutant.log"
resize_log="$out_dir/partition-resize-mutant.log"
receipt="$out_dir/receipt.json"

model_sha="$(sha256sum "$model" | awk '{print $1}')"
config_sha="$(sha256sum "$config" | awk '{print $1}')"
timeout_sha="$(sha256sum "$timeout_mutant" | awk '{print $1}')"
timeout_config_sha="$(sha256sum "$timeout_config" | awk '{print $1}')"
stale_sha="$(sha256sum "$stale_mutant" | awk '{print $1}')"
stale_config_sha="$(sha256sum "$stale_config" | awk '{print $1}')"
escrow_sha="$(sha256sum "$escrow_model" | awk '{print $1}')"
escrow_config_sha="$(sha256sum "$escrow_config" | awk '{print $1}')"
duplicate_sha="$(sha256sum "$duplicate_mutant" | awk '{print $1}')"
duplicate_config_sha="$(sha256sum "$duplicate_config" | awk '{print $1}')"
resize_sha="$(sha256sum "$resize_mutant" | awk '{print $1}')"
resize_config_sha="$(sha256sum "$resize_config" | awk '{print $1}')"
jar_sha="$(sha256sum "$jar" | awk '{print $1}')"
java_version="$(java -version 2>&1 | sed -n '1p' | tr -cd '[:alnum:]._- ')"

safe_meta="$out_dir/states-fen-safe"
mkdir -p "$safe_meta"
(
  cd "$formal_dir"
  java -cp "$jar" tlc2.TLC -metadir "$safe_meta" -config "$(basename "$config")" "$(basename "$model")"
) >"$safe_log" 2>&1 || {
  cat "$safe_log" >&2
  echo "safe NCF-FEN model failed; inspect $safe_log" >&2
  exit 1
}

safe_stats="$(grep -E 'states generated, .*distinct states found' "$safe_log" | tail -n 1 || true)"
if [[ -z "$safe_stats" ]]; then
  cat "$safe_log" >&2
  echo "could not identify TLC state statistics in the safe-model log" >&2
  exit 1
fi
safe_stats="$(printf '%s' "$safe_stats" | tr -cd '[:alnum:],. _-')"

escrow_meta="$out_dir/states-escrow-safe"
mkdir -p "$escrow_meta"
(
  cd "$formal_dir"
  java -cp "$jar" tlc2.TLC -metadir "$escrow_meta" -config "$(basename "$escrow_config")" "$(basename "$escrow_model")"
) >"$escrow_log" 2>&1 || {
  cat "$escrow_log" >&2
  echo "safe NCF-ESCROW model failed; inspect $escrow_log" >&2
  exit 1
}
escrow_stats="$(grep -E 'states generated, .*distinct states found' "$escrow_log" | tail -n 1 || true)"
if [[ -z "$escrow_stats" ]]; then
  cat "$escrow_log" >&2
  echo "could not identify TLC state statistics in the escrow-model log" >&2
  exit 1
fi
escrow_stats="$(printf '%s' "$escrow_stats" | tr -cd '[:alnum:],. _-')"

run_expected_mutant() {
  local model_file="$1"
  local config_file="$2"
  local log_file="$3"
  local invariant="$4"
  local label="$5"
  local status=0
  local metadata_dir="$out_dir/states-$label"
  mkdir -p "$metadata_dir"

  set +e
  (
    cd "$formal_dir"
    java -cp "$jar" tlc2.TLC -metadir "$metadata_dir" -config "$(basename "$config_file")" "$(basename "$model_file")"
  ) >"$log_file" 2>&1
  status=$?
  set -e

  if [[ "$status" -eq 0 ]]; then
    cat "$log_file" >&2
    echo "unsafe $label mutant unexpectedly passed all invariants" >&2
    exit 1
  fi
  if ! grep -Eiq "Invariant .* ${invariant} .*violated|${invariant}.*violated" "$log_file"; then
    cat "$log_file" >&2
    echo "$label mutant failed, but not with expected invariant $invariant" >&2
    exit 1
  fi
}

run_expected_mutant "$timeout_mutant" "$timeout_config" "$timeout_log" "NoTransferWithoutFence" "timeout-reallocation"
run_expected_mutant "$stale_mutant" "$stale_config" "$stale_log" "NoOldWriteOutsideActive" "stale-owner-write"
run_expected_mutant "$duplicate_mutant" "$duplicate_config" "$duplicate_log" "ReservedCapacityBound" "duplicate-allocation"
run_expected_mutant "$resize_mutant" "$resize_config" "$resize_log" "PartitionedReservationsStayFixed" "partition-resize"

safe_log_sha="$(sha256sum "$safe_log" | awk '{print $1}')"
timeout_log_sha="$(sha256sum "$timeout_log" | awk '{print $1}')"
stale_log_sha="$(sha256sum "$stale_log" | awk '{print $1}')"
escrow_log_sha="$(sha256sum "$escrow_log" | awk '{print $1}')"
duplicate_log_sha="$(sha256sum "$duplicate_log" | awk '{print $1}')"
resize_log_sha="$(sha256sum "$resize_log" | awk '{print $1}')"
cat >"$receipt" <<EOF
{
  "schema": "mycelix.ncf-fen-tlc-receipt",
  "schema_version": "1.0.0",
  "source_commit": "$source_commit",
  "source_tree": "$source_tree",
  "safe_model_sha256": "$model_sha",
  "safe_config_sha256": "$config_sha",
  "timeout_mutant_sha256": "$timeout_sha",
  "timeout_mutant_config_sha256": "$timeout_config_sha",
  "stale_owner_mutant_sha256": "$stale_sha",
  "stale_owner_mutant_config_sha256": "$stale_config_sha",
  "escrow_model_sha256": "$escrow_sha",
  "escrow_config_sha256": "$escrow_config_sha",
  "duplicate_allocation_mutant_sha256": "$duplicate_sha",
  "duplicate_allocation_mutant_config_sha256": "$duplicate_config_sha",
  "partition_resize_mutant_sha256": "$resize_sha",
  "partition_resize_mutant_config_sha256": "$resize_config_sha",
  "tla2tools_jar_sha256": "$jar_sha",
  "java_version": "$java_version",
  "safe_model_status": "passed",
  "timeout_mutant_status": "expected_failure_detected",
  "timeout_mutant_invariant": "NoTransferWithoutFence",
  "stale_owner_mutant_status": "expected_failure_detected",
  "stale_owner_mutant_invariant": "NoOldWriteOutsideActive",
  "duplicate_allocation_mutant_status": "expected_failure_detected",
  "duplicate_allocation_mutant_invariant": "ReservedCapacityBound",
  "partition_resize_mutant_status": "expected_failure_detected",
  "partition_resize_mutant_invariant": "PartitionedReservationsStayFixed",
  "safe_model_state_statistics_line": "$safe_stats",
  "escrow_model_status": "passed",
  "escrow_model_state_statistics_line": "$escrow_stats",
  "safe_log_sha256": "$safe_log_sha",
  "timeout_mutant_log_sha256": "$timeout_log_sha",
  "stale_owner_mutant_log_sha256": "$stale_log_sha",
  "escrow_model_log_sha256": "$escrow_log_sha",
  "duplicate_allocation_mutant_log_sha256": "$duplicate_log_sha",
  "partition_resize_mutant_log_sha256": "$resize_log_sha",
  "claim_ceiling": "bounded TLA+ abstraction only; not a proof of production Holochain, cryptographic authority, or unbounded distributed safety"
}
EOF

echo "NCF-FEN bounded formal campaign qualified for this exact checkout."
receipt_sha="$(sha256sum "$receipt" | awk '{print $1}')"
echo "receipt: $receipt"
echo "receipt sha256: $receipt_sha"
echo "safe log sha256: $safe_log_sha"
echo "timeout mutant log sha256: $timeout_log_sha"
echo "stale-owner mutant log sha256: $stale_log_sha"
echo "escrow model log sha256: $escrow_log_sha"
echo "duplicate-allocation mutant log sha256: $duplicate_log_sha"
echo "partition-resize mutant log sha256: $resize_log_sha"
