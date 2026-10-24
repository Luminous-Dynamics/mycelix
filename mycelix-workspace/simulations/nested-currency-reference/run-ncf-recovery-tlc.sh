#!/usr/bin/env bash
set -Eeuo pipefail

# Exact-head bounded crash/recovery model plus invariant-specific retry mutant.
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

model="$formal_dir/NCF_RECOVERY_1.tla"
config="$formal_dir/NCF_RECOVERY_1.cfg"
mutant="$formal_dir/NCF_RECOVERY_1_CommitReplayMutant.tla"
mutant_config="$formal_dir/NCF_RECOVERY_1_CommitReplayMutant.cfg"
for path in "$model" "$config" "$mutant" "$mutant_config"; do
  [[ -f "$path" ]] || { echo "missing model artifact: $path" >&2; exit 2; }
done

source_commit="$(git -C "$repo_root" rev-parse HEAD)"
source_tree="$(git -C "$repo_root" rev-parse 'HEAD^{tree}')"
out_dir="${NCF_RECOVERY_OUTPUT_DIR:-${TMPDIR:-/tmp}/ncf-recovery-tlc-$source_commit}"
mkdir -p "$out_dir"

safe_log="$out_dir/safe-model.log"
mutant_log="$out_dir/commit-replay-mutant.log"
receipt="$out_dir/receipt.json"
safe_meta="$out_dir/states-safe"
mutant_meta="$out_dir/states-mutant"
mkdir -p "$safe_meta" "$mutant_meta"

model_sha="$(sha256sum "$model" | awk '{print $1}')"
config_sha="$(sha256sum "$config" | awk '{print $1}')"
mutant_sha="$(sha256sum "$mutant" | awk '{print $1}')"
mutant_config_sha="$(sha256sum "$mutant_config" | awk '{print $1}')"
jar_sha="$(sha256sum "$jar" | awk '{print $1}')"
java_version="$(java -version 2>&1 | sed -n '1p' | tr -cd '[:alnum:]._- ')"

(
  cd "$formal_dir"
  java -cp "$jar" tlc2.TLC -metadir "$safe_meta" -config "$(basename "$config")" "$(basename "$model")"
) >"$safe_log" 2>&1 || {
  cat "$safe_log" >&2
  echo "safe NCF-RECOVERY model failed; inspect $safe_log" >&2
  exit 1
}
safe_stats="$(grep -E 'states generated, .*distinct states found' "$safe_log" | tail -n 1 || true)"
if [[ -z "$safe_stats" ]]; then
  cat "$safe_log" >&2
  echo "could not identify TLC state statistics in the safe-model log" >&2
  exit 1
fi
safe_stats="$(printf '%s' "$safe_stats" | tr -cd '[:alnum:],. _-')"

status=0
set +e
(
  cd "$formal_dir"
  java -cp "$jar" tlc2.TLC -metadir "$mutant_meta" -config "$(basename "$mutant_config")" "$(basename "$mutant")"
) >"$mutant_log" 2>&1
status=$?
set -e
if [[ "$status" -eq 0 ]]; then
  cat "$mutant_log" >&2
  echo "unsafe commit-replay mutant unexpectedly passed" >&2
  exit 1
fi
if ! grep -Eiq "Invariant .* NoDoubleCommit .*violated|NoDoubleCommit.*violated" "$mutant_log"; then
  cat "$mutant_log" >&2
  echo "mutant failed, but not because NoDoubleCommit was violated" >&2
  exit 1
fi

safe_log_sha="$(sha256sum "$safe_log" | awk '{print $1}')"
mutant_log_sha="$(sha256sum "$mutant_log" | awk '{print $1}')"
cat >"$receipt" <<EOF
{
  "schema": "mycelix.ncf-recovery-tlc-receipt",
  "schema_version": "1.0.0",
  "source_commit": "$source_commit",
  "source_tree": "$source_tree",
  "safe_model_sha256": "$model_sha",
  "safe_config_sha256": "$config_sha",
  "commit_replay_mutant_sha256": "$mutant_sha",
  "commit_replay_mutant_config_sha256": "$mutant_config_sha",
  "tla2tools_jar_sha256": "$jar_sha",
  "java_version": "$java_version",
  "safe_model_status": "passed",
  "safe_model_state_statistics_line": "$safe_stats",
  "commit_replay_mutant_status": "expected_failure_detected",
  "commit_replay_mutant_expected_invariant": "NoDoubleCommit",
  "safe_log_sha256": "$safe_log_sha",
  "commit_replay_mutant_log_sha256": "$mutant_log_sha",
  "claim_ceiling": "bounded crash/recovery TLA+ abstraction only; production durability and Holochain enforcement require separate qualification"
}
EOF

echo "NCF-RECOVERY bounded formal campaign qualified for this exact checkout."
echo "receipt: $receipt"
echo "receipt sha256: $(sha256sum "$receipt" | awk '{print $1}')"
echo "safe log sha256: $safe_log_sha"
echo "mutant log sha256: $mutant_log_sha"
