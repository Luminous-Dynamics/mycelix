#!/usr/bin/env bash
set -Eeuo pipefail

# Reproducible bounded TLC campaign for the NCF-FEN-1 safe model and its
# deliberately unsafe timeout-only transfer mutant. The mutant must fail for
# the expected safety invariant; a parse/tool failure is not a successful test.

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
mutant="$formal_dir/NCF_FEN_1_TimeoutMutant.tla"
mutant_config="$formal_dir/NCF_FEN_1_TimeoutMutant.cfg"
expected_invariant="NoTransferWithoutFence"

for path in "$model" "$config" "$mutant" "$mutant_config"; do
  [[ -f "$path" ]] || { echo "missing model artifact: $path" >&2; exit 2; }
done

out_dir="${NCF_TLC_OUTPUT_DIR:-${TMPDIR:-/tmp}/ncf-fen-tlc-$source_commit}"
mkdir -p "$out_dir"
safe_log="$out_dir/safe-model.log"
mutant_log="$out_dir/timeout-mutant.log"
receipt="$out_dir/receipt.json"

model_sha="$(sha256sum "$model" | awk '{print $1}')"
config_sha="$(sha256sum "$config" | awk '{print $1}')"
mutant_sha="$(sha256sum "$mutant" | awk '{print $1}')"
mutant_config_sha="$(sha256sum "$mutant_config" | awk '{print $1}')"
jar_sha="$(sha256sum "$jar" | awk '{print $1}')"
java_version="$(java -version 2>&1 | sed -n '1p' | tr -cd '[:alnum:]._- ')"

(
  cd "$formal_dir"
  java -cp "$jar" tlc2.TLC -config "$(basename "$config")" "$(basename "$model")"
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

set +e
(
  cd "$formal_dir"
  java -cp "$jar" tlc2.TLC -config "$(basename "$mutant_config")" "$(basename "$mutant")"
) >"$mutant_log" 2>&1
mutant_status=$?
set -e

if [[ "$mutant_status" -eq 0 ]]; then
  cat "$mutant_log" >&2
  echo "unsafe timeout mutant unexpectedly passed all invariants" >&2
  exit 1
fi
if ! grep -Eiq "Invariant .*${expected_invariant}.*violated|${expected_invariant}.*violated" "$mutant_log"; then
  cat "$mutant_log" >&2
  echo "mutant failed, but not with the expected $expected_invariant counterexample" >&2
  exit 1
fi

safe_log_sha="$(sha256sum "$safe_log" | awk '{print $1}')"
mutant_log_sha="$(sha256sum "$mutant_log" | awk '{print $1}')"
cat >"$receipt" <<EOF
{
  "schema": "mycelix.ncf-fen-tlc-receipt",
  "schema_version": "1.0.0",
  "source_commit": "$source_commit",
  "source_tree": "$source_tree",
  "safe_model_sha256": "$model_sha",
  "safe_config_sha256": "$config_sha",
  "mutant_model_sha256": "$mutant_sha",
  "mutant_config_sha256": "$mutant_config_sha",
  "tla2tools_jar_sha256": "$jar_sha",
  "java_version": "$java_version",
  "safe_model_status": "passed",
  "negative_control_status": "expected_failure_detected",
  "negative_control_invariant": "$expected_invariant",
  "safe_model_state_statistics_line": "$safe_stats",
  "safe_log_sha256": "$safe_log_sha",
  "negative_control_log_sha256": "$mutant_log_sha",
  "claim_ceiling": "bounded TLA+ abstraction only; not a proof of production Holochain, cryptographic authority, or unbounded distributed safety"
}
EOF

echo "NCF-FEN bounded formal campaign qualified for this exact checkout."
receipt_sha="$(sha256sum "$receipt" | awk '{print $1}')"
echo "receipt: $receipt"
echo "receipt sha256: $receipt_sha"
echo "safe log sha256: $safe_log_sha"
echo "expected-negative-control log sha256: $mutant_log_sha"
