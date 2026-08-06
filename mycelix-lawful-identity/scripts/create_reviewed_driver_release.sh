#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
driver="${MYCELIX_CONDUCTOR_DRIVER:-$root/tests/conductor/holochain_real_driver.py}"
control="${MYCELIX_HOLOCHAIN_CONTROL:-}"
config_output="${MYCELIX_HOLOCHAIN_DRIVER_CONFIG_OUTPUT:-}"
manifest_output="${MYCELIX_DRIVER_RELEASE_MANIFEST_OUTPUT:-}"
signature_output="${MYCELIX_DRIVER_RELEASE_SIGNATURE_OUTPUT:-}"
signing_key="${MYCELIX_DRIVER_REVIEW_SIGNING_KEY:-}"
signer_policy="${MYCELIX_SIGNER_POLICY:-}"
signer_policy_signature="${MYCELIX_SIGNER_POLICY_SIGNATURE:-}"
policy_root_public_key="${MYCELIX_POLICY_ROOT_PUBLIC_KEY:-}"
minimum_policy_version="${MYCELIX_MINIMUM_SIGNER_POLICY_VERSION:-}"
validity_seconds="${MYCELIX_DRIVER_RELEASE_VALIDITY_SECONDS:-2592000}"
scenario_contract="${MYCELIX_CONDUCTOR_SCENARIO_CONTRACT:-$root/tests/conductor/settlement_scenarios_v2.json}"
step_timeout="${MYCELIX_CONDUCTOR_STEP_TIMEOUT_SECONDS:-900}"

fail() { echo "create reviewed driver release: FAIL: $*" >&2; exit 2; }
regular() { [[ -n "$1" && -f "$1" && ! -L "$1" ]] || fail "$2 must be a regular non-symlink file"; }

regular "$driver" MYCELIX_CONDUCTOR_DRIVER
[[ -x "$driver" ]] || fail "MYCELIX_CONDUCTOR_DRIVER must be executable"
regular "$control" MYCELIX_HOLOCHAIN_CONTROL
[[ -x "$control" ]] || fail "MYCELIX_HOLOCHAIN_CONTROL must be executable"
regular "$scenario_contract" MYCELIX_CONDUCTOR_SCENARIO_CONTRACT
regular "$signing_key" MYCELIX_DRIVER_REVIEW_SIGNING_KEY
regular "$signer_policy" MYCELIX_SIGNER_POLICY
regular "$signer_policy_signature" MYCELIX_SIGNER_POLICY_SIGNATURE
regular "$policy_root_public_key" MYCELIX_POLICY_ROOT_PUBLIC_KEY
[[ "$minimum_policy_version" =~ ^[1-9][0-9]*$ ]] || fail "MYCELIX_MINIMUM_SIGNER_POLICY_VERSION must be positive"
[[ "$validity_seconds" =~ ^[1-9][0-9]*$ ]] || fail "MYCELIX_DRIVER_RELEASE_VALIDITY_SECONDS must be positive"
[[ "$step_timeout" =~ ^[1-9][0-9]*$ ]] || fail "MYCELIX_CONDUCTOR_STEP_TIMEOUT_SECONDS must be positive"
for output in "$config_output" "$manifest_output" "$signature_output"; do
  [[ -n "$output" ]] || fail "all output paths are required"
  [[ ! -e "$output" && ! -L "$output" ]] || fail "output already exists: $output"
  [[ -d "$(dirname "$output")" && ! -L "$(dirname "$output")" ]] || fail "output parent is invalid: $(dirname "$output")"
done

cd "$root"
python3 tests/conductor/holochain_driver_config.py \
  --create \
  --config "$config_output" \
  --control "$control" \
  --scenario-contract "$scenario_contract" \
  --step-timeout-seconds "$step_timeout"

hash_b64() {
  python3 - "$1" <<'PY_HASH'
import base64, hashlib, pathlib, sys
print(base64.b64encode(hashlib.sha256(pathlib.Path(sys.argv[1]).read_bytes()).digest()).decode("ascii"))
PY_HASH
}

now="$(python3 - <<'PY_NOW'
import time
print(time.time_ns() // 1000)
PY_NOW
)"
valid_from="$((now + 5000000))"
valid_until="$((valid_from + validity_seconds * 1000000))"
git_commit="$(git rev-parse HEAD)"
git_tree="$(git rev-parse 'HEAD^{tree}')"
build_recipe_hash="$(hash_b64 tests/conductor/REAL_DRIVER_BUILD_RECIPE_V1.md)"
lockfile_hash="$(hash_b64 tests/conductor/real_driver.lock)"

export MYCELIX_HOLOCHAIN_DRIVER_CONFIG="$(cd "$(dirname "$config_output")" && pwd -P)/$(basename "$config_output")"
python3 tests/conductor/driver_release_manifest.py \
  --create \
  --manifest "$manifest_output" \
  --signature "$signature_output" \
  --driver "$driver" \
  --signing-key "$signing_key" \
  --signer-policy "$signer_policy" \
  --signer-policy-signature "$signer_policy_signature" \
  --policy-root-public-key "$policy_root_public_key" \
  --minimum-policy-version "$minimum_policy_version" \
  --source-git-commit "$git_commit" \
  --source-git-tree "$git_tree" \
  --build-recipe-sha256 "$build_recipe_hash" \
  --lockfile-sha256 "$lockfile_hash" \
  --valid-from-unix-micros "$valid_from" \
  --valid-until-unix-micros "$valid_until"

chmod 0444 "$manifest_output" "$signature_output"
echo "create reviewed driver release: PASS"
echo "driver_config=$config_output"
echo "driver_release_manifest=$manifest_output"
echo "driver_release_signature=$signature_output"
