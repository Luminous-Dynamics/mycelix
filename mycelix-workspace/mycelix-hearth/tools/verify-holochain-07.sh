#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

fail=0

require() {
  local file="$1"
  local pattern="$2"
  local label="$3"
  if ! grep -Fq -- "$pattern" "$file"; then
    echo "FAIL: $label"
    echo "  expected: $pattern"
    echo "  file: $file"
    fail=1
  else
    echo "PASS: $label"
  fi
}

forbid() {
  local file="$1"
  local pattern="$2"
  local label="$3"
  if grep -Fq -- "$pattern" "$file"; then
    echo "FAIL: $label"
    echo "  forbidden: $pattern"
    echo "  file: $file"
    fail=1
  else
    echo "PASS: $label"
  fi
}

cargo_toml="Cargo.toml"
cargo_lock="Cargo.lock"
tests_cargo_lock="tests/Cargo.lock"
flake_nix="flake.nix"
flake_lock="flake.lock"
sdk_package="sdk-ts/package.json"
sdk_lock="sdk-ts/package-lock.json"

require "$cargo_toml" 'hdk = "=0.7.0"' "HDK is pinned to Holochain 0.7"
require "$cargo_toml" 'hdi = "=0.8.0"' "HDI is pinned to Holochain 0.7"
require "$cargo_toml" 'holochain_integrity_types = "=0.7.0"' "integrity types are pinned to 0.7"
require "$cargo_lock" 'name = "hdk"' "Hearth Cargo lock contains HDK"
require "$cargo_lock" 'version = "0.7.0"' "Hearth Cargo lock contains a 0.7.0 package"
require "$tests_cargo_lock" 'name = "holochain"' "Sweettest Cargo lock contains Holochain"
require "$tests_cargo_lock" 'name = "holochain_types"' "Sweettest Cargo lock contains Holochain types"
require "$tests_cargo_lock" 'version = "0.7.0"' "Sweettest Cargo lock contains a 0.7.0 package"
require "$flake_nix" 'ref=main-0.7' "Holonix declaration targets main-0.7"
require "$sdk_package" '"@holochain/client": "^0.21.0"' "JS manifest targets client 0.21"
require "$sdk_lock" '"@holochain/client": "^0.21.0"' "JS lock root targets client 0.21"
require "$sdk_lock" '"node_modules/@holochain/client": {' "JS lock contains the client package"
require "$sdk_lock" '"version": "0.21.0"' "JS lock resolves a 0.21 client"
forbid "$sdk_lock" '"version": "0.20.2"' "JS lock no longer resolves client 0.20.2"

# A declaration is not enough: the checked-in Nix graph must not retain the
# 0.6-era Holonix component set.
forbid "$flake_lock" '"ref": "holochain-0.6.0"' "lockfile no longer resolves Holochain 0.6"
forbid "$flake_lock" '"ref": "0.600.0-dev.0"' "lockfile no longer resolves hc-scaffold 0.6"
forbid "$flake_lock" '"ref": "v0.6.3"' "lockfile no longer resolves Lair 0.6"
forbid "$flake_lock" '"ref": "v0.3.2"' "lockfile no longer resolves Kitsune2 0.3"
forbid "$cargo_lock" 'name = "hdk"\nversion = "0.6' "Cargo lock contains no Holochain 0.6 HDK"
forbid "$cargo_lock" 'name = "hdi"\nversion = "0.7.1' "Cargo lock contains no Holochain 0.6 HDI"
forbid "$tests_cargo_lock" 'name = "holochain"\nversion = "0.6' "Sweettest lock contains no Holochain 0.6 conductor"

# The dependency floor is not source qualification. HDI 0.8 uses the 0.7
# FlatOp vocabulary, so reject known 0.6 validation dispatcher patterns.
legacy_source=0
while IFS= read -r -d '' file; do
  for pattern in     'FlatOp::StoreEntry'     'FlatOp::RegisterCreateLink'     'FlatOp::RegisterDeleteLink'     'FlatOp::RegisterUpdate'     'FlatOp::RegisterDelete'     'FlatOp::RegisterAgentActivity'     'Action::Create'     'Action::Update'     'Action::Delete'     'SignedActionHashed<'     'signal_url'     'webrtc_config'     'transport-iroh'     'sqlite-encrypted'     'wasmer_sys'
  do
    if grep -Fq -- "$pattern" "$file"; then
      echo "FAIL: legacy Holochain 0.6 source pattern '$pattern' in $file"
      legacy_source=1
    fi
  done
done < <(find zomes -path '*/src/*.rs' -type f -print0)

# Coordinator and SDK surfaces are part of the 0.7 migration too; catch stale
# generic action types/imports and removed transport configuration outside the
# integrity dispatchers.
while IFS= read -r -d '' file; do
  for pattern in 'SignedActionHashed<' 'signal_url' 'webrtc_config' 'transport-iroh' 'sqlite-encrypted' 'wasmer_sys'; do
    if grep -Fq -- "$pattern" "$file"; then
      echo "FAIL: legacy Holochain 0.6 pattern '$pattern' in $file"
      legacy_source=1
    fi
  done
done < <(find zomes tests -type f \( -name '*.rs' -o -name '*.toml' -o -name '*.ts' -o -name '*.json' \) -print0)

if [[ "$legacy_source" -ne 0 ]]; then
  fail=1
fi

if [[ "$fail" -ne 0 ]]; then
  echo
  echo "Holochain 0.7 qualification guard: FAIL"
  echo "Resolve dependency/lock/source blockers before labeling Hearth 0.7-qualified."
  exit 1
fi

echo "Holochain 0.7 qualification guard: PASS"
