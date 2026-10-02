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

require_file() {
  local file="$1"
  local label="$2"
  if [[ ! -f "$file" ]]; then
    echo "FAIL: $label"
    echo "  missing: $file"
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
tests_cargo_toml="tests/Cargo.toml"
tests_cargo_lock="tests/Cargo.lock"
flake_nix="flake.nix"
flake_lock="flake.lock"
sdk_package="sdk-ts/package.json"
sdk_lock="sdk-ts/package-lock.json"
require_file "$cargo_lock" "Hearth Cargo lock is generated and committed"
require_file "$tests_cargo_lock" "Sweettest Cargo lock is generated and committed"

require "$cargo_toml" 'hdk = "=0.7.0"' "HDK is pinned to Holochain 0.7"
require "$cargo_toml" 'hdi = "=0.8.0"' "HDI is pinned to Holochain 0.7"
require "$cargo_toml" 'holochain_integrity_types = "=0.7.0"' "integrity types are pinned to 0.7"
require "$tests_cargo_toml" 'hdk = "=0.7.0"' "Sweettest HDK is pinned to 0.7"
require "$tests_cargo_toml" 'hdi = "=0.8.0"' "Sweettest HDI is pinned to 0.8"
require "$tests_cargo_toml" 'holochain = { version = "0.7.0"' "Sweettest conductor targets 0.7"
require "$tests_cargo_toml" 'holochain_types = "0.7.0"' "Sweettest types target 0.7"
require "$tests_cargo_toml" 'wasmer-sys-cranelift' "Sweettest uses the Holochain 0.7 Wasmer feature"
for spec in   'hdk|0.7.0'   'hdi|0.8.0'   'holochain_integrity_types|0.7.0'   'holo_hash|0.7.0'
do
  package="${spec%%|*}"
  version="${spec#*|}"
  if grep -A1 -F "name = \"$package\"" "$cargo_lock" | grep -Fq "version = \"$version\""; then
    echo "PASS: Cargo lock pins $package $version"
  else
    echo "FAIL: Cargo lock does not pin $package $version"
    fail=1
  fi
done

for spec in   'holochain|0.7.0'   'holochain_types|0.7.0'
do
  package="${spec%%|*}"
  version="${spec#*|}"
  if grep -A1 -F "name = \"$package\"" "$tests_cargo_lock" | grep -Fq "version = \"$version\""; then
    echo "PASS: Sweettest lock pins $package $version"
  else
    echo "FAIL: Sweettest lock does not pin $package $version"
    fail=1
  fi
done
require "$flake_nix" 'ref=main-0.7' "Holonix declaration targets main-0.7"
require "$sdk_package" '"@holochain/client": "^0.21.0"' "JS manifest targets client 0.21"
require "$sdk_lock" '"@holochain/client": "^0.21.0"' "JS lock root targets client 0.21"
require "$sdk_lock" '"node_modules/@holochain/client": {' "JS lock contains the client package"
require "$sdk_lock" '"version": "0.21.0"' "JS lock resolves a 0.21 client"
forbid "$sdk_lock" '"version": "0.20.2"' "JS lock no longer resolves client 0.20.2"

# A dependency declaration is not enough: the checked-in Nix graph must
# positively identify the intended Holochain 0.7 component family.
require "$flake_lock" '"ref": "holochain-0.7.0"' "Nix lock resolves Holochain 0.7.0"
require "$flake_lock" '"ref": "v0.5.0"' "Nix lock resolves Kitsune2 0.5.0"
require "$flake_lock" '"ref": "v0.7.1"' "Nix lock resolves Lair 0.7.1"

# Retain explicit negative checks as defense in depth.
forbid "$flake_lock" '"ref": "holochain-0.6.0"' "lockfile no longer resolves Holochain 0.6"
forbid "$flake_lock" '"ref": "0.600.0-dev.0"' "lockfile no longer resolves hc-scaffold 0.6"
forbid "$flake_lock" '"ref": "v0.6.3"' "lockfile no longer resolves Lair 0.6"
forbid "$flake_lock" '"ref": "v0.3.2"' "lockfile no longer resolves Kitsune2 0.3"
if grep -A1 -F 'name = "hdk"' "$cargo_lock" | grep -Fq 'version = "0.6'; then
  echo "FAIL: Cargo lock contains Holochain 0.6 HDK"
  fail=1
else
  echo "PASS: Cargo lock contains no Holochain 0.6 HDK"
fi

if grep -A1 -F 'name = "hdi"' "$cargo_lock" | grep -Fq 'version = "0.7.1'; then
  echo "FAIL: Cargo lock contains Holochain 0.6 HDI"
  fail=1
else
  echo "PASS: Cargo lock contains no Holochain 0.6 HDI"
fi

if grep -A1 -F 'name = "holochain"' "$tests_cargo_lock" | grep -Fq 'version = "0.6'; then
  echo "FAIL: Sweettest lock contains Holochain 0.6 conductor"
  fail=1
else
  echo "PASS: Sweettest lock contains no Holochain 0.6 conductor"
fi



# The dependency floor is not source qualification. HDI 0.8 uses the 0.7
# FlatOp vocabulary, so reject known 0.6 validation dispatcher patterns.
legacy_source=0
while IFS= read -r -d '' file; do
  for pattern in     'FlatOp::StoreEntry'     'FlatOp::RegisterCreateLink'     'FlatOp::RegisterDeleteLink'     'FlatOp::RegisterUpdate'     'FlatOp::RegisterDelete'     'FlatOp::RegisterAgentActivity'     'Action::Create'     'Action::Update'     'Action::Delete'     'EntryCreationAction'     'ActionBuilderCommon'     'ActionBuilder'     'NewEntryAction'     'NewEntryActionRef'     'SignedActionHashed<'     'signal_url'     'webrtc_config'     'transport-iroh'     'sqlite-encrypted'     'wasmer_sys'
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
