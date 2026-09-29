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

check_lock_package() {
  local file="$1"
  local package="$2"
  local version="$3"
  local label="$4"
  if grep -A1 -F "name = \"$package\"" "$file" | grep -Fq "version = \"$version\""; then
    echo "PASS: $label"
  else
    echo "FAIL: $label"
    echo "  expected: $package $version"
    echo "  file: $file"
    fail=1
  fi
}

cargo_toml="Cargo.toml"
cargo_lock="Cargo.lock"
tests_toml="tests/Cargo.toml"
tests_lock="tests/Cargo.lock"
flake_nix="flake.nix"
flake_lock="flake.lock"
sdk_package="sdk-ts/package.json"
sdk_lock="sdk-ts/package-lock.json"

require "$cargo_toml" 'hdk = "=0.7.0"' "Hearth HDK is pinned to 0.7.0"
require "$cargo_toml" 'hdi = "=0.8.0"' "Hearth HDI is pinned to 0.8.0"
require "$cargo_toml" 'holochain_integrity_types = "=0.7.0"' "Hearth integrity types are pinned to 0.7.0"
require "$tests_toml" 'hdk = "=0.7.0"' "Sweettest HDK is pinned to 0.7.0"
require "$tests_toml" 'hdi = "=0.8.0"' "Sweettest HDI is pinned to 0.8.0"
require "$tests_toml" 'holochain = { version = "0.7.0"' "Sweettest conductor is pinned to 0.7.0"
require "$tests_toml" 'holochain_types = "0.7.0"' "Sweettest Holochain types are pinned to 0.7.0"
require "$flake_nix" 'ref=main-0.7' "Holonix declaration targets main-0.7"
require "$sdk_package" '"@holochain/client": "^0.21.0"' "SDK manifest targets client 0.21"

check_lock_package "$cargo_lock" "hdk" "0.7.0" "Cargo lock pins HDK 0.7.0"
check_lock_package "$cargo_lock" "hdi" "0.8.0" "Cargo lock pins HDI 0.8.0"
check_lock_package "$cargo_lock" "holochain_integrity_types" "0.7.0" "Cargo lock pins integrity types 0.7.0"
check_lock_package "$cargo_lock" "holo_hash" "0.7.0" "Cargo lock pins holo_hash 0.7.0"
check_lock_package "$cargo_lock" "holochain_serialized_bytes" "0.0.57" "Cargo lock pins serialized bytes 0.0.57"

check_lock_package "$tests_lock" "hdk" "0.7.0" "Sweettest lock pins HDK 0.7.0"
check_lock_package "$tests_lock" "hdi" "0.8.0" "Sweettest lock pins HDI 0.8.0"
check_lock_package "$tests_lock" "holochain" "0.7.0" "Sweettest lock pins Holochain 0.7.0"
check_lock_package "$tests_lock" "holochain_types" "0.7.0" "Sweettest lock pins Holochain types 0.7.0"

require "$sdk_lock" '"@holochain/client": "^0.21.0"' "SDK lock root targets client 0.21"
require "$sdk_lock" '"version": "0.21.0"' "SDK lock resolves client 0.21.0"
forbid "$sdk_lock" '"version": "0.20.2"' "SDK lock no longer resolves client 0.20.2"

forbid "$flake_lock" '"ref": "holochain-0.6.0"' "Nix lock no longer resolves Holochain 0.6"
forbid "$flake_lock" '"ref": "0.600.0-dev.0"' "Nix lock no longer resolves hc-scaffold 0.6"
forbid "$flake_lock" '"ref": "v0.6.3"' "Nix lock no longer resolves Lair 0.6"
forbid "$flake_lock" '"ref": "v0.3.2"' "Nix lock no longer resolves Kitsune2 0.3"

if [[ "$fail" -ne 0 ]]; then
  echo
  echo "Holochain 0.7 dependency-lock guard: FAIL"
  exit 1
fi

echo "Holochain 0.7 dependency-lock guard: PASS"
