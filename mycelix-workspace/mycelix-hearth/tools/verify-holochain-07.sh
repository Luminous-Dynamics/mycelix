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
flake_nix="flake.nix"
flake_lock="flake.lock"
sdk_package="sdk-ts/package.json"

require "$cargo_toml" 'hdk = "=0.7.0"' "HDK is pinned to Holochain 0.7"
require "$cargo_toml" 'hdi = "=0.8.0"' "HDI is pinned to Holochain 0.7"
require "$cargo_toml" 'holochain_integrity_types = "=0.7.0"' "integrity types are pinned to 0.7"
require "$flake_nix" 'ref=main-0.7' "Holonix declaration targets main-0.7"
require "$sdk_package" '"@holochain/client": "^0.21.0"' "JS client targets 0.21"

# A declaration is not enough: the checked-in Nix graph must not retain the
# 0.6-era Holonix component set.
forbid "$flake_lock" '"ref": "holochain-0.6.0"' "lockfile no longer resolves Holochain 0.6"
forbid "$flake_lock" '"ref": "0.600.0-dev.0"' "lockfile no longer resolves hc-scaffold 0.6"
forbid "$flake_lock" '"ref": "v0.6.3"' "lockfile no longer resolves Lair 0.6"
forbid "$flake_lock" '"ref": "v0.3.2"' "lockfile no longer resolves Kitsune2 0.3"

if [[ "$fail" -ne 0 ]]; then
  echo
  echo "Holochain 0.7 qualification guard: FAIL"
  echo "Regenerate mycelix-hearth/flake.lock from the 0.7 Holonix declaration before qualifying the branch."
  exit 1
fi

echo "Holochain 0.7 qualification guard: PASS"
