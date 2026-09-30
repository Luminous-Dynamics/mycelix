#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${root}/Cargo.toml"
tests="${root}/tests/Cargo.toml"
flake_lock="${root}/flake.lock"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

[[ -f "$workspace" ]] || fail "missing Hearth workspace Cargo.toml"
[[ -f "$tests" ]] || fail "missing Sweettest workspace Cargo.toml"
[[ -f "$flake_lock" ]] || fail "missing Hearth flake.lock"

require_exact() {
  local file="$1"
  local needle="$2"
  grep -Fq -- "$needle" "$file" || fail "$file does not contain required Holochain 0.7 declaration: $needle"
}

forbidden() {
  local file="$1"
  local needle="$2"
  if grep -Fq -- "$needle" "$file"; then
    fail "$file still contains forbidden pre-0.7 declaration: $needle"
  fi
}

# The production Hearth workspace must declare the 0.7 compatibility family.
require_exact "$workspace" 'hdk = "0.7.0"'
require_exact "$workspace" 'hdi = "0.8.0"'
require_exact "$workspace" 'holochain_integrity_types = "0.7.0"'
require_exact "$workspace" 'holochain_serialized_bytes = "0.0.57"'

# The dedicated Sweettest workspace is itself part of qualification and must not
# silently exercise the old 0.6 runtime/API.
require_exact "$tests" 'hdk = "0.7.0"'
require_exact "$tests" 'hdi = "0.8.0"'
require_exact "$tests" 'holochain = "0.7.0"'
require_exact "$tests" 'holochain_types = "0.7.0"'

for file in "$workspace" "$tests"; do
  forbidden "$file" 'hdk = "0.6'
  forbidden "$file" 'hdi = "0.7'
  forbidden "$file" 'holochain = "0.6'
  forbidden "$file" 'holochain_types = "0.6'
done

# The Nix lockfile is part of the qualification closure. The workflow must not
# silently resolve a Holochain 0.6 Holonix input while claiming 0.7.
require_exact "$flake_lock" '"ref": "holochain-0.7.0"'
require_exact "$flake_lock" '"ref": "main-0.7"'
require_exact "$flake_lock" '"ref": "v0.7.1"'
require_exact "$flake_lock" '"ref": "v0.5.0"'

echo "Holochain 0.7 source invariants: PASS"
echo "workspace=$workspace"
echo "tests=$tests"
echo "flake_lock=$flake_lock"
