#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "$BASH_SOURCE")/.." && pwd)"
workspace="$root/Cargo.toml"
tests="$root/tests/Cargo.toml"
flake_lock="$root/flake.lock"
sdk_package="$root/sdk-ts/package.json"
sdk_lock="$root/sdk-ts/package-lock.json"

fail() { echo "FAIL: $*" >&2; exit 1; }

[[ -f "$workspace" ]] || fail "missing Hearth workspace Cargo.toml"
[[ -f "$tests" ]] || fail "missing Sweettest workspace Cargo.toml"
[[ -f "$flake_lock" ]] || fail "missing Hearth flake.lock"
[[ -f "$sdk_package" ]] || fail "missing Hearth SDK package.json"
[[ -f "$sdk_lock" ]] || fail "missing Hearth SDK package-lock.json"

require_version_family() {
  local file="$1" name="$2" version="$3"
  grep -Eq -- "^$name[[:space:]]*=[[:space:]]*(\"?=?$version\"?|\\{[^}]*version[[:space:]]*=[[:space:]]*\"=?$version\"\\})" "$file" ||
    fail "$file does not declare $name in required $version family"
}

forbidden() {
  local file="$1" needle="$2"
  ! grep -Fq -- "$needle" "$file" || fail "$file still contains forbidden pre-0.7 declaration: $needle"
}

require_version_family "$workspace" "hdk" "0\\.7\\.0"
require_version_family "$workspace" "hdi" "0\\.8\\.0"
require_version_family "$workspace" "holochain_integrity_types" "0\\.7\\.0"
require_version_family "$workspace" "holochain_serialized_bytes" "0\\.0\\.57"
require_version_family "$tests" "hdk" "0\\.7\\.0"
require_version_family "$tests" "hdi" "0\\.8\\.0"
require_version_family "$tests" "holochain" "0\\.7\\.0"
require_version_family "$tests" "holochain_types" "0\\.7\\.0"

grep -Fq '"@holochain/client": "^0.21.0"' "$sdk_package" || fail "$sdk_package does not declare @holochain/client ^0.21.0"
grep -Fq '"node": ">=24.0.0"' "$sdk_package" || fail "$sdk_package does not require Node >=24.0.0"

flake_source="$root/flake.nix"
grep -Fq 'extraBuildInputs = with pkgs; [ nodejs_24 ];' "$flake_source" || fail "$flake_source default shell does not provide nodejs_24"
grep -Fq '              nodejs_24' "$flake_source" || fail "$flake_source CI shell does not provide nodejs_24"
grep -Fq '              perl' "$flake_source" || fail "$flake_source CI shell does not provide perl for Sweettest builds"

for file in "$workspace" "$tests"; do
  forbidden "$file" 'hdk = "0.6'
  forbidden "$file" 'hdi = "0.7'
  forbidden "$file" 'holochain = "0.6'
  forbidden "$file" 'holochain_types = "0.6'
done

python3 - "$flake_lock" <<'PY'
import json, sys
with open(sys.argv[1], encoding="utf-8") as f: lock = json.load(f)
nodes = lock.get("nodes", {})
def ref(node): return nodes.get(node, {}).get("original", {}).get("ref")
def require(node, pred, desc):
    value = ref(node)
    if not pred(value): raise SystemExit(f"FAIL: flake.lock {node}.original.ref={value!r}; expected {desc}")
require("holonix", lambda r: r == "main-0.7", "main-0.7")
print("Holochain 0.7 Nix lock invariants: PASS")
PY
echo "Holochain 0.7 source invariants: PASS"
