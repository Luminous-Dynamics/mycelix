#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "$BASH_SOURCE")/.." && pwd)"
workspace="$root/Cargo.toml"
tests="$root/tests/Cargo.toml"
flake_lock="$root/flake.lock"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

[[ -f "$workspace" ]] || fail "missing Hearth workspace Cargo.toml"
[[ -f "$tests" ]] || fail "missing Sweettest workspace Cargo.toml"
[[ -f "$flake_lock" ]] || fail "missing Hearth flake.lock"

require_version_family() {
  local file="$1"
  local name="$2"
  local version="$3"
  if ! grep -Eq -- "^$name[[:space:]]*=[[:space:]]*(\"?=?$version\"?|\\{[^}]*version[[:space:]]*=[[:space:]]*\"=?$version\"\\})" "$file"; then
    fail "$file does not declare $name in the required $version family"
  fi
}

forbidden() {
  local file="$1"
  local needle="$2"
  if grep -Fq -- "$needle" "$file"; then
    fail "$file still contains forbidden pre-0.7 declaration: $needle"
  fi
}

require_version_family "$workspace" "hdk" "0\\.7\\.0"
require_version_family "$workspace" "hdi" "0\\.8\\.0"
require_version_family "$workspace" "holochain_integrity_types" "0\\.7\\.0"
require_version_family "$workspace" "holochain_serialized_bytes" "0\\.0\\.57"

require_version_family "$tests" "hdk" "0\\.7\\.0"
require_version_family "$tests" "hdi" "0\\.8\\.0"
require_version_family "$tests" "holochain" "0\\.7\\.0"
require_version_family "$tests" "holochain_types" "0\\.7\\.0"

for file in "$workspace" "$tests"; do
  forbidden "$file" 'hdk = "0.6'
  forbidden "$file" 'hdi = "0.7'
  forbidden "$file" 'holochain = "0.6'
  forbidden "$file" 'holochain_types = "0.6'
done

python3 - "$flake_lock" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, encoding="utf-8") as f:
    lock = json.load(f)

nodes = lock.get("nodes", {})
def original_ref(node):
    return nodes.get(node, {}).get("original", {}).get("ref")

def require(node, predicate, description):
    ref = original_ref(node)
    if not predicate(ref):
        raise SystemExit(f"FAIL: flake.lock {node}.original.ref={ref!r}; expected {description}")

require("holonix", lambda r: r == "main-0.7", "main-0.7")
require("holochain", lambda r: isinstance(r, str) and r.startswith("holochain-0.7"), "a holochain-0.7* ref")
require("lair-keystore", lambda r: isinstance(r, str) and r.startswith("v0.7"), "a v0.7* ref")
require("kitsune2", lambda r: isinstance(r, str) and r.startswith("v0.5"), "a v0.5* ref")

print("Holochain 0.7 Nix lock invariants: PASS")
PY

echo "Holochain 0.7 source invariants: PASS"
echo "workspace=$workspace"
echo "tests=$tests"
echo "flake_lock=$flake_lock"
