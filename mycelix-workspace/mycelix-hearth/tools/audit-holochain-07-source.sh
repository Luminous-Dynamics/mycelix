#!/usr/bin/env bash
# HEARTH-0.7 source migration audit.
# Deterministic, read-only: exits non-zero on any known 0.6 API/config residue.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

fail=0
check_absent() {
  local label="$1"
  local pattern="$2"
  if git grep -nE -- "$pattern" --       'mycelix-workspace/mycelix-hearth/**/*.rs'       'mycelix-workspace/mycelix-hearth/**/*.ts'       'mycelix-workspace/mycelix-hearth/**/*.tsx'       'mycelix-workspace/mycelix-hearth/**/*.js'       'mycelix-workspace/mycelix-hearth/**/*.json'       'mycelix-workspace/mycelix-hearth/**/*.toml'       'mycelix-workspace/mycelix-hearth/**/*.yaml'       'mycelix-workspace/mycelix-hearth/**/*.yml'       'mycelix-workspace/mycelix-hearth/**/*.nix'       2>/dev/null; then
    echo "FAIL: $label"
    fail=1
  else
    echo "OK:   $label"
  fi
}

check_present() {
  local label="$1"
  local pattern="$2"
  if git grep -nE -- "$pattern" -- 'mycelix-workspace/mycelix-hearth/**/*.toml' 'mycelix-workspace/mycelix-hearth/**/*.nix'; then
    echo "OK:   $label"
  else
    echo "FAIL: $label"
    fail=1
  fi
}

check_lock_present() {
  local label="$1"
  local pattern="$2"
  if rg -n --pcre2 "$pattern" mycelix-workspace/mycelix-hearth/flake.lock >/dev/null; then
    echo "OK:   $label"
  else
    echo "FAIL: $label"
    fail=1
  fi
}

check_absent_multiline() {
  local label="$1"
  local pattern="$2"
  if rg -nU --pcre2 "$pattern" mycelix-workspace/mycelix-hearth; then
    echo "FAIL: $label"
    fail=1
  else
    echo "OK:   $label"
  fi
}

echo "HEARTH-0.7 deterministic source audit"
echo "HEAD: $(git rev-parse HEAD)"
echo

# Legacy Rust action model / removed APIs.
check_absent "legacy FlatOp variants" 'FlatOp::(StoreEntry|StoreRecord|RegisterUpdate|RegisterDelete|RegisterCreateLink|RegisterDeleteLink|RegisterAgentActivity)'
check_absent "legacy Action enum variants" 'Action::(Create|Update|Delete|CreateLink|DeleteLink|Dna|AgentValidationPkg|InitZomesComplete|OpenChain|CloseChain)([({])'
check_absent "removed action builders" '\b(ActionBuilder|ActionBuilderCommon|NewEntryAction|NewEntryActionRef)\b'
check_absent "removed EntryCreationAction" '\bEntryCreationAction\b'
check_absent "removed agent blocking APIs" '\b(block_agent|unblock_agent)\s*\('
check_absent "old generic SignedActionHashed" 'SignedActionHashed\s*<'
check_absent "old transport/config symbols" '\b(signal_url|webrtc_config|transport-iroh|wasmer_sys|sqlite-encrypted)\b'

# Old client/test package names and 0.6 version declarations.
check_absent "legacy Tryorama package" '@holochain/tryorama'
check_absent "legacy Holochain 0.6 package versions" '(@holochain/client[^0-9]*0\.20\.|hdk[^0-9]*0\.6\.|hdi[^0-9]*0\.7\.|holochain[^0-9]*0\.6\.)'

# Subtle 0.7 source/API changes that can compile incorrectly or regress at runtime.
check_absent "legacy DnaStorageInfo size fields" '\b(authored_data_size|cache_data_size)\b'
check_absent "legacy client WebRTC predicate" '\bis_webrtc\b'
check_absent "legacy client signaling-server field" '\bsignalingServerUrl\b'
check_absent "legacy config sync strategy field" '\bdb_sync_strategy\b'
check_absent "legacy 0.6 sandbox transport spelling" '\bwebrtc\b'
check_absent_multiline "removed ChainFilter builder methods" 'ChainFilter::new\([^)]*\)[[:space:]]*\.[[:space:]]*(until_hash|take|until_timestamp)[[:space:]]*\('

# 0.7 dependency floor must be visible in Hearth manifests.
check_present "Hearth HDK/HDI 0.7 dependency floor" 'hdk\s*=\s*"=0\.7\.0"|hdi\s*=\s*"=0\.8\.0"'
check_present "Hearth flake uses Holonix main-0.7" 'holonix.*ref=main-0\.7'
check_lock_present "flake.lock pins Holochain 0.7.0" '"original"[[:space:]]*:[[:space:]]*\{[[:space:]]*"owner"[[:space:]]*:[[:space:]]*"holochain"[[:space:]]*,[[:space:]]*"ref"[[:space:]]*:[[:space:]]*"holochain-0\.7\.0"'
check_lock_present "flake.lock pins Kitsune2 0.5.0" '"original"[[:space:]]*:[[:space:]]*\{[[:space:]]*"owner"[[:space:]]*:[[:space:]]*"holochain"[[:space:]]*,[[:space:]]*"ref"[[:space:]]*:[[:space:]]*"v0\.5\.0"'
check_lock_present "flake.lock pins Lair 0.7.1" '"original"[[:space:]]*:[[:space:]]*\{[[:space:]]*"owner"[[:space:]]*:[[:space:]]*"holochain"[[:space:]]*,[[:space:]]*"ref"[[:space:]]*:[[:space:]]*"v0\.7\.1"'
check_lock_present "flake.lock pins Holonix main-0.7" '"original"[[:space:]]*:[[:space:]]*\{[[:space:]]*"owner"[[:space:]]*:[[:space:]]*"holochain"[[:space:]]*,[[:space:]]*"ref"[[:space:]]*:[[:space:]]*"main-0\.7"'

# Coordinator/client action access must use the 0.7 header/data split where action
# content is inspected. This is intentionally a presence audit, not a style gate.
if git grep -nE -- '\bActionData::|\.hashed\.content\.header\.(author|timestamp)|\.hashed\.content\.data' --     'mycelix-workspace/mycelix-hearth/**/*.rs'     'mycelix-workspace/mycelix-hearth/**/*.ts'     'mycelix-workspace/mycelix-hearth/**/*.tsx' >/dev/null 2>&1; then
  echo "OK:   0.7 action header/data access present"
else
  echo "WARN: no explicit ActionData/header access found in Hearth source"
fi

echo
if [[ "$fail" -ne 0 ]]; then
  echo "HEARTH-0.7 source audit: FAIL"
  exit "$fail"
fi
echo "HEARTH-0.7 source audit: PASS"
