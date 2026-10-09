#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
manifest="${root}/mycelix-governance/Cargo.toml"

fail() {
  echo "GOV-TOOLCHAIN-007C0: $*" >&2
  exit 1
}

[[ -f "$manifest" ]] || fail "missing canonical Governance workspace manifest: $manifest"

grep -Fq 'hdk = "=0.7.0"' "$manifest" || fail "Governance HDK is not pinned to 0.7.0"
grep -Fq 'hdi = "=0.8.0"' "$manifest" || fail "Governance HDI is not pinned to 0.8.0"
grep -Fq 'holochain_integrity_types = "=0.7.0"' "$manifest" || fail "Governance integrity types are not pinned to 0.7.0"
grep -Fq 'holochain_zome_types = "=0.7.0"' "$manifest" || fail "Governance zome types are not pinned to 0.7.0"
grep -Fq 'holo_hash = "=0.7.0"' "$manifest" || fail "Governance holo_hash is not pinned to 0.7.0"
grep -Fq 'hdk_derive = "=0.7.0"' "$manifest" || fail "Governance hdk_derive is not pinned to 0.7.0"
grep -Fq 'holochain_serialized_bytes = "=0.0.57"' "$manifest" || fail "Governance serialized bytes are not pinned to 0.0.57"

# Match complete Rust enum paths and variant tokens. Without identifier
# boundaries, Action::Update also matches the suffix of GovernanceAction::UpdateParameter.
legacy_pattern='(^|[^[:alnum:]_])(FlatOp::(StoreEntry|StoreRecord|RegisterUpdate|RegisterDelete|RegisterCreateLink|RegisterDeleteLink|RegisterAgentActivity)|Action::(Create|Update|Delete|CreateLink|DeleteLink))([^[:alnum:]_]|$)'
if grep -RInE "$legacy_pattern" "$root/mycelix-governance/zomes" >/tmp/governance-legacy-action-model.txt; then
  cat /tmp/governance-legacy-action-model.txt >&2
  fail "legacy Holochain 0.6 action/FlatOp forms remain in Governance integrity/coordinator sources"
fi

for zome in   budgeting proposals voting execution threshold-signing constitution councils jurisdiction bridge; do
  [[ -f "$root/mycelix-governance/zomes/$zome/integrity/src/lib.rs" ]] ||
    fail "missing active Governance integrity zome: $zome"
done

echo "GOV-TOOLCHAIN-007C0: canonical standalone Governance tree has the required Holochain 0.7 dependency tuple and no legacy 0.6 action/FlatOp forms."
