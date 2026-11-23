#!/usr/bin/env bash
set -euo pipefail

FINANCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${1:-${RUNNER_TEMP:-${TMPDIR:-/tmp}}/holochain-runtime-evidence}"
mkdir -p "$OUT_DIR"
cd "$FINANCE_ROOT"

for cmd in git nix jq sha256sum readlink; do
  command -v "$cmd" >/dev/null 2>&1 || {
    echo "Required command not found: $cmd" >&2
    exit 127
  }
done

subject_commit="$(git -C .. rev-parse HEAD)"
nix_version="$(nix --version)"
flake_metadata="$OUT_DIR/flake-metadata.json"
nix flake metadata --json --no-write-lock-file . > "$flake_metadata"

holonix_rev="$(jq -r '.locks.nodes.holonix.locked.rev // empty' "$flake_metadata")"
holochain_ref="$(jq -r '.locks.nodes.holochain.original.ref // empty' "$flake_metadata")"
holochain_rev="$(jq -r '.locks.nodes.holochain.locked.rev // empty' "$flake_metadata")"

test "$holonix_rev" = "d21b35431e425e615bc05da790987380a84b8280"
test "$holochain_ref" = "holochain-0.6.0"
test "$holochain_rev" = "a6d4e805a0971ccbc0dcb3f3ed6a9e2fac980a3b"

cargo_lock="$FINANCE_ROOT/Cargo.lock"
test -f "$cargo_lock"
cargo_package_json="$OUT_DIR/cargo-package-versions.json"
jq -n \
  --arg hdk "$(awk '/^name = "hdk"$/{f=1} f && /^version = /{gsub(/"/,"",$3); print $3; exit}' "$cargo_lock")" \
  --arg hdi "$(awk '/^name = "hdi"$/{f=1} f && /^version = /{gsub(/"/,"",$3); print $3; exit}' "$cargo_lock")" \
  --arg holochain_zome_types "$(awk '/^name = "holochain_zome_types"$/{f=1} f && /^version = /{gsub(/"/,"",$3); print $3; exit}' "$cargo_lock")" \
  --arg holochain_integrity_types "$(awk '/^name = "holochain_integrity_types"$/{f=1} f && /^version = /{gsub(/"/,"",$3); print $3; exit}' "$cargo_lock")" \
  '{hdk:$hdk,hdi:$hdi,holochain_zome_types:$holochain_zome_types,holochain_integrity_types:$holochain_integrity_types}' > "$cargo_package_json"

runtime_tsv="$OUT_DIR/runtime-binaries.tsv"
nix develop .#ci --no-write-lock-file --command bash -c '
  set -euo pipefail
  for name in holochain hc; do
    path="$(command -v "$name")"
    resolved="$(readlink -f "$path")"
    version="$("$name" --version 2>&1 | tr "\\n" " ")"
    digest="$(sha256sum "$resolved" | cut -d" " -f1)"
    printf "%s\\t%s\\t%s\\t%s\\t%s\\n" "$name" "$path" "$resolved" "$version" "$digest"
  done
' > "$runtime_tsv"

runtime_binaries="$OUT_DIR/runtime-binaries.json"
jq -Rn '
  [inputs | split("\t") | {
    name:.[0],
    path:.[1],
    resolved_path:.[2],
    version_output:.[3],
    sha256:.[4]
  }] |
  {
    holochain:(map(select(.name == "holochain")) | first),
    hc:(map(select(.name == "hc")) | first)
  }
' "$runtime_tsv" > "$runtime_binaries"

dna_path="$FINANCE_ROOT/dna/dna.yaml"
flake_lock_path="$FINANCE_ROOT/flake.lock"
flake_path="$FINANCE_ROOT/flake.nix"
cargo_manifest_path="$FINANCE_ROOT/Cargo.toml"
dna_sha="$(sha256sum "$dna_path" | cut -d' ' -f1)"
flake_lock_sha="$(sha256sum "$flake_lock_path" | cut -d' ' -f1)"
flake_sha="$(sha256sum "$flake_path" | cut -d' ' -f1)"
cargo_lock_sha="$(sha256sum "$cargo_lock" | cut -d' ' -f1)"
cargo_manifest_sha="$(sha256sum "$cargo_manifest_path" | cut -d' ' -f1)"

jq -n \
  --arg subject_commit "$subject_commit" \
  --arg captured_at "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
  --arg nix_version "$nix_version" \
  --arg holonix_rev "$holonix_rev" \
  --arg holochain_ref "$holochain_ref" \
  --arg holochain_rev "$holochain_rev" \
  --arg dna_sha256 "$dna_sha" \
  --arg flake_lock_sha256 "$flake_lock_sha" \
  --arg flake_sha256 "$flake_sha" \
  --arg cargo_lock_sha256 "$cargo_lock_sha" \
  --arg cargo_manifest_sha256 "$cargo_manifest_sha" \
  --slurpfile packages "$cargo_package_json" \
  --slurpfile binaries "$runtime_binaries" \
  '{
    schema_version:"HolochainRuntimeEvidenceV1",
    status:"runtime_binary_observed_capability_behavior_not_qualified",
    subject_commit:$subject_commit,
    captured_at_utc:$captured_at,
    nix_version:$nix_version,
    locked_inputs:{
      holonix_commit:$holonix_rev,
      holochain_source_ref:$holochain_ref,
      holochain_source_commit:$holochain_rev
    },
    cargo_packages:$packages[0],
    runtime_binaries:$binaries[0],
    source_sha256:{
      dna_manifest:$dna_sha256,
      flake_lock:$flake_lock_sha256,
      flake:$flake_sha256,
      cargo_lock:$cargo_lock_sha256,
      cargo_manifest:$cargo_manifest_sha256
    },
    behavior_not_tested:{
      countersigning_enablement:true,
      multi_agent_countersigning_session:true,
      fork_warrant_publication:true,
      delayed_fork_discovery:true,
      chain_head_move_atomic_rollback:true
    },
    claim_ceiling:"This artifact proves which locked Flake binaries were executed and their digests only. It does not establish countersigning enablement, fork-warrant behavior, multi-agent safety, or settlement finality."
  }' > "$OUT_DIR/holochain-runtime-evidence.json"

cat "$OUT_DIR/holochain-runtime-evidence.json"
echo "Runtime evidence written to: $OUT_DIR/holochain-runtime-evidence.json"
