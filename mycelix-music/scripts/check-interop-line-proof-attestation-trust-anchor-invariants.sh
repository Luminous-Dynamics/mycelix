#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
anchor="$root/packages/accounting/src/interop-line-proof-attestation-trust-anchor.ts"
test_file="$root/packages/accounting/src/interop-line-proof-attestation-trust-anchor.test.ts"
index="$root/packages/accounting/src/index.ts"

require() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if ! grep -Fq -- "$pattern" "$file"; then
    echo "reporting evidence trust-anchor invariant failed: $message" >&2
    exit 1
  fi
}

forbid() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if grep -Fq -- "$pattern" "$file"; then
    echo "reporting evidence trust-anchor invariant failed: $message" >&2
    exit 1
  fi
}

require "$anchor" "REPORTING_EVIDENCE_ISSUER_TRUST_BUNDLE_ATTESTATION_SCOPE" "root attestation must have an explicit capability scope"
require "$anchor" "reporting_evidence_issuer_trust_bundle_attestation_request_v1" "root request must be domain separated"
require "$anchor" "reporting_evidence_issuer_trust_bundle_attestation_v1" "root attestation must be domain separated"
require "$anchor" "minimumPolicySequence" "root policy must expose an anti-rollback floor"
require "$anchor" "below anti-rollback floor" "older-but-valid policy generations must fail below the pinned floor"
require "$anchor" "predecessorBundleRoot" "successor policies must bind the exact predecessor bundle root"
require "$anchor" "advance policy sequence exactly once" "policy succession must be contiguous"
require "$anchor" "cannot move policyAsOf backward" "policy succession must not move observation time backward"
require "$anchor" "successor must be signed strictly after predecessor" "policy succession must be temporally monotonic"
require "$anchor" "cannot authorize an earlier verification view" "future root attestations must not leak backward"
require "$anchor" "anchored_reporting_evidence_package_issuer_verification_receipt_v1" "anchored verification must emit a rooted receipt"
require "$anchor" "VERIFIED_TRUST_ATTESTATIONS" "root-attested trust bundles must carry runtime provenance"
require "$anchor" "VERIFIED_ANCHORED_ISSUER_AUTHORITIES" "anchored package issuer authorities must carry runtime provenance"
require "$index" "export * from './interop-line-proof-attestation-trust-anchor.js';" "root-anchor API must be exported"

forbid "$anchor" "privateKey" "root verification must not custody root private keys"
forbid "$anchor" "sign(" "root verification must not sign trust bundles"

require "$test_file" "verifies the complete package -> issuer trust -> root anchor chain" "end-to-end anchored verification regression must remain"
require "$test_file" "below the pinned anti-rollback floor" "rollback regression must remain"
require "$test_file" "uses a half-open root-signature request window" "root signing window regression must remain"
require "$test_file" "rejects a root signature from the wrong Ed25519 key" "wrong-root-key regression must remain"
require "$test_file" "rejects root attestations signed after the requested package verification instant" "future-attestation regression must remain"
require "$test_file" "requires exact predecessor continuity and one-step policy sequence advancement" "successor-lineage regression must remain"
require "$test_file" "does not let structural clones regain anchored package issuer authority" "runtime provenance regression must remain"

echo "reporting evidence trust-anchor source invariants: PASS"
