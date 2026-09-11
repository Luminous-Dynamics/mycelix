#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
trust="$root/packages/accounting/src/interop-line-proof-attestation-trust.ts"
test_file="$root/packages/accounting/src/interop-line-proof-attestation-trust.test.ts"
index="$root/packages/accounting/src/index.ts"

require() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if ! grep -Fq -- "$pattern" "$file"; then
    echo "reporting evidence issuer trust invariant failed: $message" >&2
    exit 1
  fi
}

forbid() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if grep -Fq -- "$pattern" "$file"; then
    echo "reporting evidence issuer trust invariant failed: $message" >&2
    exit 1
  fi
}

require "$trust" "reporting_evidence_package_issuer_trust_bundle_v1" "trust bundle must have a domain-separated canonical commitment"
require "$trust" "signedMs >= Date.parse(entry.validUntil)" "trust windows must remain half-open"
require "$trust" "signedMs >= Date.parse(entry.revokedAt)" "revocation must remain prospective at revokedAt"
require "$trust" "policy must be observed through attestation signedAt" "trust policy must cover the original signature instant"
require "$trust" "policy cannot be newer than verification time" "future trust policy must not leak backward into verification"
require "$trust" "entry.capabilityRoot === capability.capabilityRoot" "trust selection must pin the exact capability root"
require "$trust" "contains overlapping authority windows" "overlapping windows for one exact authority identity must fail closed"
require "$trust" "reporting_evidence_package_issuer_verification_receipt_v1" "verified issuer trust must emit a rooted receipt"
require "$trust" "VERIFIED_REPORTING_EVIDENCE_ISSUER_AUTHORITIES" "verified issuer trust must carry runtime provenance"
require "$trust" "requireVerifiedReportingEvidencePackageAuthority(attestationAuthority)" "trust verification must delegate cryptographic verification to the detached-attestation layer"
require "$index" "export * from './interop-line-proof-attestation-trust.js';" "issuer trust API must be exported"

forbid "$trust" "privateKey" "issuer trust verification must not custody signing private keys"
forbid "$trust" "sign(" "issuer trust verification must not sign evidence packages"

require "$test_file" "keeps a pre-retirement signature verifiable after rotation" "historical validity regression must remain"
require "$test_file" "authorizes the rotated signer in its own window" "rotated-key regression must remain"
require "$test_file" "uses half-open retirement semantics at validUntil" "exact retirement boundary regression must remain"
require "$test_file" "treats revocation as prospective and rejects signatures at revokedAt" "revocation boundary regression must remain"
require "$test_file" "rejects a stale policy snapshot predating the signature" "stale-policy regression must remain"
require "$test_file" "rejects overlapping windows for the same exact authority identity" "overlap regression must remain"
require "$test_file" "does not let structural clones regain issuer trust authority" "runtime provenance regression must remain"

echo "reporting evidence issuer trust source invariants: PASS"
