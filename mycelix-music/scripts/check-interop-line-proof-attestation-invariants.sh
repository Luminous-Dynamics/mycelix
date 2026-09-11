#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "interop line-proof attestation invariant failed: $1" >&2
  exit 1
}

source_file=packages/accounting/src/interop-line-proof-attestation.ts
tests=packages/accounting/src/interop-line-proof-attestation.test.ts
policy=packages/accounting/promotion-policy.json
workflow=../.github/workflows/music-accounting.yml

[[ -f "$source_file" ]] || fail "reporting evidence attestation source is missing"
[[ -f "$tests" ]] || fail "reporting evidence attestation regressions are missing"

grep -Fq "REPORTING_EVIDENCE_PACKAGE_ATTESTATION_SCOPE" "$source_file" \
  || fail "attestation capability scope is missing"
grep -Fq "recordType: 'reporting_evidence_package_attestation_capability_v1'" "$source_file" \
  || fail "capability commitment must be domain separated"
grep -Fq "recordType: 'reporting_evidence_package_attestation_request_v1'" "$source_file" \
  || fail "request commitment must be domain separated"
grep -Fq "recordType: 'reporting_evidence_package_attestation_v1'" "$source_file" \
  || fail "attestation commitment must be domain separated"
grep -Fq "recordType: 'reporting_evidence_package_attestation_receipt_v1'" "$source_file" \
  || fail "verification receipt must be domain separated"
grep -Fq 'projection root does not match canonical projection contents' "$source_file" \
  || fail "attestation must recompute projection contents before signing"
grep -Fq 'reporting evidence package root does not match canonical contents' "$source_file" \
  || fail "attestation must recompute package root"
grep -Fq 'proofByObligationId' "$source_file" \
  || fail "projection lines must be matched to proofs by obligation id"
grep -Fq 'reporting evidence trust policy does not pin exact capability root' "$source_file" \
  || fail "trust policy must pin exact capability root"
grep -Fq 'VERIFIED_REPORTING_EVIDENCE_AUTHORITIES = new WeakSet' "$source_file" \
  || fail "verified package authorities require runtime provenance seal"

if grep -Eq 'privateKeyPem|createPrivateKey|sign as signEd25519|signEd25519\(' "$source_file"; then
  fail "accounting reporting attestation verifier must not custody or use private keys"
fi

grep -Fq 'verifies an externally signed package and emits a sealed receipt' "$tests" \
  || fail "regressions must exercise detached external signing"
grep -Fq 'accepts a package built from unsorted caller lines because proof matching is obligation-id based' "$tests" \
  || fail "regressions must exercise obligation-id proof matching"
grep -Fq 'rejects projection content tampering behind an unchanged projection root' "$tests" \
  || fail "regressions must reject projection/root mismatch"
grep -Fq 'rejects capabilities that do not authorize the exact format/profile pair' "$tests" \
  || fail "regressions must enforce format/profile capability scope"
grep -Fq 'uses a half-open signature request window' "$tests" \
  || fail "regressions must enforce half-open request validity"
grep -Fq 'rejects a detached signature produced by the wrong Ed25519 key' "$tests" \
  || fail "regressions must reject wrong signer key"
grep -Fq 'requires trust policy to pin the exact capability root' "$tests" \
  || fail "regressions must pin capability root"
grep -Fq 'does not let structural clones regain verified package authority' "$tests" \
  || fail "regressions must reject structural authority clones"

grep -Fq "'mycelix-music/scripts/check-interop-line-proof-attestation-invariants.sh'" "$workflow" \
  || fail "workflow must trigger when attestation invariants change"
grep -Fq 'check-interop-line-proof-attestation-invariants.sh' ../mycelix-music/package.json \
  || fail "root invariant command must execute attestation gate"
grep -Fq 'Possession of a reporting evidence package or compiler access implies possession of the reporting evidence attestation private key.' "$policy" \
  || fail "promotion policy must preserve signing-key custody separation"
grep -Fq 'Pinning a reporting evidence attestation capabilityId is equivalent to pinning the exact capability root.' "$policy" \
  || fail "promotion policy must require exact capability-root pinning"
grep -Fq 'A detached reporting evidence package attestation turns DDEX or CISAC CRD into accounting ledger authority.' "$policy" \
  || fail "promotion policy must preserve reporting-only authority after attestation"

echo "interop line-proof attestation invariants passed"
