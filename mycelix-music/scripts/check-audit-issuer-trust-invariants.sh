#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "audit issuer trust invariant failed: $1" >&2
  exit 1
}

trust=packages/accounting/src/audit-capsule-attestation-trust.ts
tests=packages/accounting/src/audit-capsule-attestation-trust.test.ts
index=packages/accounting/src/index.ts
policy=packages/accounting/promotion-policy.json
workflow=../.github/workflows/music-accounting.yml

[[ -f "$trust" ]] || fail "audit issuer trust source is missing"
[[ -f "$tests" ]] || fail "audit issuer trust regressions are missing"

grep -Fq "recordType: 'economic_audit_capsule_issuer_trust_bundle_v1'" "$trust" \
  || fail "audit issuer trust bundle must have a domain-separated commitment"
grep -Fq "recordType: 'economic_audit_capsule_issuer_verification_receipt_v1'" "$trust" \
  || fail "audit issuer verification receipt must have a domain-separated commitment"
grep -Fq 'audit issuer trust policy must be observed through attestation signedAt' "$trust" \
  || fail "verification must reject trust policy snapshots stale at signedAt"
grep -Fq 'signedMs >= Date.parse(entry.validUntil)' "$trust" \
  || fail "audit issuer trust validity must use a half-open end boundary"
grep -Fq 'signedMs >= Date.parse(entry.revokedAt)' "$trust" \
  || fail "audit issuer revocation must block signatures at and after the cutoff"
grep -Fq 'audit issuer trust bundle contains overlapping authority windows' "$trust" \
  || fail "same audit issuer authority cannot have overlapping trust windows"
grep -Fq 'trustBundleRoot: bundle.bundleRoot' "$trust" \
  || fail "issuer verification receipt must bind exact trust bundle root"
grep -Fq 'trustEntryId: entry.entryId' "$trust" \
  || fail "issuer verification receipt must identify exact trust entry"
grep -Fq 'const VERIFIED_AUDIT_ISSUER_AUTHORITIES = new WeakSet<object>();' "$trust" \
  || fail "verified issuer authorities must carry private runtime provenance"
grep -Fq 'requireVerifiedEconomicAuditCapsuleIssuerAuthority' "$trust" \
  || fail "verified issuer authority promotion accessor is missing"
grep -Fq "export * from './audit-capsule-attestation-trust.js';" "$index" \
  || fail "package index must expose audit issuer trust rotation API"

grep -Fq 'preserves historical old-key verification after rotation and emits an auditable receipt' "$tests" \
  || fail "regressions must prove historical issuer signatures survive rotation"
grep -Fq 'accepts the rotated audit issuer key after its trust window begins' "$tests" \
  || fail "regressions must prove rotated issuer key acceptance"
grep -Fq 'rejects an old issuer signature at and after the trust window expires' "$tests" \
  || fail "regressions must reject retired issuer signatures"
grep -Fq 'treats audit issuer revocation prospectively: pre-cutoff signatures survive, post-cutoff signatures fail' "$tests" \
  || fail "regressions must prove prospective audit issuer revocation"
grep -Fq 'rejects stale audit issuer trust policy snapshots predating the attestation signature' "$tests" \
  || fail "regressions must reject stale issuer trust policy"
grep -Fq 'rejects audit issuer trust-bundle tampering behind an unchanged root' "$tests" \
  || fail "regressions must reject issuer trust-bundle mutation"
grep -Fq 'rejects overlapping authority windows for the same audit issuer capability identity' "$tests" \
  || fail "regressions must reject ambiguous issuer trust windows"
grep -Fq 'rejects structural clones of verified audit issuer authority' "$tests" \
  || fail "regressions must reject fabricated verified issuer authority"

grep -Fq "'mycelix-music/scripts/check-audit-issuer-trust-invariants.sh'" "$workflow" \
  || fail "accounting workflow must trigger when audit issuer trust invariants change"
grep -Fq 'check-audit-issuer-trust-invariants.sh' ../mycelix-music/package.json \
  || fail "root invariant command must execute audit issuer trust invariants"
grep -Fq 'A stale audit-issuer trust bundle predating the capsule attestation signature is sufficient revocation evidence.' "$policy" \
  || fail "promotion policy must forbid stale issuer trust evidence"
grep -Fq 'Audit issuer key rotation invalidates all historically valid capsule attestations by default.' "$policy" \
  || fail "promotion policy must preserve historical issuer signatures"
grep -Fq 'A revoked audit issuer trust entry can authorize signatures at or after revokedAt.' "$policy" \
  || fail "promotion policy must enforce prospective issuer revocation"

echo "audit issuer trust invariants passed"
