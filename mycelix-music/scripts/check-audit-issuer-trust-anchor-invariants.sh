#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "audit issuer trust-anchor invariant failed: $1" >&2
  exit 1
}

anchor=packages/accounting/src/audit-capsule-attestation-trust-anchor.ts
tests=packages/accounting/src/audit-capsule-attestation-trust-anchor.test.ts
index=packages/accounting/src/index.ts
policy=packages/accounting/promotion-policy.json
workflow=../.github/workflows/music-accounting.yml

[[ -f "$anchor" ]] || fail "audit issuer trust-anchor source is missing"
[[ -f "$tests" ]] || fail "audit issuer trust-anchor regressions are missing"

grep -Fq 'ECONOMIC_AUDIT_ISSUER_TRUST_BUNDLE_ATTESTATION_SCOPE' "$anchor" \
  || fail "audit issuer trust-bundle root-attestation scope missing"
grep -Fq "recordType: 'economic_audit_issuer_trust_bundle_attestation_request_v1'" "$anchor" \
  || fail "audit issuer trust-bundle attestation request must be domain-separated"
grep -Fq "recordType: 'economic_audit_issuer_trust_bundle_attestation_v1'" "$anchor" \
  || fail "audit issuer trust-bundle attestation must have a domain-separated root"
grep -Fq "recordType: 'economic_audit_capsule_anchored_issuer_verification_receipt_v1'" "$anchor" \
  || fail "anchored audit issuer verification receipt must be domain-separated"
grep -Fq 'audit issuer trust bundle policy sequence is below anti-rollback floor' "$anchor" \
  || fail "audit issuer trust anchor must enforce externally pinned rollback floor"
grep -Fq 'audit issuer trust attestation successor must advance policy sequence exactly once' "$anchor" \
  || fail "audit issuer trust-policy succession must be contiguous"
grep -Fq 'audit issuer trust attestation successor must bind predecessor bundle root' "$anchor" \
  || fail "audit issuer trust-policy succession must bind predecessor root"
grep -Fq 'audit issuer trust attestation cannot be newer than anchored verification time' "$anchor" \
  || fail "future root attestations must not leak backward into earlier verification views"
grep -Fq 'const VERIFIED_TRUST_ATTESTATIONS = new WeakSet<object>();' "$anchor" \
  || fail "verified audit issuer trust attestations must carry runtime provenance"
grep -Fq 'const VERIFIED_ANCHORED_ISSUER_AUTHORITIES = new WeakSet<object>();' "$anchor" \
  || fail "anchored audit issuer authorities must carry runtime provenance"
grep -Fq 'requireAnchoredEconomicAuditCapsuleIssuerAuthority' "$anchor" \
  || fail "anchored audit issuer authority promotion accessor is missing"
if grep -Eq 'privateKeyPem|createPrivateKey|signEd25519' "$anchor"; then
  fail "audit issuer trust-anchor module must not contain root private-key custody or signing calls"
fi

grep -Fq "export * from './audit-capsule-attestation-trust-anchor.js';" "$index" \
  || fail "package index must expose root-attested audit issuer trust verification"

grep -Fq 'rejects a cryptographically valid older issuer trust bundle below the pinned rollback floor' "$tests" \
  || fail "regressions must exercise audit issuer trust rollback"
grep -Fq 'requires exact predecessor continuity and one-step issuer trust policy advancement' "$tests" \
  || fail "regressions must prove contiguous audit issuer trust-policy succession"
grep -Fq 'rejects a successor that names the wrong predecessor issuer trust bundle root' "$tests" \
  || fail "regressions must reject predecessor substitution"
grep -Fq 'uses a half-open root-attestation request window' "$tests" \
  || fail "regressions must prove root-attestation expiry boundary"
grep -Fq 'rejects root signatures from a key not pinned by the audit issuer trust anchor' "$tests" \
  || fail "regressions must reject wrong root key"
grep -Fq 'verifies capsule issuer authority through the root-attested trust bundle and emits an anchored receipt' "$tests" \
  || fail "regressions must prove end-to-end anchored issuer verification"
grep -Fq 'rejects a root trust attestation newer than the requested anchored verification instant' "$tests" \
  || fail "regressions must reject future root-attestation leakage"
grep -Fq 'does not let structural clones participate in verified issuer trust succession or anchored authority' "$tests" \
  || fail "regressions must reject structural clones of root-trust authority"

grep -Fq "'mycelix-music/scripts/check-audit-issuer-trust-anchor-invariants.sh'" "$workflow" \
  || fail "accounting workflow must trigger when audit issuer trust-anchor invariants change"
grep -Fq 'check-audit-issuer-trust-anchor-invariants.sh' ../mycelix-music/package.json \
  || fail "root invariant command must execute audit issuer trust-anchor invariants"
grep -Fq 'An unattested audit-issuer trust bundle is sufficient root trust authority.' "$policy" \
  || fail "promotion policy must require root attestation of audit issuer trust bundles"
grep -Fq 'A cryptographically valid audit-issuer trust bundle below the pinned minimum policy sequence is acceptable.' "$policy" \
  || fail "promotion policy must forbid audit issuer trust rollback"
grep -Fq 'A root attestation signed after the requested audit verification instant can authorize that earlier view.' "$policy" \
  || fail "promotion policy must forbid future root-attestation leakage"
grep -Fq 'Accounting or audit verification code should custody audit issuer trust-anchor signing keys.' "$policy" \
  || fail "promotion policy must separate root signing-key custody"

echo "audit issuer trust-anchor invariants passed"
