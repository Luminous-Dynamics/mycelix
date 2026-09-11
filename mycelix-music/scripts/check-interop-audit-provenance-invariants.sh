#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "interop audit provenance invariant failed: $1" >&2
  exit 1
}

interop=packages/accounting/src/interop.ts
tests=packages/accounting/src/interop-attested-audit.test.ts
policy=packages/accounting/promotion-policy.json
workflow=../.github/workflows/music-accounting.yml

[[ -f "$interop" ]] || fail "interop source is missing"
[[ -f "$tests" ]] || fail "attested interop regressions are missing"

grep -Fq 'requireAnchoredEconomicAuditCapsuleIssuerAuthority' "$interop" \
  || fail "attested reporting must require sealed root-anchored audit issuer authority"
grep -Fq 'attested reporting issuer authority is not bound to the supplied audit capsule' "$interop" \
  || fail "attested reporting must bind issuer authority to exact capsule digest"
grep -Fq 'statementId: input.capsule.statement.statementId' "$interop" \
  || fail "attested reporting source statement id must derive from audit capsule"
grep -Fq 'statementCommitmentRoot: input.capsule.statementSnapshotRoot' "$interop" \
  || fail "attested reporting source root must derive from audit capsule statement snapshot"
grep -Fq "provenanceKind: 'root_attested_economic_audit_capsule_v1'" "$interop" \
  || fail "attested reporting provenance kind missing"
grep -Fq "lineEvidenceScope: 'statement_level_reference_only'" "$interop" \
  || fail "interop must state that audit provenance is statement-level rather than line-membership proof"
grep -Fq 'attested reporting line beneficiary does not match audited statement beneficiary' "$interop" \
  || fail "attested reporting lines must remain inside statement beneficiary boundary"
grep -Fq 'attested reporting line currency does not match audited statement currency' "$interop" \
  || fail "attested reporting lines must remain inside statement currency boundary"
grep -Fq "recordType: 'interop_root_attested_audit_reference_v1'" "$interop" \
  || fail "attested audit reference must be domain-separated"
grep -Fq "recordType: 'ddex_royalty_reporting_projection_attested_audit_v1'" "$interop" \
  || fail "attested DDEX projection root must be domain-separated"
grep -Fq "recordType: 'cisac_crd_reporting_projection_attested_audit_v1'" "$interop" \
  || fail "attested CRD projection root must be domain-separated"
grep -Fq "authority: 'reporting_projection_only'" "$interop" \
  || fail "interop projections must remain explicitly non-authoritative"
grep -Fq 'attested reporting cannot predate anchored issuer verification' "$interop" \
  || fail "report generation must not predate the trust evidence it cites"
grep -Fq 'projectionRoot` is a deterministic content commitment, not' "$interop" \
  || fail "interop source must explicitly distinguish projection root from issuer signature"

grep -Fq 'derives DDEX statement provenance from the sealed attested audit chain' "$tests" \
  || fail "regressions must prove DDEX provenance derivation"
grep -Fq 'derives CRD provenance from the same attested statement without granting ledger authority' "$tests" \
  || fail "regressions must prove CRD remains non-authoritative"
grep -Fq 'rejects structural clones of the anchored issuer authority' "$tests" \
  || fail "regressions must reject fabricated anchored issuer authority"
grep -Fq 'rejects reports generated before anchored issuer verification' "$tests" \
  || fail "regressions must reject temporal provenance leakage"
grep -Fq 'rejects lines that cross the audited beneficiary or currency boundary' "$tests" \
  || fail "regressions must reject cross-statement beneficiary/currency misuse"
grep -Fq 'keeps projection content commitments sensitive to reporting profile and line content' "$tests" \
  || fail "regressions must bind profile and content into projection root"

grep -Fq "'mycelix-music/scripts/check-interop-audit-provenance-invariants.sh'" "$workflow" \
  || fail "accounting workflow must trigger when interop provenance invariants change"
grep -Fq 'check-interop-audit-provenance-invariants.sh' ../mycelix-music/package.json \
  || fail "root invariant command must execute interop audit provenance invariants"
grep -Fq 'A root-attested audit capsule reference turns a DDEX or CISAC CRD projection into accounting ledger authority.' "$policy" \
  || fail "promotion policy must keep reporting projections non-authoritative"
grep -Fq 'A statement-level attested audit reference proves per-line obligation membership without inclusion proofs.' "$policy" \
  || fail "promotion policy must forbid line-membership overclaim"
grep -Fq 'An interop projectionRoot is an issuer signature.' "$policy" \
  || fail "promotion policy must distinguish deterministic content root from signature"
grep -Fq 'Caller-supplied reporting source provenance is equivalent to provenance derived from a sealed anchored audit issuer authority.' "$policy" \
  || fail "promotion policy must distinguish derived attested provenance from caller metadata"

echo "interop audit provenance invariants passed"
