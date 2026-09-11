#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "interop line proof invariant failed: $1" >&2
  exit 1
}

source_file=packages/accounting/src/interop-line-proofs.ts
tests=packages/accounting/src/interop-line-proofs.test.ts
policy=packages/accounting/promotion-policy.json
workflow=../.github/workflows/music-accounting.yml

[[ -f "$source_file" ]] || fail "reporting-safe line proof source is missing"
[[ -f "$tests" ]] || fail "reporting-safe line proof regressions are missing"

grep -Fq 'reporting-safe disclosure obligations do not reconstruct audited internal obligation root' "$source_file" \
  || fail "sidecar must reconstruct the audited internal obligation root before public disclosure"
grep -Fq "authority: 'compiler_derived_sidecar_not_issuer_attested'" "$source_file" \
  || fail "sidecar must explicitly deny issuer-attested authority"
grep -Fq "proofKind: 'reporting_safe_merkle_inclusion_not_zero_knowledge'" "$source_file" \
  || fail "line proof must explicitly state it is ordinary Merkle inclusion, not zero knowledge"
grep -Fq "fieldEvidenceScope: 'obligation_id_authority_root_beneficiary_amount_currency_only'" "$source_file" \
  || fail "public line proof field scope must remain narrow and explicit"
grep -Fq "recordType: 'reporting_safe_obligation_v1'" "$source_file" \
  || fail "reporting-safe leaves must be domain separated"
grep -Fq "recordType: 'reporting_safe_disclosure_sidecar_v1'" "$source_file" \
  || fail "reporting-safe sidecar commitment must be domain separated"
grep -Fq "recordType: 'attested_reporting_evidence_package_v1'" "$source_file" \
  || fail "reporting evidence package must be domain separated"
grep -Fq "evidenceStatus: 'compiler_derived_line_evidence_not_issuer_attested'" "$source_file" \
  || fail "line evidence package must not claim issuer attestation"
grep -Fq "authority: 'reporting_projection_only'" "$source_file" \
  || fail "proof-bearing interoperability package must remain non-authoritative"
grep -Fq 'reporting line amount does not match line evidence' "$source_file" \
  || fail "exported line amount must match proved obligation fields"

if grep -A12 -F "recordType: 'reporting_safe_obligation_v1'" "$source_file" | grep -Eq 'usageEvidenceRef|rightsResolutionRef|economicTermsRef'; then
  fail "reporting-safe public leaf must not reveal internal usage/rights/economic provenance references"
fi

grep -Fq 'proves a safe obligation leaf without disclosing internal provenance references' "$tests" \
  || fail "regressions must prove private provenance fields are not disclosed"
grep -Fq 'fails closed when the supplied obligation set omits audited debt' "$tests" \
  || fail "regressions must reject omitted debt before sidecar commitment"
grep -Fq 'rejects tampered public-safe Merkle leaves' "$tests" \
  || fail "regressions must reject proof leaf tampering"
grep -Fq 'packages DDEX projection with one safe proof per exported obligation line' "$tests" \
  || fail "regressions must exercise DDEX proof packaging"
grep -Fq 'packages CRD projection without turning line proofs into ledger authority' "$tests" \
  || fail "regressions must preserve CRD reporting-only authority"
grep -Fq 'rejects a reporting line amount that does not match its proved obligation' "$tests" \
  || fail "regressions must reject report/proof amount mismatch"

grep -Fq "'mycelix-music/scripts/check-interop-line-proof-invariants.sh'" "$workflow" \
  || fail "accounting workflow must trigger when line-proof invariants change"
grep -Fq 'check-interop-line-proof-invariants.sh' ../mycelix-music/package.json \
  || fail "root invariant command must execute line-proof invariants"
grep -Fq 'A reporting-safe Merkle inclusion proof is a zero-knowledge proof.' "$policy" \
  || fail "promotion policy must forbid ZK overclaim"
grep -Fq 'A compiler-derived reporting disclosure sidecar is issuer-attested evidence.' "$policy" \
  || fail "promotion policy must distinguish compiler-derived sidecar from issuer attestation"
grep -Fq 'A reporting-safe obligation proof authenticates workReference, usageReference, or territory fields.' "$policy" \
  || fail "promotion policy must limit line proof field scope"
grep -Fq 'A proof-bearing DDEX or CISAC CRD package becomes accounting ledger authority.' "$policy" \
  || fail "promotion policy must keep proof-bearing reports non-authoritative"

echo "interop line proof invariants passed"
