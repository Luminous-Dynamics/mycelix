#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "audit capsule invariant failed: $1" >&2
  exit 1
}

audit=packages/accounting/src/audit.ts
audit_tests=packages/accounting/src/audit.test.ts
attestation=packages/accounting/src/audit-capsule-attestation.ts
attestation_tests=packages/accounting/src/audit-capsule-attestation.test.ts
index=packages/accounting/src/index.ts
policy=packages/accounting/promotion-policy.json
workflow=../.github/workflows/music-accounting.yml

[[ -f "$audit" ]] || fail "audit capsule source is missing"
[[ -f "$audit_tests" ]] || fail "audit capsule regressions are missing"
[[ -f "$attestation" ]] || fail "audit capsule attestation source is missing"
[[ -f "$attestation_tests" ]] || fail "audit capsule attestation regressions are missing"

grep -Fq 'export interface EconomicAuditCapsuleV1' "$audit" \
  || fail "historical v1 audit capsules must remain explicitly verifiable"
grep -Fq 'export interface EconomicAuditCapsuleV2' "$audit" \
  || fail "audit capsule v2 type is missing"
grep -Fq 'readonly protocolVersion: 2;' "$audit" \
  || fail "new audit capsule protocol must be v2"
grep -Fq 'readonly statement: RoyaltyStatementSnapshot;' "$audit" \
  || fail "v2 capsule must embed the complete immutable statement snapshot"
grep -Fq 'readonly statementInput: CompileRoyaltyStatementInput;' "$audit" \
  || fail "v2 creation must consume exact statement compiler input"
grep -Fq 'const statement = compileRoyaltyStatement(input.statementInput);' "$audit" \
  || fail "v2 creation must compile the exact statement input itself"
grep -Fq "recordType: 'economic_audit_royalty_statement_snapshot_v1'" "$audit" \
  || fail "embedded statement snapshot must have a domain-separated commitment"
grep -Fq 'statementSnapshotRoot: statementSnapshotRoot(statement)' "$audit" \
  || fail "v2 capsule must commit the complete embedded statement snapshot"
grep -Fq 'statementSnapshotRoot does not match embedded immutable statement' "$audit" \
  || fail "serialized v2 validation must recompute statement snapshot identity"
grep -Fq 'assertStatementArithmetic(capsule.statement)' "$audit" \
  || fail "serialized v2 validation must re-check statement arithmetic"
grep -Fq 'requireAnchoredSettlementAllocationLineageCheckpointAuthority' "$audit" \
  || fail "audit trust evidence must come from resolver-minted anchored authority"
grep -Fq ".filter(settlement => settlement.anchoredTrust !== undefined)" "$audit" \
  || fail "audit trust set must derive from the exact statement settlement inputs"
grep -Fq "recordType: 'economic_audit_anchored_settlement_trust_set_v1'" "$audit" \
  || fail "anchored settlement trust set must have a domain-separated commitment"
grep -Fq 'anchoredSettlementTrustRoot: trustSetRoot(anchoredSettlementTrust)' "$audit" \
  || fail "v2 capsule must commit the complete derived trust set"
grep -Fq 'residualTotal: Object.freeze({ ...subtractMoney(statement.netPayable, statement.paid) })' "$audit" \
  || fail "v2 conservation residual must derive from statement unpaid payable value"
grep -Fq 'audit capsule conservation proof does not match embedded statement economics' "$audit" \
  || fail "serialized v2 validation must recompute conservation from statement economics"
grep -Fq 'must be a lowercase SHA-256 digest' "$audit" \
  || fail "v2 externally supplied roots must require canonical SHA-256 form"
grep -Fq 'trust policy sequence must be canonical positive-integer text' "$audit" \
  || fail "v2 trust policy sequence must remain canonical"
grep -Fq 'verifiedAt must equal statementAsOf' "$audit" \
  || fail "v2 trust verification must remain pinned to the statement instant"

v2_input=$(sed -n '/^export interface CreateEconomicAuditCapsuleV2Input {/,/^}/p' "$audit")
[[ -n "$v2_input" ]] || fail "v2 capsule input type missing"
if grep -Fq 'conservationProof' <<<"$v2_input"; then
  fail "v2 creation must not accept caller-supplied conservation proof"
fi
if grep -Fq 'anchoredSettlementTrust' <<<"$v2_input"; then
  fail "v2 creation must not accept a caller-supplied parallel trust list"
fi

grep -Fq 'compiles the exact statement input and derives a complete per-batch anchored-trust set' "$audit_tests" \
  || fail "regressions must prove trust-set derivation from exact statement input"
grep -Fq 'is reproducible for identical authoritative statement and trust inputs' "$audit_tests" \
  || fail "regressions must prove v2 reproducibility"
grep -Fq 'changes identity when root trust policy evidence changes' "$audit_tests" \
  || fail "regressions must prove root-trust sensitivity"
grep -Fq 'detects omission or mutation inside the committed anchored-trust set' "$audit_tests" \
  || fail "regressions must attack trust-set omission and mutation"
grep -Fq 'binds the full statement snapshot and recomputes conservation from embedded economics' "$audit_tests" \
  || fail "regressions must attack statement/conservation drift"
grep -Fq 'requires v2 external roots and compiler build identity to be canonical SHA-256 digests' "$audit_tests" \
  || fail "regressions must reject malformed v2 roots"
grep -Fq 'keeps historical v1 capsules verifiable without allowing v1 creation through the v2 constructor' "$audit_tests" \
  || fail "regressions must preserve historical v1 verification"

# Capsule issuer authority is detached from accounting/compiler key custody and
# must bind the exact v2 capsule digest, statement snapshot and compiler build.
grep -Fq 'ECONOMIC_AUDIT_CAPSULE_ATTESTATION_SCOPE' "$attestation" \
  || fail "audit capsule attestation scope is missing"
grep -Fq "recordType: 'economic_audit_capsule_attestation_capability_v1'" "$attestation" \
  || fail "audit issuer capability must have a domain-separated root"
grep -Fq "recordType: 'economic_audit_capsule_attestation_request_v1'" "$attestation" \
  || fail "audit attestation request must have a domain-separated root"
grep -Fq "recordType: 'economic_audit_capsule_attestation_v1'" "$attestation" \
  || fail "audit attestation must have a domain-separated root"
grep -Fq "recordType: 'economic_audit_capsule_attestation_verification_receipt_v1'" "$attestation" \
  || fail "audit attestation verification receipt must be domain-separated"
grep -Fq 'compilerBuildDigest: digest' "$attestation" \
  || fail "audit issuer capability must bind exact compiler build digest"
grep -Fq 'capabilityRoot: capability.capabilityRoot' "$attestation" \
  || fail "attestation request must bind exact capability root"
grep -Fq 'capsuleDigest: economicAuditCapsuleDigest(capsule)' "$attestation" \
  || fail "attestation request must bind exact audit capsule digest"
grep -Fq 'statementSnapshotRoot: capsule.statementSnapshotRoot' "$attestation" \
  || fail "attestation request must bind exact statement snapshot root"
grep -Fq 'audit capsule attestation request is outside exact capability authority' "$attestation" \
  || fail "attestation request must fail outside exact capability authority"
grep -Fq 'audit capsule attestation request must remain inside capability validity window' "$attestation" \
  || fail "attestation request must remain inside capability validity window"
grep -Fq 'Date.parse(signedAt) >= Date.parse(canonical.expiresAt)' "$attestation" \
  || fail "audit attestation request signing window must be half-open"
grep -Fq 'audit capsule attestation is not authorized by pinned issuer trust policy' "$attestation" \
  || fail "verification must pin exact issuer/signer/capability root"
grep -Fq 'const VERIFIED_AUDIT_CAPSULE_ATTESTATIONS = new WeakSet<object>();' "$attestation" \
  || fail "verified audit attestations must carry private runtime provenance"
grep -Fq 'requireVerifiedEconomicAuditCapsuleAttestation' "$attestation" \
  || fail "verified audit attestation promotion accessor is missing"
if grep -Eq 'privateKeyPem|createPrivateKey|signEd25519' "$attestation"; then
  fail "audit attestation module must not contain private-key custody or signing calls"
fi

grep -Fq "export * from './audit-capsule-attestation.js';" "$index" \
  || fail "package index must expose detached audit capsule attestation API"

grep -Fq 'verifies a capability-scoped detached signature and emits a root-bearing receipt' "$attestation_tests" \
  || fail "regressions must prove detached issuer verification"
grep -Fq 'rejects a capability that does not authorize the exact compiler build' "$attestation_tests" \
  || fail "regressions must attack compiler-build scope"
grep -Fq 'rejects capability content changed behind an unchanged capability root' "$attestation_tests" \
  || fail "regressions must attack capability mutation"
grep -Fq 'rejects rebinding a request or signature to another capsule' "$attestation_tests" \
  || fail "regressions must attack capsule/signature rebinding"
grep -Fq 'uses a half-open request signing window' "$attestation_tests" \
  || fail "regressions must prove attestation expiry boundary"
grep -Fq 'rejects signatures from an untrusted key' "$attestation_tests" \
  || fail "regressions must reject wrong issuer key"
grep -Fq 'requires issuer trust to pin the exact capability root' "$attestation_tests" \
  || fail "regressions must prove exact capability-root pinning"
grep -Fq 'rejects structural clones of verified attestation authority' "$attestation_tests" \
  || fail "regressions must reject fabricated verified attestation authority"

grep -Fq "'mycelix-music/scripts/check-audit-capsule-invariants.sh'" "$workflow" \
  || fail "accounting workflow must trigger when audit capsule invariants change"
grep -Fq 'check-audit-capsule-invariants.sh' ../mycelix-music/package.json \
  || fail "root invariant command must execute audit capsule invariants"
grep -Fq 'A caller-supplied trust list is equivalent to trust evidence derived from the exact statement compiler input.' "$policy" \
  || fail "promotion policy must forbid caller-supplied parallel trust authority"
grep -Fq 'A caller-supplied conservation proof is authoritative for an EconomicAuditCapsule v2.' "$policy" \
  || fail "promotion policy must require statement-derived v2 conservation"
grep -Fq 'A structurally valid serialized audit capsule proves who produced or anchored it.' "$policy" \
  || fail "promotion policy must distinguish committed evidence from provenance/authentication"
grep -Fq 'An EconomicAuditCapsule digest without a verified detached issuer attestation proves issuer identity.' "$policy" \
  || fail "promotion policy must require issuer attestation for attribution"
grep -Fq 'Accounting, database or statement compiler access implies possession of audit-capsule issuer signing keys.' "$policy" \
  || fail "promotion policy must separate audit issuer key custody"
grep -Fq 'Pinning an audit attestation capabilityId is equivalent to pinning the exact capability root.' "$policy" \
  || fail "promotion policy must require exact capability-root pinning"

echo "audit capsule invariants passed"
