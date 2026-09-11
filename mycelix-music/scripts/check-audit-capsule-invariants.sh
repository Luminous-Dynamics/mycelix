#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "audit capsule invariant failed: $1" >&2
  exit 1
}

audit=packages/accounting/src/audit.ts
tests=packages/accounting/src/audit.test.ts
policy=packages/accounting/promotion-policy.json
workflow=../.github/workflows/music-accounting.yml

[[ -f "$audit" ]] || fail "audit capsule source is missing"
[[ -f "$tests" ]] || fail "audit capsule regressions are missing"

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

grep -Fq 'compiles the exact statement input and derives a complete per-batch anchored-trust set' "$tests" \
  || fail "regressions must prove trust-set derivation from exact statement input"
grep -Fq 'is reproducible for identical authoritative statement and trust inputs' "$tests" \
  || fail "regressions must prove v2 reproducibility"
grep -Fq 'changes identity when root trust policy evidence changes' "$tests" \
  || fail "regressions must prove root-trust sensitivity"
grep -Fq 'detects omission or mutation inside the committed anchored-trust set' "$tests" \
  || fail "regressions must attack trust-set omission and mutation"
grep -Fq 'binds the full statement snapshot and recomputes conservation from embedded economics' "$tests" \
  || fail "regressions must attack statement/conservation drift"
grep -Fq 'requires v2 external roots and compiler build identity to be canonical SHA-256 digests' "$tests" \
  || fail "regressions must reject malformed v2 roots"
grep -Fq 'keeps historical v1 capsules verifiable without allowing v1 creation through the v2 constructor' "$tests" \
  || fail "regressions must preserve historical v1 verification"

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

echo "audit capsule invariants passed"
