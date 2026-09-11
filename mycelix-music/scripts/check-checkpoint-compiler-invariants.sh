#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "checkpoint compiler invariant failed: $1" >&2
  exit 1
}

compiler=packages/accounting/src/settlement-allocation-lineage-checkpoint-compiler.ts
checkpoint=packages/accounting/src/settlement-allocation-lineage-checkpoint.ts
trust=packages/accounting/src/settlement-allocation-lineage-checkpoint-trust.ts
trust_anchor=packages/accounting/src/settlement-allocation-lineage-checkpoint-trust-anchor.ts
projection=packages/accounting/src/projection.ts
projection_tests=packages/accounting/src/projection-allocation-lineage.test.ts
tests=packages/accounting/src/settlement-allocation-lineage-checkpoint-compiler.test.ts
trust_tests=packages/accounting/src/settlement-allocation-lineage-checkpoint-trust.test.ts
trust_anchor_tests=packages/accounting/src/settlement-allocation-lineage-checkpoint-trust-anchor.test.ts
index=packages/accounting/src/index.ts
workflow=../.github/workflows/music-accounting.yml
policy=packages/accounting/promotion-policy.json

[[ -f "$compiler" ]] || fail "checkpoint compiler source is missing"
[[ -f "$checkpoint" ]] || fail "checkpoint verification source is missing"
[[ -f "$trust" ]] || fail "checkpoint signer trust source is missing"
[[ -f "$trust_anchor" ]] || fail "checkpoint trust-anchor source is missing"
[[ -f "$projection" ]] || fail "statement projection source is missing"
[[ -f "$projection_tests" ]] || fail "statement anchored-trust regressions are missing"
[[ -f "$tests" ]] || fail "checkpoint compiler regressions are missing"
[[ -f "$trust_tests" ]] || fail "checkpoint signer trust regressions are missing"
[[ -f "$trust_anchor_tests" ]] || fail "checkpoint trust-anchor regressions are missing"

grep -Fq 'verifyPersistedSettlementAllocation' "$compiler" \
  || fail "compiler must canonically replay persisted allocations before checkpoint creation"
grep -Fq 'verifyPersistedSettlementAllocationSuccessorLink' "$compiler" \
  || fail "compiler must canonically replay persisted successor links"
grep -Fq 'missing replay context for allocation' "$compiler" \
  || fail "compiler must fail closed when an allocation lacks semantic replay context"
grep -Fq 'successor link ingestion cursor must follow both endpoint allocations' "$compiler" \
  || fail "compiler must bind link ingestion after both endpoint allocations"
grep -Fq 'duplicate allocation-lineage ingestion cursor' "$compiler" \
  || fail "compiler must reject duplicate ingestion cursors"
grep -Fq 'must be canonical unsigned-integer text' "$compiler" \
  || fail "compiler must reject noncanonical cursor text"
grep -Fq 'ingestion cursor exceeds snapshot highWaterMark' "$compiler" \
  || fail "compiler must bound every source row by captured high-water mark"
grep -Fq 'lineage source allocation is later than snapshot asOf' "$compiler" \
  || fail "compiler must reject allocation evidence later than snapshot asOf"
grep -Fq 'lineage source successor link is later than snapshot asOf' "$compiler" \
  || fail "compiler must reject successor evidence later than snapshot asOf"
grep -Fq 'createSettlementAllocationLineageCheckpoint' "$compiler" \
  || fail "compiler must construct checkpoint only after canonical replay"
grep -Fq 'const COMPILED_CHECKPOINTS = new WeakSet<object>();' "$compiler" \
  || fail "compiled candidates must carry private runtime provenance"
grep -Fq 'createCompiledSettlementAllocationLineageCheckpointSigningRequest' "$compiler" \
  || fail "compiler must mint scoped detached-signing requests"
grep -Fq 'must be produced by canonical source compiler before requesting signature' "$compiler" \
  || fail "signing requests must reject caller-fabricated compiler candidates"

if grep -Eq 'privateKeyPem|createPrivateKey|signEd25519|signSettlementAllocationLineageCheckpoint' "$compiler"; then
  fail "checkpoint compiler must not contain private-key custody or signing calls"
fi
if grep -Eq 'PrismaClient|\$transaction|DATABASE_URL' "$compiler"; then
  fail "checkpoint compiler must remain independent of database connection custody"
fi

grep -Fq 'SETTLEMENT_ALLOCATION_LINEAGE_CHECKPOINT_SIGNING_SCOPE' "$checkpoint" \
  || fail "checkpoint signing scope is missing"
grep -Fq "recordType: 'settlement_allocation_lineage_checkpoint_signing_request_v1'" "$checkpoint" \
  || fail "checkpoint signing requests must be domain-separated commitments"
grep -Fq 'checkpoint signing capability does not authorize checkpoint source identity' "$checkpoint" \
  || fail "signing capability must bind exact source identity"
grep -Fq 'checkpoint signing request must remain inside capability validity window' "$checkpoint" \
  || fail "signing request must remain inside capability validity window"
grep -Fq 'checkpoint detached signature does not bind the canonical signing request' "$checkpoint" \
  || fail "detached signature must bind exact canonical request root"
grep -Fq 'settlementAllocationLineageCheckpointSigningPayloadBase64' "$checkpoint" \
  || fail "external signer payload surface is missing"
grep -Fq 'attachSettlementAllocationLineageCheckpointDetachedSignature' "$checkpoint" \
  || fail "detached signature verification/attachment surface is missing"
if grep -Fq "export * from './settlement-allocation-lineage-checkpoint.js';" "$index"; then
  fail "package index must not wildcard-export the deep-module inline PEM signer"
fi
if grep -Fq 'signSettlementAllocationLineageCheckpoint,' "$index"; then
  fail "package index must not export the inline PEM checkpoint signer"
fi
grep -Fq 'attachSettlementAllocationLineageCheckpointDetachedSignature' "$index" \
  || fail "package index must expose detached signature verification"
grep -Fq 'createSettlementAllocationLineageCheckpointSigningRequest' "$index" \
  || fail "package index must expose capability-scoped signing requests"
grep -Fq "export * from './settlement-allocation-lineage-checkpoint-trust.js';" "$index" \
  || fail "package index must expose signer trust rotation and receipt verification"
grep -Fq "export * from './settlement-allocation-lineage-checkpoint-trust-anchor.js';" "$index" \
  || fail "package index must expose root-attested trust verification"

grep -Fq "recordType: 'settlement_allocation_lineage_checkpoint_signer_trust_bundle_v1'" "$trust" \
  || fail "signer trust bundle must have a domain-separated commitment"
grep -Fq "recordType: 'settlement_allocation_lineage_checkpoint_verification_receipt_v1'" "$trust" \
  || fail "checkpoint verification receipts must have a domain-separated commitment"
grep -Fq 'checkpoint signer trust policy must be observed through checkpoint signedAt' "$trust" \
  || fail "verification must reject trust policy snapshots stale at signedAt"
grep -Fq 'signedMs >= Date.parse(entry.validUntil)' "$trust" \
  || fail "signer trust validity must use a half-open end boundary"
grep -Fq 'signedMs >= Date.parse(entry.revokedAt)' "$trust" \
  || fail "revocation must block signatures at and after the cutoff"
grep -Fq 'checkpoint signer trust bundle contains overlapping authority windows' "$trust" \
  || fail "same-authority overlapping trust windows must fail closed"
grep -Fq 'trustBundleRoot: bundle.bundleRoot' "$trust" \
  || fail "verification receipt must bind exact trust bundle root"
grep -Fq 'trustEntryId: entry.entryId' "$trust" \
  || fail "verification receipt must identify the exact trust entry"

grep -Fq 'SETTLEMENT_ALLOCATION_LINEAGE_TRUST_BUNDLE_ATTESTATION_SCOPE' "$trust_anchor" \
  || fail "trust bundle root-attestation scope missing"
grep -Fq "recordType: 'settlement_allocation_lineage_trust_bundle_attestation_request_v1'" "$trust_anchor" \
  || fail "trust bundle attestation request must be domain-separated"
grep -Fq "recordType: 'settlement_allocation_lineage_trust_bundle_attestation_v1'" "$trust_anchor" \
  || fail "trust bundle attestation must have a domain-separated root"
grep -Fq 'trust bundle policy sequence is below anti-rollback floor' "$trust_anchor" \
  || fail "trust anchor must enforce an externally pinned anti-rollback floor"
grep -Fq 'trust bundle attestation successor must advance policy sequence exactly once' "$trust_anchor" \
  || fail "trust bundle succession must be contiguous"
grep -Fq 'trust bundle attestation successor must bind predecessor bundle root' "$trust_anchor" \
  || fail "trust bundle succession must bind predecessor root"
grep -Fq 'Date.parse(signedAt) >= Date.parse(canonical.expiresAt)' "$trust_anchor" \
  || fail "trust bundle attestation signing window must be half-open"
grep -Fq 'attachSettlementAllocationLineageTrustBundleDetachedSignature' "$trust_anchor" \
  || fail "root signer must remain detached from accounting key custody"
grep -Fq 'const VERIFIED_ANCHORED_CHECKPOINT_AUTHORITIES = new WeakSet<object>();' "$trust_anchor" \
  || fail "anchored checkpoint authorities must carry private runtime provenance"
grep -Fq 'requireAnchoredSettlementAllocationLineageCheckpointAuthority' "$trust_anchor" \
  || fail "anchored checkpoint authority promotion accessor missing"
grep -Fq "recordType: 'settlement_allocation_lineage_checkpoint_anchored_verification_receipt_v1'" "$trust_anchor" \
  || fail "anchored checkpoint verification must emit a domain-separated receipt"
grep -Fq 'trustBundleAttestationRoot: trustBundleAuthority.attestation.attestationRoot' "$trust_anchor" \
  || fail "anchored receipt must bind exact trust bundle attestation"
grep -Fq 'trustPolicySequence: trustBundleAuthority.policySequence' "$trust_anchor" \
  || fail "anchored receipt must bind exact trust policy sequence"

# Statements may consume allocation/discharge semantics only with root-anchored
# trust authority for the exact signed checkpoint envelope and exact statement instant.
grep -Fq 'readonly anchoredTrust?: AnchoredSettlementAllocationLineageCheckpointAuthority;' "$projection" \
  || fail "statement settlement evidence must carry root-anchored trust authority"
grep -Fq 'checkpoint-backed allocation lineage requires anchored trust authority' "$projection" \
  || fail "statement compiler must reject unanchored allocation lineage"
grep -Fq 'requireAnchoredSettlementAllocationLineageCheckpointAuthority' "$projection" \
  || fail "statement compiler must require resolver-minted anchored trust authority"
grep -Fq 'anchored trust authority is not bound to the lineage signed checkpoint envelope' "$projection" \
  || fail "statement compiler must bind anchored trust to exact signed checkpoint envelope"
grep -Fq "evidenceKind: 'settlement_execution_v7'" "$projection" \
  || fail "statement settlement commitment must use root-trust-aware v7 evidence domain"
grep -Fq 'checkpointVerificationReceiptRoot: validatedAllocation?.checkpointVerificationReceiptRoot' "$projection" \
  || fail "statement settlement root must commit checkpoint verification receipt"
grep -Fq 'anchoredVerificationReceiptRoot: validatedAllocation?.anchoredVerificationReceiptRoot' "$projection" \
  || fail "statement settlement root must commit anchored verification receipt"
grep -Fq 'trustBundleRoot: validatedAllocation?.trustBundleRoot' "$projection" \
  || fail "statement settlement root must commit signer trust bundle root"
grep -Fq 'trustBundleAttestationRoot: validatedAllocation?.trustBundleAttestationRoot' "$projection" \
  || fail "statement settlement root must commit root-attested trust bundle"
grep -Fq 'trustAnchorId: validatedAllocation?.trustAnchorId' "$projection" \
  || fail "statement settlement root must identify trust anchor"
grep -Fq 'trustPolicySequence: validatedAllocation?.trustPolicySequence' "$projection" \
  || fail "statement settlement root must commit anti-rollback policy sequence"
grep -Fq 'rejects checkpoint-backed lineage without root-anchored trust authority' "$projection_tests" \
  || fail "statement regressions must reject unanchored checkpoint lineage"
grep -Fq 'rejects a structural clone of a genuine anchored trust authority' "$projection_tests" \
  || fail "statement regressions must reject fabricated anchored trust"
grep -Fq 'changes settlement commitment when root trust-policy evidence changes' "$projection_tests" \
  || fail "statement regressions must bind trust policy evidence into settlement root"

grep -Fq 'rejects a cryptographically valid older trust bundle below the pinned rollback floor' "$trust_anchor_tests" \
  || fail "trust-anchor regressions must exercise rollback attack"
grep -Fq 'requires exact predecessor continuity and one-step policy advancement' "$trust_anchor_tests" \
  || fail "trust-anchor regressions must prove contiguous policy succession"
grep -Fq 'rejects a successor that names the wrong predecessor bundle root' "$trust_anchor_tests" \
  || fail "trust-anchor regressions must reject predecessor substitution"
grep -Fq 'uses half-open attestation request windows' "$trust_anchor_tests" \
  || fail "trust-anchor regressions must prove expiry boundary"
grep -Fq 'does not let structural clones participate in verified policy succession' "$trust_anchor_tests" \
  || fail "trust-anchor regressions must reject fabricated verified authority"
grep -Fq 'rejects fabricated anchored checkpoint authority objects' "$trust_anchor_tests" \
  || fail "trust-anchor regressions must reject fabricated anchored checkpoint authorities"

grep -Fq 'preserves historical old-key verification after rotation and emits an auditable receipt' "$trust_tests" \
  || fail "regressions must prove historical verification survives rotation"
grep -Fq 'accepts the rotated key after its authority window begins' "$trust_tests" \
  || fail "regressions must prove new key acceptance after rotation"
grep -Fq 'rejects an old-key signature after the trust window expires' "$trust_tests" \
  || fail "regressions must reject retired-key signatures"
grep -Fq 'treats revocation prospectively: pre-cutoff signatures survive, post-cutoff signatures fail' "$trust_tests" \
  || fail "regressions must prove prospective revocation semantics"
grep -Fq 'rejects stale trust policy snapshots that do not reach checkpoint signedAt' "$trust_tests" \
  || fail "regressions must reject stale trust bundles"
grep -Fq 'rejects overlapping authority windows for the same signer capability identity' "$trust_tests" \
  || fail "regressions must reject ambiguous trust windows"
grep -Fq 'rejects trust-bundle tampering and binds receipt identity to the exact bundle root' "$trust_tests" \
  || fail "regressions must bind verification receipts to exact trust policy"

grep -Fq 'hands a compiler-minted request to an external signer without private-key custody' "$tests" \
  || fail "regressions must prove external detached signer flow"
grep -Fq 'rejects capabilities that do not authorize the exact source or validity window' "$tests" \
  || fail "regressions must attack signer capability scope/time"
grep -Fq 'rejects detached signatures rebound to another request or capability' "$tests" \
  || fail "regressions must attack detached signature rebinding"
grep -Fq 'rejects persisted allocation content changed behind its authority root' "$tests" \
  || fail "regressions must attack persisted allocation tampering"
grep -Fq 'requires the exact semantic replay context for every allocation' "$tests" \
  || fail "regressions must cover missing replay context"
grep -Fq 'rejects duplicate, reordered and over-high-water ingestion cursors' "$tests" \
  || fail "regressions must cover source-cursor tampering"
grep -Fq 'requires a successor link cursor to follow both endpoint allocations' "$tests" \
  || fail "regressions must prove endpoint-before-link ingestion"

grep -Fq "'mycelix-music/scripts/check-checkpoint-compiler-invariants.sh'" "$workflow" \
  || fail "accounting workflow must trigger when compiler invariants change"
grep -Fq 'check-checkpoint-compiler-invariants.sh' ../mycelix-music/package.json \
  || fail "root invariant command must execute compiler invariants"
grep -Fq 'A serialized source snapshot is signing authority without canonical accounting replay.' "$policy" \
  || fail "promotion policy must forbid signing raw source snapshots as authority"
grep -Fq 'Database or checkpoint compiler access implies possession of checkpoint signing keys.' "$policy" \
  || fail "promotion policy must forbid conflating compiler/database access with key custody"
grep -Fq 'A stale signer trust bundle predating the checkpoint signature is sufficient revocation evidence.' "$policy" \
  || fail "promotion policy must forbid stale trust policy evidence"
grep -Fq 'Key rotation invalidates all historically valid checkpoints by default.' "$policy" \
  || fail "promotion policy must preserve historical signatures under time-bounded trust"
grep -Fq 'An unattested signer trust bundle is sufficient root trust authority.' "$policy" \
  || fail "promotion policy must require trust-bundle root attestation"
grep -Fq 'A cryptographically valid trust bundle below the pinned minimum policy sequence is acceptable.' "$policy" \
  || fail "promotion policy must forbid trust-policy rollback"
grep -Fq 'A checkpoint-backed allocation lineage without root-anchored signer trust is sufficient statement discharge authority.' "$policy" \
  || fail "promotion policy must forbid unanchored statement discharge authority"

echo "checkpoint compiler invariants passed"
