#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "checkpoint compiler invariant failed: $1" >&2
  exit 1
}

compiler=packages/accounting/src/settlement-allocation-lineage-checkpoint-compiler.ts
checkpoint=packages/accounting/src/settlement-allocation-lineage-checkpoint.ts
trust=packages/accounting/src/settlement-allocation-lineage-checkpoint-trust.ts
tests=packages/accounting/src/settlement-allocation-lineage-checkpoint-compiler.test.ts
trust_tests=packages/accounting/src/settlement-allocation-lineage-checkpoint-trust.test.ts
index=packages/accounting/src/index.ts
workflow=../.github/workflows/music-accounting.yml
policy=packages/accounting/promotion-policy.json

[[ -f "$compiler" ]] || fail "checkpoint compiler source is missing"
[[ -f "$checkpoint" ]] || fail "checkpoint verification source is missing"
[[ -f "$trust" ]] || fail "checkpoint signer trust source is missing"
[[ -f "$tests" ]] || fail "checkpoint compiler regressions are missing"
[[ -f "$trust_tests" ]] || fail "checkpoint signer trust regressions are missing"

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

echo "checkpoint compiler invariants passed"
