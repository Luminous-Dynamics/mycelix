#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "checkpoint compiler invariant failed: $1" >&2
  exit 1
}

compiler=packages/accounting/src/settlement-allocation-lineage-checkpoint-compiler.ts
tests=packages/accounting/src/settlement-allocation-lineage-checkpoint-compiler.test.ts
index=packages/accounting/src/index.ts
workflow=../.github/workflows/music-accounting.yml
policy=packages/accounting/promotion-policy.json

[[ -f "$compiler" ]] || fail "checkpoint compiler source is missing"
[[ -f "$tests" ]] || fail "checkpoint compiler regressions are missing"

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
grep -Fq 'must be produced by canonical source compiler before signing' "$compiler" \
  || fail "strict signer must reject caller-fabricated compiler candidates"
grep -Fq 'signSettlementAllocationLineageCheckpoint(compiled.checkpoint' "$compiler" \
  || fail "strict signing surface must sign only the compiler-minted checkpoint"

# The raw source snapshot/compiler path must not itself own database credentials or
# signer keys. DB capture is #563's boundary; signing is an explicit later capability.
if grep -Eq 'PrismaClient|\$transaction|DATABASE_URL|privateKeyPem.*snapshot|snapshot.*privateKeyPem' "$compiler"; then
  fail "checkpoint compiler must remain independent of database connection and signer-key custody"
fi

grep -Fq "export * from './settlement-allocation-lineage-checkpoint-compiler.js';" "$index" \
  || fail "checkpoint compiler must be exported through the accounting package"

grep -Fq 'rejects persisted allocation content changed behind its authority root' "$tests" \
  || fail "regressions must attack persisted allocation tampering"
grep -Fq 'requires the exact semantic replay context for every allocation' "$tests" \
  || fail "regressions must cover missing replay context"
grep -Fq 'rejects duplicate, reordered and over-high-water ingestion cursors' "$tests" \
  || fail "regressions must cover source-cursor tampering"
grep -Fq 'requires a successor link cursor to follow both endpoint allocations' "$tests" \
  || fail "regressions must prove endpoint-before-link ingestion"
grep -Fq 'signs only a compiler-minted candidate through the strict signing surface' "$tests" \
  || fail "regressions must reject structural signing candidates"

grep -Fq "'mycelix-music/scripts/check-checkpoint-compiler-invariants.sh'" "$workflow" \
  || fail "accounting workflow must trigger when compiler invariants change"
grep -Fq 'check-checkpoint-compiler-invariants.sh' ../mycelix-music/package.json \
  || fail "root invariant command must execute compiler invariants"
grep -Fq 'A serialized source snapshot is signing authority without canonical accounting replay.' "$policy" \
  || fail "promotion policy must forbid signing raw source snapshots as authority"

echo "checkpoint compiler invariants passed"
