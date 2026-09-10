#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "source invariant failed: $1" >&2
  exit 1
}

grep -Fq "const allowAnyOrigin = String(process.env.ALLOW_ANY_ORIGIN || 'false')" apps/api/src/index.ts \
  || fail "credentialed CORS must default to the allowlist"

if grep -Fq "process.env.ENABLE_CORS || 'true'" apps/api/src/index.ts; then
  fail "legacy allow-any-origin CORS default returned"
fi

grep -Fq 'default = []' apps/leptos/Cargo.toml \
  || fail "the Leptos production build must not enable fixtures"

if grep -Eq '^default[[:space:]]*=.*fixtures' apps/leptos/Cargo.toml; then
  fail "fixtures cannot be a default Leptos feature"
fi

for message in \
  'Deposit verification unavailable:' \
  'Cashout unavailable:' \
  'Transfers unavailable:'
do
  grep -Fq "$message" dnas/mycelix-music/zomes/balances/coordinator/src/lib.rs \
    || fail "economic mutation gate missing: $message"
done

play_input=$(sed -n '/pub struct RecordPlayInput {/,/^}/p' dnas/mycelix-music/zomes/plays/coordinator/src/lib.rs)
for forbidden in artist song_duration strategy_id amount_owed; do
  if grep -Fq "$forbidden" <<<"$play_input"; then
    fail "listener-controlled play field returned: $forbidden"
  fi
done

legacy_payment=$(sed -n '/^model RoyaltyPayment {/,/^}/p' packages/database/prisma/schema.prisma)
[[ -n "$legacy_payment" ]] || fail "legacy RoyaltyPayment model unexpectedly missing"
if grep -Eq '^[[:space:]]+(status|payoutStatus|owedAmount|debtBalance|settled)[[:space:]]' <<<"$legacy_payment"; then
  fail "legacy RoyaltyPayment must not acquire obligation/status authority"
fi
grep -Fq "authority: 'receipt_projection_only'" packages/accounting/src/legacy.ts \
  || fail "legacy royalty payments must remain receipt-only evidence"
grep -Fq 'railCoversBatchGross' packages/accounting/src/recovery.ts \
  || fail "settlement recovery must report rail coverage rather than debt discharge"
if grep -Fq 'obligationSetSettled' packages/accounting/src/recovery.ts; then
  fail "rail recovery must not claim obligation-set settlement authority"
fi
grep -Fq 'compileRoyaltyStatement' packages/accounting/src/projection.ts \
  || fail "royalty statements must be compiled projections"

# Paid value must derive from durable rail-finalized receipt evidence, never from
# a parallel caller-supplied amount. Rail finality and economic discharge are
# deliberately separate authorities.
statement_settlement=$(sed -n '/^export interface StatementSettlementEvidence {/,/^}/p' packages/accounting/src/projection.ts)
[[ -n "$statement_settlement" ]] || fail "statement settlement evidence interface missing"
if grep -Fq 'settledAmount' <<<"$statement_settlement"; then
  fail "statement compiler must not accept caller-supplied settledAmount authority"
fi
grep -Fq 'finalSettledAmount' packages/accounting/src/projection.ts \
  || fail "statement paid projection must derive from reconstructed final receipt amount"
grep -Fq "'partial_finality'" packages/accounting/src/recovery.ts \
  || fail "rail-finalized partial payment must remain distinct from whole-batch rail coverage"
grep -Fq 'cannot exceed batch gross amount' packages/accounting/src/recovery.ts \
  || fail "rail-finalized amount must never exceed deterministic batch gross"
grep -Fq 'receipt-backed settlement amounts exceed statement net payable value' packages/accounting/src/projection.ts \
  || fail "statement paid value must remain bounded by statement net payable"

# Only the conservation-bound allocation authority may claim economic discharge.
grep -Fq 'createSettlementAllocationAuthority' packages/accounting/src/settlement-allocation.ts \
  || fail "settlement allocation authority constructor missing"
grep -Fq 'obligationSetDischarged' packages/accounting/src/settlement-allocation.ts \
  || fail "settlement allocation must own obligation-set discharge authority"
grep -Fq 'must conserve batch gross = creator paid + deductions + residual held' packages/accounting/src/settlement-allocation.ts \
  || fail "settlement allocation conservation theorem missing"
grep -Fq 'nonzero settlement residual requires residualAuthorityRef' packages/accounting/src/settlement-allocation.ts \
  || fail "nonzero settlement residual must retain authority provenance"

# Statements may consume allocation semantics only through a resolver-minted,
# complete allocation lineage for the exact statement asOf. Direct allocation
# input is rejected at runtime even from structurally permissive JS callers.
if grep -Fq 'readonly allocation?:' <<<"$statement_settlement"; then
  fail "statement compiler must not accept direct allocation authority"
fi
grep -Fq 'readonly allocationLineage?: SettlementAllocationLineageResolution;' <<<"$statement_settlement" \
  || fail "statement settlement evidence must carry allocation lineage rather than direct allocation"
grep -Fq 'direct settlement allocation evidence is forbidden; provide resolver-minted allocationLineage' packages/accounting/src/projection.ts \
  || fail "statement compiler must reject legacy direct allocation evidence at runtime"
grep -Fq 'requireCanonicalSettlementAllocationHead' packages/accounting/src/projection.ts \
  || fail "statement compiler must require resolver-minted canonical allocation head"
grep -Fq 'settlement allocation lineage boundary asOf must equal statement asOf' packages/accounting/src/projection.ts \
  || fail "statement compiler must bind allocation lineage completeness to exact statement asOf"
grep -Fq "evidenceKind: 'settlement_execution_v5'" packages/accounting/src/projection.ts \
  || fail "statement settlement commitment must use lineage-aware v5 evidence domain"
grep -Fq 'allocationLineageRoot: evidence.allocationLineage?.lineageRoot' packages/accounting/src/projection.ts \
  || fail "statement settlement root must commit allocation lineage root"
grep -Fq 'allocationBoundaryRoot: evidence.allocationLineage?.boundary.boundaryRoot' packages/accounting/src/projection.ts \
  || fail "statement settlement root must commit allocation completeness boundary"
grep -Fq 'allocationHeadRoot: allocation?.allocationRoot' packages/accounting/src/projection.ts \
  || fail "statement settlement root must commit canonical allocation head"
grep -Fq 'obligationSetDischarged: allocation?.obligationSetDischarged' packages/accounting/src/projection.ts \
  || fail "statement settlement root must derive discharge only from canonical allocation head"

# Allocation lineage must be explicit, monotonic, completeness-bound and sealed
# against structurally fabricated TypeScript resolution objects.
grep -Fq 'const VERIFIED_LINEAGE_RESOLUTIONS = new WeakSet<object>();' packages/accounting/src/settlement-allocation-lineage.ts \
  || fail "allocation lineage canonical resolutions must carry private runtime provenance"
grep -Fq "recordType: 'settlement_allocation_successor_link_v1'" packages/accounting/src/settlement-allocation-lineage.ts \
  || fail "allocation successor links must have a domain-separated commitment"
grep -Fq "recordType: 'settlement_allocation_lineage_boundary_v1'" packages/accounting/src/settlement-allocation-lineage.ts \
  || fail "allocation lineage completeness boundary commitment missing"
grep -Fq 'complete settlement allocation lineage boundary must be observed through asOf' packages/accounting/src/settlement-allocation-lineage.ts \
  || fail "complete lineage coverage must reach requested asOf"
grep -Fq 'settlement allocation successor must strictly reduce residual held value' packages/accounting/src/settlement-allocation-lineage.ts \
  || fail "allocation successors must make strict residual progress"
grep -Fq 'settlement allocation lineage fork detected' packages/accounting/src/settlement-allocation-lineage.ts \
  || fail "allocation lineage must fail closed on forks"
grep -Fq 'settlement allocation lineage join detected' packages/accounting/src/settlement-allocation-lineage.ts \
  || fail "allocation lineage must fail closed on joins"
grep -Fq "recordType: 'settlement_allocation_lineage_v2'" packages/accounting/src/settlement-allocation-lineage.ts \
  || fail "allocation lineage root must bind completeness-aware v2 semantics"
grep -Fq 'boundaryRoot: boundary.boundaryRoot' packages/accounting/src/settlement-allocation-lineage.ts \
  || fail "allocation lineage root must bind source completeness boundary"
grep -Fq 'VERIFIED_LINEAGE_RESOLUTIONS.has(resolution as object)' packages/accounting/src/settlement-allocation-lineage.ts \
  || fail "canonical allocation head promotion must reject fabricated resolution objects"
grep -Fq 'canonical head requires complete source coverage' packages/accounting/src/settlement-allocation-lineage.ts \
  || fail "provisional allocation lineage must never become canonical discharge authority"

# Deductions are canonical root-bearing accounting authorities before persistence.
grep -Fq "export type StatementDeduction = RoyaltyDeductionAuthority;" packages/accounting/src/projection.ts \
  || fail "statement deduction must be the canonical deduction authority type"
grep -Fq 'createRoyaltyDeductionAuthority' packages/accounting/src/deduction-authority.ts \
  || fail "canonical deduction authority constructor missing"
grep -Fq 'assertRoyaltyDeductionAuthority' packages/accounting/src/deduction-authority.ts \
  || fail "deduction authority root verifier missing"
grep -Fq "recordType: 'royalty_deduction_v1'" packages/accounting/src/deduction-authority.ts \
  || fail "deduction authority root domain missing"
grep -Fq "evidenceKind: 'royalty_deduction_authority_v1'" packages/accounting/src/projection.ts \
  || fail "statement adjustment root must commit deduction authorities"
grep -Fq 'deductionRoot: deduction.deductionRoot' packages/accounting/src/projection.ts \
  || fail "statement adjustment root must bind exact deduction roots"
deduction_projector=$(sed -n '/^export function projectDeductionRecord(/,/^): Readonly<PersistedDeductionRecord> {/p' packages/accounting/src/persistence-projection.ts)
[[ -n "$deduction_projector" ]] || fail "deduction persistence projector signature missing"
if grep -Fq 'authorityRef:' <<<"$deduction_projector"; then
  fail "deduction persistence must not accept a parallel authorityRef argument"
fi
grep -Fq 'assertRoyaltyDeductionAuthority(deduction)' packages/accounting/src/persistence-projection.ts \
  || fail "deduction persistence must verify precomputed authority"
grep -Fq 'deductionRoot: deduction.deductionRoot' packages/accounting/src/persistence-projection.ts \
  || fail "deduction persistence must serialize the accounting authority root"
grep -Fq 'createRoyaltyDeductionAuthority' packages/accounting/src/persistence-verification.ts \
  || fail "deduction replay must reconstruct canonical authority"

for model in \
  RoyaltyObligationRecord \
  RoyaltyEligibilityObservation \
  RoyaltyDeductionRecord \
  SettlementAttemptObservationRecord \
  RoyaltyStatementSnapshotRecord
do
  block=$(sed -n "/^model ${model} {/,/^}/p" packages/database/prisma/schema.prisma)
  [[ -n "$block" ]] || fail "creator accounting authority model missing: $model"
  if grep -Eq '^[[:space:]]+updatedAt[[:space:]]' <<<"$block"; then
    fail "append-only creator accounting model cannot contain updatedAt: $model"
  fi
  grep -Fq "'$model'" packages/database/prisma/accounting-append-only.sql \
    || fail "append-only PostgreSQL guard missing for $model"
done

grep -Fq 'BEFORE UPDATE OR DELETE' packages/database/prisma/accounting-append-only.sql \
  || fail "accounting append-only database trigger must reject UPDATE and DELETE"
grep -Fq 'test:accounting-guards' packages/database/package.json \
  || fail "database package must expose the live append-only guard regression"
grep -Fq 'postgres:16' ../.github/workflows/music-accounting-persistence.yml \
  || fail "persistence qualification must exercise PostgreSQL"
grep -Fq 'test:accounting-guards' ../.github/workflows/music-accounting-persistence.yml \
  || fail "persistence qualification must prove runtime mutation rejection"

settlement_record=$(sed -n '/^model SettlementAttemptObservationRecord {/,/^}/p' packages/database/prisma/schema.prisma)
for field in eligibilityAsOf eligibilityEvidenceRoot settledAmountMinor settledCurrency; do
  grep -Eq "^[[:space:]]+${field}[[:space:]]" <<<"$settlement_record" \
    || fail "settlement persistence missing evidence field: $field"
  grep -Fq "$field" packages/database/src/accounting-authority.ts \
    || fail "append store drops settlement evidence field: $field"
done

for compiler_only in below_threshold awaiting_eligibility_evidence; do
  if grep -Fq "'$compiler_only'" packages/database/src/accounting-authority.ts; then
    fail "compiler-only eligibility state leaked into database source vocabulary: $compiler_only"
  fi
done

grep -Fq 'isObservableSettlementEligibilityCode' packages/accounting/src/persistence-projection.ts \
  || fail "accounting persistence projection must reject compiler-only eligibility states"
grep -Fq 'predates obligation' packages/accounting/src/settlement.ts \
  || fail "eligibility reconstruction must reject evidence that predates its debt"
grep -Fq 'settlement_attempt_observation_v3' packages/accounting/src/persistence-projection.ts \
  || fail "receipt amount must be covered by a versioned settlement observation commitment"
grep -Fq 'verifyPersistedSettlementRecovery' packages/accounting/src/persistence-verification.ts \
  || fail "persisted settlement observations must replay into authoritative recovery"

for constraint in \
  royalty_deduction_canonical_shape \
  royalty_eligibility_digest_shape \
  royalty_eligibility_observable_code \
  royalty_obligation_canonical_shape \
  royalty_statement_canonical_shape \
  royalty_statement_period_order \
  settlement_finality_evidence_shape \
  settlement_observation_after_eligibility \
  settlement_observation_digest_shape \
  settlement_observation_state_code
do
  grep -Fq "$constraint" packages/database/prisma/accounting-append-only.sql \
    || fail "creator accounting database constraint missing: $constraint"
done
grep -Fq 'royalty_eligibility_causality_guard' packages/database/prisma/accounting-append-only.sql \
  || fail "database must enforce eligibility observation chronology against its obligation"
grep -Fq 'CHECK ("observedAt" >= "eligibilityAsOf")' packages/database/prisma/accounting-append-only.sql \
  || fail "settlement persistence must reject evidence that predates eligibilityAsOf"
grep -Fq 'btrim("railReceiptRef")' packages/database/prisma/accounting-append-only.sql \
  || fail "settlement finality must require a non-empty durable rail receipt"
grep -Fq '"settledAmountMinor" IS NOT NULL' packages/database/prisma/accounting-append-only.sql \
  || fail "settlement finality must require exact settled amount evidence"
grep -Fq '"settledAmountMinor" IS NULL' packages/database/prisma/accounting-append-only.sql \
  || fail "non-final settlement observations must not carry settled value"
grep -Fq "'below_threshold'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt a compiler-only eligibility insert"
grep -Fq "'guard:eligibility:predates'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt pre-obligation eligibility evidence"
grep -Fq "'teleported'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt an invalid settlement state"
grep -Fq "'not-a-digest'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt a malformed commitment root"
grep -Fq "'guard:obligation:negative'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt negative obligation money"
grep -Fq "'guard:statement:nonconserving'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt a non-conserving statement"
grep -Fq "'guard:settlement:no-amount'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt receipt-only finality without amount"
grep -Fq "'guard:settlement:nonfinal-amount'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must reject settled value on non-final evidence"
grep -Fq 'settlement_finality_evidence_shape' packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must prove receipt-and-amount-backed settlement finality"
grep -Fq 'settlement_observation_after_eligibility' packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must prove settlement causality enforcement"

# Settlement allocation is persisted as append-only authority evidence, but the
# deterministic batch remains the sole source of batch gross for conservation replay.
allocation_model=$(sed -n '/^model SettlementAllocationRecord {/,/^}/p' packages/database/prisma/schema.prisma)
[[ -n "$allocation_model" ]] || fail "settlement allocation persistence model missing"
grep -Eq '^[[:space:]]+allocationId[[:space:]]+String[[:space:]]+@id' <<<"$allocation_model" \
  || fail "settlement allocation canonical identity must be allocationId"
if grep -Eq '^[[:space:]]+updatedAt[[:space:]]' <<<"$allocation_model"; then
  fail "settlement allocation authority must remain append-only"
fi
if grep -Fq 'batchGrossMinor' <<<"$allocation_model"; then
  fail "settlement allocation persistence must not duplicate deterministic batch gross authority"
fi
for field in obligationSetRoot eligibilityAsOf eligibilityEvidenceRoot railReceiptRef creatorPaidMinor deductionRoots deductionTotalMinor residualHeldMinor allocatedAt obligationSetDischarged allocationRoot; do
  grep -Eq "^[[:space:]]+${field}[[:space:]]" <<<"$allocation_model" \
    || fail "settlement allocation persistence missing field: $field"
done
grep -Fq 'projectSettlementAllocationRecord' packages/accounting/src/persistence-projection.ts \
  || fail "settlement allocation persistence projector missing"
grep -Fq 'assertSettlementAllocationAuthority(allocation, { batch, recovery, deductions })' packages/accounting/src/persistence-projection.ts \
  || fail "allocation persistence must verify semantic authority before serialization"
grep -Fq 'verifyPersistedSettlementAllocation' packages/accounting/src/persistence-verification.ts \
  || fail "settlement allocation canonical replay verifier missing"
grep -Fq 'persisted allocation deduction roots do not match authoritative deductions' packages/accounting/src/persistence-verification.ts \
  || fail "allocation replay must bind the exact deduction authority set"
grep -Fq 'appendSettlementAllocation' packages/database/src/accounting-authority.ts \
  || fail "database append boundary missing settlement allocation support"
grep -Fq 'digestArray' packages/database/src/accounting-authority.ts \
  || fail "allocation append boundary must enforce canonical deduction-root arrays"
grep -Fq 'discharge state must equal residualHeldMinor == 0' packages/database/src/accounting-authority.ts \
  || fail "allocation append boundary must enforce residual/discharge equivalence"
grep -Fq 'canonical_sha256_json_array' packages/database/prisma/accounting-append-only.sql \
  || fail "PostgreSQL must enforce sorted unique allocation deduction roots"
grep -Fq 'settlement_allocation_canonical_shape' packages/database/prisma/accounting-append-only.sql \
  || fail "PostgreSQL allocation canonical-shape constraint missing"
grep -Fq 'settlement_allocation_append_only_guard' packages/database/prisma/accounting-append-only.sql \
  || fail "PostgreSQL allocation append-only trigger missing"
grep -Fq 'BEFORE UPDATE OR DELETE ON "SettlementAllocationRecord"' packages/database/prisma/accounting-append-only.sql \
  || fail "allocation table must reject UPDATE and DELETE"
grep -Fq 'accounting-allocation-authority.test.ts' packages/database/package.json \
  || fail "database test command must include allocation append-boundary regressions"
grep -Fq 'accounting-allocation.integration.ts' packages/database/package.json \
  || fail "database live guard command must include allocation PostgreSQL regressions"
for attack in bad-root unsorted duplicate zero-payment residual-no-authority false-discharge predates; do
  grep -Fq "guard:allocation:${attack}" packages/database/src/accounting-allocation.integration.ts \
    || fail "live allocation database attack missing: $attack"
done
grep -Fq "column_name = 'batchGrossMinor'" packages/database/src/accounting-allocation.integration.ts \
  || fail "live allocation regression must prove batch gross authority is not duplicated"

if grep -Fq 'git+ssh://' package-lock.json; then
  fail "package lock contains an SSH-only dependency"
fi

grep -Fq 'GNU AGPL v3 or later' README.md \
  || fail "README license does not match the repository license"

echo "source invariants passed"
