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
grep -Fq 'obligationSetSettled: true' packages/accounting/src/recovery.ts \
  || fail "settlement finality must be reconstructed from durable evidence"
grep -Fq 'compileRoyaltyStatement' packages/accounting/src/projection.ts \
  || fail "royalty statements must be compiled projections"

# Paid value must derive from durable rail-finalized receipt evidence, never from
# a parallel caller-supplied amount. Rail finality may be partial; debt settlement
# may not silently erase the unpaid residual.
statement_settlement=$(sed -n '/^export interface StatementSettlementEvidence {/,/^}/p' packages/accounting/src/projection.ts)
[[ -n "$statement_settlement" ]] || fail "statement settlement evidence interface missing"
if grep -Fq 'settledAmount' <<<"$statement_settlement"; then
  fail "statement compiler must not accept caller-supplied settledAmount authority"
fi
grep -Fq 'finalSettledAmount' packages/accounting/src/projection.ts \
  || fail "statement paid projection must derive from reconstructed final receipt amount"
grep -Fq "'partial_finality'" packages/accounting/src/recovery.ts \
  || fail "rail-finalized partial payment must remain distinct from whole-batch debt settlement"
grep -Fq 'cannot exceed batch gross amount' packages/accounting/src/recovery.ts \
  || fail "rail-finalized amount must never exceed deterministic batch gross"
grep -Fq 'receipt-backed settlement amounts exceed statement net payable value' packages/accounting/src/projection.ts \
  || fail "statement paid value must remain bounded by statement net payable"

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

if grep -Fq 'git+ssh://' package-lock.json; then
  fail "package lock contains an SSH-only dependency"
fi

grep -Fq 'GNU AGPL v3 or later' README.md \
  || fail "README license does not match the repository license"

echo "source invariants passed"
