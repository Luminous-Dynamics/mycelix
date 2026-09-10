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

# Historical Prisma payments are rail receipts/projections, never the debt ledger.
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

# Creator accounting authority tables must remain immutable snapshots/events.
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

# Settlement execution must retain the exact eligibility snapshot that admitted
# the obligation set. Compiler-only outcomes are never source observations.
settlement_record=$(sed -n '/^model SettlementAttemptObservationRecord {/,/^}/p' packages/database/prisma/schema.prisma)
for field in eligibilityAsOf eligibilityEvidenceRoot; do
  grep -Eq "^[[:space:]]+${field}[[:space:]]" <<<"$settlement_record" \
    || fail "settlement persistence missing eligibility snapshot field: $field"
  grep -Fq "$field" packages/database/src/accounting-authority.ts \
    || fail "append store drops eligibility snapshot field: $field"
done

for compiler_only in below_threshold awaiting_eligibility_evidence; do
  if grep -Fq "'$compiler_only'" packages/database/src/accounting-authority.ts; then
    fail "compiler-only eligibility state leaked into database source vocabulary: $compiler_only"
  fi
done

grep -Fq 'isObservableSettlementEligibilityCode' packages/accounting/src/persistence-projection.ts \
  || fail "accounting persistence projection must reject compiler-only eligibility states"

# Direct SQL must enforce the same source/shape/causality/finality boundary as application code.
for constraint in \
  royalty_deduction_canonical_shape \
  royalty_eligibility_digest_shape \
  royalty_eligibility_observable_code \
  royalty_obligation_canonical_shape \
  royalty_statement_canonical_shape \
  royalty_statement_period_order \
  settlement_finalized_requires_receipt \
  settlement_observation_after_eligibility \
  settlement_observation_digest_shape \
  settlement_observation_state_code
do
  grep -Fq "$constraint" packages/database/prisma/accounting-append-only.sql \
    || fail "creator accounting database constraint missing: $constraint"
done
grep -Fq 'CHECK ("observedAt" >= "eligibilityAsOf")' packages/database/prisma/accounting-append-only.sql \
  || fail "settlement persistence must reject evidence that predates eligibilityAsOf"
grep -Fq 'btrim("railReceiptRef")' packages/database/prisma/accounting-append-only.sql \
  || fail "settlement finality must require a non-empty durable rail receipt"
grep -Fq "'below_threshold'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt a compiler-only eligibility insert"
grep -Fq "'teleported'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt an invalid settlement state"
grep -Fq "'not-a-digest'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt a malformed commitment root"
grep -Fq "'guard:obligation:negative'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt negative obligation money"
grep -Fq "'guard:statement:nonconserving'" packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must attempt a non-conserving statement"
grep -Fq 'settlement_finalized_requires_receipt' packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must prove receipt-backed settlement finality"
grep -Fq 'settlement_observation_after_eligibility' packages/database/src/accounting-append-only.integration.ts \
  || fail "live database regression must prove settlement causality enforcement"

if grep -Fq 'git+ssh://' package-lock.json; then
  fail "package lock contains an SSH-only dependency"
fi

grep -Fq 'GNU AGPL v3 or later' README.md \
  || fail "README license does not match the repository license"

echo "source invariants passed"
