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

if grep -Fq 'git+ssh://' package-lock.json; then
  fail "package lock contains an SSH-only dependency"
fi

grep -Fq 'GNU AGPL v3 or later' README.md \
  || fail "README license does not match the repository license"

echo "source invariants passed"
