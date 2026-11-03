# SAP Payment Saga Model v1

**Status:** pure Rust reference model; not wired to production Finance and not evidence of distributed atomicity.

## Why a saga/journal, not a transaction flag

The current `send_payment` source path mutates the sender SAP balance, mutates the recipient SAP balance, attempts Treasury fee delivery, then writes Payment/Receipt and indexes. These are distinct effects. A failure after any one can leave a partial economic state. Treasury fee delivery is currently non-fatal in this path. The payment ID is generated from sender plus the current timestamp rather than a durable client operation key.

A database-style atomic transaction across agents/source chains is not assumed. The design is a recoverable journal: each monetary leg has a stable operation-derived effect ID, immutable request fingerprint, source evidence and an explicit pending/committed/indeterminate disposition.

## Modeled effect order

1. immutable sender-scoped operation claim;
2. payer authorisation of exact intent;
3. sender debit;
4. recipient credit;
5. Treasury fee allocation (or explicit NotRequired when fee is zero);
6. Payment record evidence;
7. signed Receipt evidence;
8. local ledger operation Completed.

Every retry reuses the same effect ID. Same ID plus changed intent/evidence conflicts. An unknown result remains Indeterminate until reconciled against the original effect identity.

## Model guarantees

- At most one journal per `(sender, client idempotency key)` inside the in-memory registry model.
- Exact retry returns the existing operation; changed intent conflicts.
- Debit must commit before credit can prepare; credit must commit before fee; all money legs must be satisfied before Payment record; Payment record before Receipt.
- A fee failure blocks Payment record completion.
- `Completed` is unreachable until all required money legs plus Payment and Receipt evidence are committed.
- Prepared/indeterminate legs retain one stable effect ID; duplicate identical commit is idempotent; changed evidence conflicts.
- Once a value-moving effect may have happened, the model refuses to reject and erase the operation.
- Zero fee is explicit `NotRequired`, not a fake fee transfer.

## Tests

The unit tests cover exact retry, changed-payload conflict, overflow, zero amount, ordering, debit/credit/fee partial failures, indeterminate recovery, evidence failure, receipt gating, stable operation IDs, duplicate commits and concurrent access to the in-memory reference registry.

## Critical limitations

This model does **not** prove Holochain distributed uniqueness or compare-and-swap across agents. The test using a mutex proves only that this in-memory registry serializes its own threads. Production must implement the operation claim at the authoritative integrity boundary and prove singleton cardinality under competing source-chain/DHT creates and forks.

It does not implement the actual ledger, signatures, custody, Treasury worker, fee split, compensation/reversal policy or bank settlement. A production integration requires #4491, #4559, #627, #4965 and #5716 to be satisfied together.

## Run

```sh
cargo fmt --manifest-path tools/finance/sap-payment-saga-model/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sap-payment-saga-model/Cargo.toml
cargo clippy --manifest-path tools/finance/sap-payment-saga-model/Cargo.toml --all-targets -- -D warnings
```
