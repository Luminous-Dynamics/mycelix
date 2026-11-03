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

## Typed effect-subject checks

A leg cannot be committed with a bare non-empty string anymore. Each commit carries an `EffectEvidence` value containing the operation ID, stable effect ID, leg, sender, recipient, principal, fee, currency, a SHA-256-shaped evidence reference, and a leg-specific subject.

The model checks the exact effect contract:

- **Sender debit:** sender account, recipient counterparty, debit direction, and principal + fee.
- **Recipient credit:** recipient account, sender counterparty, credit direction, and principal only.
- **Treasury fee:** the named global fee-pool account, sender counterparty, credit direction, and exact fee.
- **Payment record:** non-empty Payment action reference and `Completed` status, with all common intent fields matched.
- **Receipt:** distinct receipt action reference, the exact committed Payment action, sender signer, the v1 signing-profile identifier, and a SHA-256-shaped signature-evidence reference.

The negative controls mutate movement amount, account, counterparty, currency, operation ID and direction; attempt Pending/Failed Payment evidence; and substitute the Payment action, signer, receipt action or signing profile. All must be rejected before the modeled commit.

**Important evidence boundary:** the reference model checks the shape and semantic agreement of supplied evidence. It does not compute or verify that an evidence digest is the digest of those fields, verify an Ed25519 signature, fetch a Holochain action, prove predecessor authority, or independently validate a ledger movement. Production integrity code must resolve and validate the referenced source records and proofs. A string beginning with `sha256:` is not by itself evidence of authenticity.

## Tests

The 23 unit tests cover exact retry, changed-payload conflict, overflow, zero amount, ordering, debit/credit/fee partial failures, indeterminate recovery, typed amount/account/counterparty/currency/direction binding, Payment status, receipt Payment-action and signer binding, evidence-reference shape, receipt gating, stable operation IDs, duplicate commits and concurrent access to the in-memory reference registry.

## Critical limitations

This model does **not** prove Holochain distributed uniqueness or compare-and-swap across agents. The test using a mutex proves only that this in-memory registry serializes its own threads. Production must implement the operation claim at the authoritative integrity boundary and prove singleton cardinality under competing source-chain/DHT creates and forks.

It does not implement the actual ledger, cryptographic digest recomputation, signature verification, Holochain/DHT source-record resolution, custody, Treasury worker, fee split, compensation/reversal policy or bank settlement. A production integration requires #4491, #4559, #627, #4965 and #5716 to be satisfied together.

## Run

```sh
cargo fmt --manifest-path tools/finance/sap-payment-saga-model/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sap-payment-saga-model/Cargo.toml
cargo clippy --manifest-path tools/finance/sap-payment-saga-model/Cargo.toml --all-targets -- -D warnings
```
