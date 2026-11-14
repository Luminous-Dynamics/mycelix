# Payment Initiation Assurance Evaluator

Effects-disabled Rust reference evaluator for a synthetic QR+/payment-intent adversarial corpus.

## Run

```sh
cargo fmt --manifest-path tools/finance/payment-initiation-assurance/Cargo.toml -- --check
cargo test --manifest-path tools/finance/payment-initiation-assurance/Cargo.toml
cargo run --manifest-path tools/finance/payment-initiation-assurance/Cargo.toml
```

## What it checks

- QR parsing is not payee authentication or payment authority.
- Provider/directory payee resolution and audience are separate evidence.
- The payer must authorise the exact payee, amount, currency/scale, provider/rail and intent digest.
- Static QR without an amount requires explicit payer entry.
- Changed amount/payee/recipient invalidates previous consent.
- Exact replay is idempotent; same operation identity with a changed fingerprint conflicts.
- Expired payloads, invalid signatures, wrong provider audience and stale directory responses fail closed.
- Payee-resolution rate limits prevent directory enumeration.
- Provider acceptance is not settlement or recipient availability.
- Unknown external outcomes retain their original operation identity and are queried/reconciled rather than blindly retried.
- Published QR+ standard is not evidence that the operational pilot or merchant implementation is complete.

The 19 cases and 16 deterministic negative mutations are synthetic conformance fixtures. The tool does not parse actual QR+ payloads, access provider directories, authenticate users, or dispatch payments.

## Claim ceiling

Passing the evaluator establishes only that this synthetic corpus and its semantic mutation controls behave as specified. It is not QR+ conformance, provider interoperability, recipient verification, PayShap access, payment settlement or production safety.
