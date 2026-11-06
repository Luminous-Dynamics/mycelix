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

The 33 cases and 33 deterministic negative mutations are synthetic conformance fixtures. The tool does not parse actual QR+ payloads, access provider directories, authenticate users, or dispatch payments.

## Claim ceiling

Passing the evaluator establishes only that this synthetic corpus and its semantic mutation controls behave as specified. It is not QR+ conformance, provider interoperability, recipient verification, PayShap access, payment settlement or production safety.


## Verification of Payee / Confirmation of Payee

The 14-case extension qualifies provider-reported `match`, `close_match`, `no_match`, `unavailable` and `other` outcomes. Results bind the exact opaque account subject, payment-intent subject, issuer, participant, audience, freshness interval, account type, payer presentation and source profile. An exact name match does not establish collection authority or debt legitimacy. Client-side fuzzy matching is not treated as provider verification. All cases remain effects-disabled.

Comparators are source-dated EU VoP and UK CoP frameworks. SARB's South African NPU/VoP proposal is not treated as evidence of a live nationwide service. The Rust evaluator qualifies fixture-declared signature state and issuer/audience binding; it does not itself verify cryptographic JWS/signature bytes or establish issuer-key trust.
