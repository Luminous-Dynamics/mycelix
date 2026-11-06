# Payment Initiation Composition Evaluator

Independent Rust reference evaluator for normalised outputs from the payment-initiation assurance components.

## Run

```sh
cargo fmt --manifest-path tools/finance/payment-initiation-composition-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/payment-initiation-composition-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/payment-initiation-composition-evaluator/Cargo.toml -- --self-test
```

The evaluator does not implement JWS verification, request-digest canonicalisation, VoP matching, fraud adjudication, payer-comprehension experiments or payment recourse. It checks the composition boundary over results emitted by those independently qualified components.

## Important limits

- Output is an assurance vector, never a global trust score.
- A VoP/account-name match is not identity, invoice legitimacy or collection authority.
- Authenticated commercial evidence is not a legal debt determination.
- An unadjudicated fraud assertion is not a finding of fraud.
- Payer acceptance is not payment execution or settlement.
- All fixtures are synthetic and external effects are disabled.
