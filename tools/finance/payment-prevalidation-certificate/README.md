# Payment Pre-validation Certificate Evaluator

Independent, effects-disabled Rust evaluator for the synthetic certificate corpus.

## Run

```sh
cargo fmt --manifest-path tools/finance/payment-prevalidation-certificate/Cargo.toml -- --check
cargo test --manifest-path tools/finance/payment-prevalidation-certificate/Cargo.toml
cargo run --manifest-path tools/finance/payment-prevalidation-certificate/Cargo.toml -- --self-test
```

The evaluator consumes upstream intent, payee-resolution, VoP/CoP, provider-JWS trust, participant limit/fee, presentation and consent evidence. It checks cross-binding and source/profile/freshness policy. It does not reimplement QR+, the upstream intent digest, VoP matching, provider JWS verification, directory lookup or payment dispatch. Provider trust and intent states in fixtures are synthetic upstream assertions, not cryptographic verification.

`prevalidated_for_psp_review` is not dispatch permission. This crate has no network client and no external-effect API.
